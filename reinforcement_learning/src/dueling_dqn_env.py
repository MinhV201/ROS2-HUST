#!/usr/bin/env python3
import numpy as np
import math
import time
from math import *
from src.respawnGoal import Respawn
from src.ros_interface_module import RosHandler
from geometry_msgs.msg import Twist, Pose, Point
import rclpy
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import Buffer
from cv_bridge import CvBridge
import subprocess
import cv2
from setup_config import *

class Env():
    def __init__(self, action_size, handle: RosHandler):
        self.init_x = init_x   #| init  
        self.init_y = init_y   #| 
        self.goal_x = goal_x   #| goal
        self.goal_y = goal_y   #|
        self.init_z = 0.1
        self.heading = 0
        self.action_size = action_size
        self.initGoal = True
        self.get_goalbox = False
        self.prev_distance = 0
        # self.k_r = 2 
        # self.k_alpha = 15 
        # self.k_beta = -3
        self.const_vel = 0.5   #0.25
        # self.goal_dist_thres = 0.2  #0.55
        # self.goal_angle_thres = 15 #degrees
        self.current_theta = 0
        self.goal_counters = 0
        self.enable_feedback_control = False
        self.safe_dist = 1.0
        self.lidar = []
        self.position = Pose()
        self.self_rotation_z_speed=0
        self.linearx = 0
        self.lineary = 0
        #self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        #self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        
        #self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        #self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        #self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.handler = handle
        self.original_sub_goals = [
            (-0.2288, 0.0494),
            (1.8688, 0.0494),
            (1.5421, 1.9965),
            (0.9210, 3.4790)
        ]
        self.current_sub_goals = list(self.original_sub_goals)
        self.past_distance = 0.0
        


    def getOdometry(self, odom, tf):
        self.linearx = odom.twist.twist.linear.x
        self.lineary = odom.twist.twist.linear.y
        self.self_rotation_z = odom.twist.twist.angular.z
        self.position = tf.transform.translation
        orientation = tf.transform.rotation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, cur_theta = euler_from_quaternion(orientation_list)
        self.current_theta = cur_theta #radian
        return self.position.x, self.position.y, self.current_theta

    def getState(self, scan,image):
        done = False
        min_range = 0.2
        scan_range = []

        bridge = CvBridge()
        image=bridge.imgmsg_to_cv2(image, desired_encoding="32FC1")
        image = np.nan_to_num(image, nan=0.0, posinf=8.0, neginf=0.0)

        # Clip depth range (D435i max ~10m)
        image = np.clip(image, 0.0, 3.0)
        image = (image / 3.0).astype(np.float32)

        
        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])
        #print('Min range: ',min(scan_range))
        if min_range > min(scan_range) > 0:
            done = True
        
        return image , done

    def setReward(self, done, action):
        
        # Tính khoảng cách hiện tại đến mục tiêu chính
        current_distance = math.sqrt((self.position.x - self.goal_x)**2 + (self.position.y - self.goal_y)**2)
        reward = 0.0
    
        if done:
            print('Current_distance: ', current_distance)
            if current_distance <= 0.2:  # Ngưỡng để xác định đã đến đích chính
                self.handler.get_logger().info("Goal reached!")
                reward += 100.0  # Phần thưởng khi đến đích chính
                self.goal_counters += 1
            else:
                self.handler.get_logger().info("Done, but not goal (no penalty for collision).")
                reward -= current_distance
            self.handler.pub_cmd_vel.publish(Twist())  # Dừng robot
        else:
            # Kiểm tra xem robot có đi qua điểm đích phụ không
            for i, (sub_x, sub_y) in enumerate(self.current_sub_goals):
                sub_distance = math.sqrt((self.position.x - sub_x)**2 + (self.position.y - sub_y)**2)
                if sub_distance <= 0.3: # Ngưỡng để xác định đã đến điểm đích phụ
                    try:
                        real_idx = self.original_sub_goals.index((sub_x, sub_y))
                    except ValueError:
                        real_idx = 0  
                    self.handler.get_logger().info(f"Sub-goal {real_idx+1} reached!")
                    reward += 20.0*(real_idx+1)  # Phần thưởng khi đi qua mỗi điểm đích phụ
                    self.current_sub_goals.pop(i)  # Loại bỏ điểm đích phụ đã đạt được để không thưởng lại
                    break
        
        return reward, self.goal_counters


    def step(self, action):
        max_angular_vel = 0.75  #1.5 0.5
        ang_vel = ((self.action_size - 1)/2 - action) * max_angular_vel * 0.5

        vel_cmd = Twist()
        vel_cmd.linear.x = self.const_vel
        vel_cmd.angular.z = ang_vel
        self.handler.pub_cmd_vel.publish(vel_cmd)
        
        
        data = None
        odom = None
        data1= None
        tf = None
        start_wait = time.time()
        
        while self.handler.depth is None or self.handler.lidar is None or self.handler.odom is None:
            if time.time() - start_wait > 2.0: # Timeout 2 giây
                print("[WARN] Sensor data timeout in step()!")
                break
            time.sleep(0.01)
        
      
        data = self.handler.depth
        data1 = self.handler.lidar
        odom = self.handler.odom
        tf = self.handler.tf
        
        state, done = self.getState(data1,data)

        reward, counters = self.setReward( done,action)

        return np.asarray(state), reward, done, counters

    def reset(self):
        '''
        subprocess.run([
            "ign", "service",
            "-s", "/world/small_maze/control",
            "--reqtype", "ignition.msgs.WorldControl",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "3000",
            "--req", "reset: {all: true}"
        ])
        '''
        subprocess.run([
        "ign", "service",
        "-s", "/world/small_maze/set_pose",
        "--reqtype", "ignition.msgs.Pose",
        "--reptype", "ignition.msgs.Boolean",
        "--timeout", "2000",
        "--req", f'name: "my_robot", position: {{x: {-2.0}, y: {0}, z: 0.2}}, orientation: {{w: 1.0}}'
        
        ])
        
        print(f"[RESET] Goal position: ({self.goal_x}, {self.goal_y})")

        
        time.sleep(0.5)
        self.current_sub_goals = list(self.original_sub_goals)
        self.handler.lidar = None
        self.handler.depth = None
        self.handler.odom = None
        self.handler.tf = None
        
        #print("Waiting for fresh data after reset...")
        while self.handler.depth is None or self.handler.lidar is None or self.handler.odom is None:
            time.sleep(0.05)
          
        self.handler.get_logger().info('All data get ready!')
        
        # Lấy tham chiếu dữ liệu
        data = self.handler.depth
        data1 = self.handler.lidar
        odom = self.handler.odom
        tf = self.handler.tf
        
        if self.initGoal:
            
            self.initGoal = False

        self.init_x, self.init_y, self.current_theta = self.getOdometry(odom, tf)
        
   
        state, done = self.getState(data1,data)
 

        return np.asarray(state)
