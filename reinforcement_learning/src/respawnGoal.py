#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import random
import time
import os
from geometry_msgs.msg import Pose
from src.ros_interface_module import RosHandler

class Respawn():
    def __init__(self, handle: RosHandler):
        self.model_path = os.path.expanduser('~/ros2_ws/src/reinforcement_learning/src/models/goal_box/goal.sdf')
        self.goal_position = Pose()
        self.init_goal_x = -1.0  # random goal position
        self.init_goal_y = 0.0
        # self.init_goal_x = 1.65  # fixed goal position
        # self.init_goal_y = 2.0
        self.goal_position.position.x = self.init_goal_x
        self.goal_position.position.y = self.init_goal_y
        self.modelName = 'goal'
        self.obstacle_1 = 0.633707, 1.26704
        self.obstacle_2 = 0.938707, 0.967037
        self.obstacle_3 = 2.61794, -0.80051
        self.obstacle_4 = -0.291293, -1.00051
        self.obstacle_5 = -2.223, -2.49812
        self.obstacle_6 = 1.0, -2.51219
        self.obstacle_7 = 2.53, 2.53
        self.obstacle_8 = -1.42, 0.95
        self.obstacle_9 = 0.008707, 2.92449
        self.obstacle_10 = 2.93371, -0.000509
        self.obstacle_11 = -2.91629, -0.000509
        self.obstacle_12 = 0.008707, -2.92551
        self.random_goal = True
        # self.random_goal = False  # fixed goal
        self.last_goal_x = self.init_goal_x
        self.last_goal_y = self.init_goal_y
        self.last_index = 0
        #self.sub_model = self.create_subscription(ModelStates, '/world/small_maze/state', self.checkModel, 10)
        #self.mode_control = self.create_publisher(WorldControl, '/world/small_maze/state')
        self.check_model = False
        self.index = 0

        self.handler = handle

    def checkModel(self, model):
        try:
            output = subprocess.check_output(
                ['ign', 'model', '--list'],
                stderr=subprocess.STDOUT,
            ).decode("utf-8")
            if model in output:
                self.handler.get_logger().info(f"Model '{model}' found via CLI.")
                self.check_model = True
            else:
                self.check_model = False
        except subprocess.CalledProcessError as e:
            self.handler.get_logger().error(f"Failed to check models: {e}")
            self.check_model = False

    def respawnModel(self):
        self.checkModel(self.modelName)
        if not self.check_model:
            subprocess.run([
            "ros2", "run", "ros_gz_sim", "create",
            "-world", "small_maze",      
            "-name", self.modelName,        
            "-file", self.model_path,          
            "-x", str(self.goal_position.position.x),       
            "-y", str(self.goal_position.position.y),       
            "-z", "0.02"                   
            ])
            self.handler.get_logger().info("Goal respawned at : %.1f, %.1f" % (self.goal_position.position.x, self.goal_position.position.y))
        else:
            cmd = [
            "ign", "service", "-s", "/world/small_maze/set_pose",
            "--reqtype", "ignition.msgs.Pose",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "2000",
            "--req", 
            f'name: "{self.modelName}", position: {{x: {self.goal_position.position.x}, y: {self.goal_position.position.y}, z: {0.02}}}, orientation: {{w: 1.0}}'
            ]
            try:
                subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
                self.handler.get_logger().info("Goal position : %.1f, %.1f" % (self.goal_position.position.x, self.goal_position.position.y))
            except subprocess.CalledProcessError:
                self.handler.get_logger().error("Failed to move goal")

    def deleteModel(self):
        
        self.checkModel(model=self.modelName)
        if not self.check_model:
            subprocess.run([
            "ros2", "run", "ros_gz_sim", "delete",
            "-name", self.modelName,
            "-world", "small_maze"                                   
            ])
        else:
            pass

    def getPosition(self, position_check=False, delete=False):
        if delete:
            self.deleteModel()
            return

        if self.random_goal:
            while position_check:
                goal_x = random.randrange(-24, 24) / 10.0
                goal_y = random.randrange(-24, 24) / 10.0
                if abs(goal_x - self.obstacle_1[0]) <= 0.85 and abs(goal_y - self.obstacle_1[1]) <= 0.85:
                    position_check = True
                elif abs(goal_x - self.obstacle_2[0]) <= 0.85 and abs(goal_y - self.obstacle_2[1]) <= 0.85:
                    position_check = True
                elif abs(goal_x - self.obstacle_3[0]) <= 0.85 and abs(goal_y - self.obstacle_3[1]) <= 0.85:
                    position_check = True
                elif abs(goal_x - self.obstacle_4[0]) <= 0.85 and abs(goal_y - self.obstacle_4[1]) <= 0.85:
                    position_check = True
                elif abs(goal_x - self.obstacle_5[0]) <= 0.85 and abs(goal_y - self.obstacle_5[1]) <= 0.85:
                    position_check = True
                elif abs(goal_x - self.obstacle_6[0]) <= 0.85 and abs(goal_y - self.obstacle_6[1]) <= 0.85:
                    position_check = True
                elif abs(goal_x - self.obstacle_7[0]) <= 0.85 and abs(goal_y - self.obstacle_4[1]) <= 0.85:
                    position_check = True
                elif abs(goal_x - self.obstacle_8[0]) <= 0.85 and abs(goal_y - self.obstacle_8[1]) <= 0.85:
                    position_check = True
                elif abs(goal_x - self.obstacle_9[0]) <= 0.85 and abs(goal_y - self.obstacle_9[1]) <= 0.85:
                    position_check = True
                elif abs(goal_x - self.obstacle_10[0]) <= 0.85 and abs(goal_y - self.obstacle_10[1]) <= 0.85:
                    position_check = True
                elif abs(goal_x - self.obstacle_11[0]) <= 0.85 and abs(goal_y - self.obstacle_11[1]) <= 0.85:
                    position_check = True
                elif abs(goal_x - self.obstacle_12[0]) <= 0.85 and abs(goal_y - self.obstacle_12[1]) <= 0.85:
                    position_check = True
                elif abs(goal_x + 2.3) <= 0.85 and abs(goal_y + 1.0 ) <= 0.85:
                    position_check = True
                else:
                    position_check = False

                if abs(goal_x - self.last_goal_x) < 1 and abs(goal_y - self.last_goal_y) < 1:
                    position_check = True

                self.goal_position.position.x = goal_x
                self.goal_position.position.y = goal_y

        time.sleep(0.5)
        self.respawnModel()

        self.last_goal_x = self.goal_position.position.x
        self.last_goal_y = self.goal_position.position.y

        return self.goal_position.position.x, self.goal_position.position.y
    
