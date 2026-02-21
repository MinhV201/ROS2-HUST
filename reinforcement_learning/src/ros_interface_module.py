import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32MultiArray
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy



class RosHandler(Node):
    def __init__(self):
        super().__init__('rl_ros_handler')
        self.lidar = None
        self.odom = None
        self.depth = None
        self.tf = None
        
        
        qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_tf = self.create_subscription(TFMessage, '/tf', callback=self.tf_callback, qos_profile=qos_policy)
        self.sub_odom = self.create_subscription(Odometry, '/odom', callback=self.odom_callback, qos_profile=qos_policy)
        self.sub_lidar = self.create_subscription(LaserScan, '/lidar', callback=self.lidar_callback, qos_profile=qos_policy)
        self.sub_depth = self.create_subscription(Image, '/depth/image_raw', callback=self.depth_callback, qos_profile=qos_policy)

        #self.mode_control = self.create_client(WorldControl, '/world/small_maze/control')

        #while not self.mode_control.wait_for_service(timeout_sec=1.0):
            #self.get_logger().info("Waiting for /world/small_maze/control service...")

        self.pub_result = self.create_publisher(Float32MultiArray, 'result', 10)
        self.pub_get_action = self.create_publisher(Float32MultiArray, 'get_action', 10)
    def tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == "base_link" and transform.header.frame_id == "odom":
                self.tf = transform
        
        

    def odom_callback(self, msg):
        self.odom = msg
        
    def lidar_callback(self, lidar_raw):
        #print(f"Lidar Update: {min(lidar_raw.ranges)}")
        #print(f"Type of msg: {type(lidar_raw)}")
        self.lidar = lidar_raw
        
            
        
    def depth_callback(self, depth_raw):
        self.depth = depth_raw
        
'''
    def _reset_done(self, future):
        try:
            result = future.result()
            self.get_logger().info("Reset simulation OK!")
        except Exception as e:
            self.get_logger().error(f"Reset failed: {e}")
'''
    



