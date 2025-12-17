#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ros_x.msg import PathDef, PathSegment 
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import numpy as np

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer_node')
        

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PathDef, '/debug/path', self.path_callback, 10)
        
        self.robot_x = []
        self.robot_y = []
        
        self.ref_x = []
        self.ref_y = []
        self.path_received = False

    def odom_callback(self, msg):
        try:
           
            t = self.tf_buffer.lookup_transform(
                'odom',         
                'base_link',   
                rclpy.time.Time()) 

        
            current_x = t.transform.translation.x
            current_y = t.transform.translation.y

            self.robot_x.append(current_x)
            self.robot_y.append(current_y)

        except TransformException as ex:
            pass

    def path_callback(self, msg):
        self.get_logger().info(f'Received Path: {len(msg.segments)} segments')
        
        new_ref_x = []
        new_ref_y = []
        
        for seg in msg.segments:
            if seg.type == PathSegment.TYPE_LINE:
                new_ref_x.extend([seg.start.x, seg.end.x, np.nan])
                new_ref_y.extend([seg.start.y, seg.end.y, np.nan])
                
            elif seg.type == PathSegment.TYPE_ARC:
                new_ref_x.extend([seg.start.x, seg.end.x, np.nan])
                new_ref_y.extend([seg.start.y, seg.end.y, np.nan])
                
            elif seg.type == PathSegment.TYPE_SPIN:

                new_ref_x.extend([seg.start.x, seg.start.x, np.nan]) 
                new_ref_y.extend([seg.start.y, seg.start.y, np.nan])

        self.ref_x = new_ref_x
        self.ref_y = new_ref_y
        self.path_received = True

def update_plot(frame, node, line_actual, line_ref):
    if node.robot_x:
        line_actual.set_data(node.robot_x, node.robot_y)
    
    if node.path_received:
        line_ref.set_data(node.ref_x, node.ref_y)
    
    # Auto-scale (Tùy chọn)
    # try:
    #     if len(node.robot_x) > 0:
    #         ax = line_actual.axes
    #         ax.set_xlim(node.robot_x[-1] - 2, node.robot_x[-1] + 2)
    #         ax.set_ylim(node.robot_y[-1] - 2, node.robot_y[-1] + 2)
    # except:
    #     pass

    return line_actual, line_ref

def main():
    rclpy.init()
    node = PathVisualizer()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    fig, ax = plt.subplots(figsize=(8, 8))

    ax.set_xlim(-3, 3) 
    ax.set_ylim(-3, 3)
    ax.grid(True)
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.set_title("----MAP ROBOT PATH----")
    ax.axis('equal')

    line_ref, = ax.plot([], [], 'r--', linewidth=5.0, alpha = 0.5, label='Reference Path') 
    line_actual, = ax.plot([], [], 'b-', linewidth=2, label='Robot Path')
    
    ax.legend(loc='upper right')

    ani = FuncAnimation(fig, update_plot, fargs=(node, line_actual, line_ref), interval=100)
    plt.show()
    rclpy.shutdown()

if __name__ == '__main__':
    main()