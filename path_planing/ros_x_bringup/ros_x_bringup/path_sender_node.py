import rclpy
from rclpy.node import Node
import time
import threading
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

from ros_x.srv import SetPath
from ros_x.msg import PathSegment, PathDef

class PathSenderNode(Node):
    def __init__(self):
        super().__init__('path_sender_node')
        self.cli = self.create_client(SetPath, 'set_path')
        self.debug_pub = self.create_publisher(PathDef, '/debug/path', 10)
        self.create_subscription(Bool, '/completed', self.finished_callback, 10)

        self.robot_is_busy = False
        self.current_direction = "FORWARD"
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for PF node...')


    def finished_callback(self, msg):
        if msg.data:
            self.get_logger().info('Robot reached the goal!')
            time.sleep(2.0)
            self.robot_is_busy = False

    def create_path_forward(self):
        segs = []
        
        s1 = PathSegment()
        s1.type = PathSegment.TYPE_LINE
        s1.start = Point(x=-2.0, y=0.0)
        s1.end   = Point(x=0.9605, y=0.0)
        segs.append(s1)

        
        return segs

    def create_path_backward(self):
        segs = []
        s_spin = PathSegment()
        s_spin.type = PathSegment.TYPE_SPIN
        s_spin.start = Point(x=0.9605, y=0.0, z=0.0)
        s_spin.end   = Point(x=-2.0, y=0.0, z=0.0) 
        segs.append(s_spin)
        s1 = PathSegment()
        s1.type = PathSegment.TYPE_LINE
        s1.start = Point(x=0.9605, y=0.0)
        s1.end   = Point(x=-2.0, y=0.0)
        segs.append(s1)

        return segs

    def send_path(self, segments):
        req = SetPath.Request()
        req.segments = segments
        req.lookahead_distance = 0.5
        self.cli.call_async(req)
        
        # Gửi debug
        d_msg = PathDef()
        d_msg.segments = segments
        self.debug_pub.publish(d_msg)
        
        self.robot_is_busy = True # Đánh dấu robot đang bận

    def patrol_loop(self):
        
        self.get_logger().info("----(PATROL MODE)----")
        
        while rclpy.ok():
            if not self.robot_is_busy:
                if self.current_direction == "FORWARD":
                    self.get_logger().info("Command: Move (A -> B)")
                    path = self.create_path_forward()
                    self.send_path(path)
                    self.current_direction = "BACKWARD" 
                
                else:
                    self.get_logger().info("Command: Move (B -> A)")
                    path = self.create_path_backward()
                    self.send_path(path)
                    self.current_direction = "FORWARD" 
            
            
            time.sleep(1.0)

def main(args=None):
    rclpy.init(args=args)
    node = PathSenderNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.patrol_loop() 
    except KeyboardInterrupt:
        pass
        
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()