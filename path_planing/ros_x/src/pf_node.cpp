#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/utils.h" 
#include "std_msgs/msg/bool.hpp"

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "ros_x/PF_algorithm.h"
#include "ros_x/PathLibs.h"
#include "ros_x/PID_Controller.h"
#include "ros_x/srv/set_path.hpp" 
#include "ros_x/msg/path_segment.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class PathFollowingNode : public rclcpp::Node {
public:
    PathFollowingNode() : Node("path_following_node") {

        los_controller_ = std::make_shared<path_following::LosController>();
        los_controller_->setLookAheadDist(0.5); 
        los_controller_->buildTestPath();

        path_ready_ = true;

        pid_yaw_ = std::make_shared<PIDController>(1.5, 0.0, 0.1, -1.0, 1.0);
        pid_vel_ = std::make_shared<PIDController>(1.0, 0.0, 0.0, 0.0, 0.5);

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        path_viz_pub_ = this->create_publisher<nav_msgs::msg::Path>("/viz/path", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&PathFollowingNode::odom_callback, this, _1));

        path_srv_ = this->create_service<ros_x::srv::SetPath>(
            "set_path", std::bind(&PathFollowingNode::set_path_callback, this, _1, _2));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        finished_pub = this->create_publisher<std_msgs::msg::Bool>("/completed", 10);
        
        RCLCPP_INFO(this->get_logger(), "Path Following Node Started!");
    }

private:

    std::shared_ptr<path_following::LosController> los_controller_;
    std::shared_ptr<PIDController> pid_yaw_;
    std::shared_ptr<PIDController> pid_vel_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_viz_pub_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Service<ros_x::srv::SetPath>::SharedPtr path_srv_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finished_pub;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    bool path_ready_ = false;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!path_ready_) return;

        double x = 0.0;
        double y = 0.0;
        double robot_yaw = 0.0;
        try {
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                "odom", "base_link", tf2::TimePointZero);

            x = t.transform.translation.x;
            y = t.transform.translation.y;

            tf2::Quaternion q(
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w);
            robot_yaw = tf2::impl::getYaw(q);

        } catch (const tf2::TransformException & ex) {
            // RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
            x = msg->pose.pose.position.x;
            y = msg->pose.pose.position.y;
            
            tf2::Quaternion q_bk(
                msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
            robot_yaw = tf2::impl::getYaw(q_bk);
        }
    
        auto result = los_controller_->update(x, y);

        if (result.end_of_path) {
            stop_robot();
            std_msgs::msg::Bool status_msg;
            status_msg.data = true;
            finished_pub->publish(status_msg);
            path_ready_ = false;
            RCLCPP_INFO(this->get_logger(), "Reached Goal! Waiting for new command...");
            return;
        }

        double yaw_error = result.target_heading - robot_yaw;
        yaw_error = los_controller_->normalizeAngle(yaw_error);

        double target_v = 0.0;
        double target_w = 0.0;
        double dt = 0.02; 

        if (result.is_spot_turn) {
            target_v = 0.0;
            double kp_spin = 1.0;
            target_w = kp_spin * yaw_error; 
            
            double min_speed = 0.2; 
            if (std::abs(yaw_error) > 0.002) { 
                if (std::abs(target_w) < min_speed) {
                    target_w = (target_w > 0) ? min_speed : -min_speed;
                }
            }

            if (target_w > 0.4) target_w = 0.4;
            if (target_w < -0.4) target_w = -0.4;
         
            if (std::abs(yaw_error) < 0.002) { 
                target_w = 0.0; 
                los_controller_->forceNextSegment();
                RCLCPP_INFO(this->get_logger(), "SPIN PERFECT! Error: %.5f", yaw_error);
            }
        }
        else {
            target_w = pid_yaw_->calculate(yaw_error, 0.0, dt);
            if (std::abs(yaw_error) > 0.5) target_v = 0.1; 
            else target_v = 0.3; 
        }

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = target_v;
        cmd.angular.z = target_w;
        cmd_pub_->publish(cmd);
    }

    void set_path_callback(const std::shared_ptr<ros_x::srv::SetPath::Request> request,
                           std::shared_ptr<ros_x::srv::SetPath::Response> response) {
        
        los_controller_->clearPath();
        
        // --- VISUALIZATION
        nav_msgs::msg::Path path_viz_msg;
        path_viz_msg.header.frame_id = "odom"; 
        path_viz_msg.header.stamp = this->now();
        // --------------------------------------------

        for (const auto & seg_msg : request->segments) {
            ::Point p_start = {seg_msg.start.x, seg_msg.start.y};
            ::Point p_end   = {seg_msg.end.x, seg_msg.end.y};

           
            if (seg_msg.type == ros_x::msg::PathSegment::TYPE_LINE) {
                los_controller_->addSegment(new LineSegment(p_start, p_end));
            } 
            else if (seg_msg.type == ros_x::msg::PathSegment::TYPE_ARC) {
                TurnDirection dir = (seg_msg.direction == ros_x::msg::PathSegment::DIR_LEFT) ? LEFT : RIGHT;
                los_controller_->addSegment(new ArcSegment(p_start, p_end, seg_msg.radius, dir));
            }
            else if (seg_msg.type == ros_x::msg::PathSegment::TYPE_SPIN) {
                los_controller_->addSegment(new SpinSegment(p_start, p_end));
            }

            
            geometry_msgs::msg::PoseStamped pose_st;
            pose_st.header = path_viz_msg.header;
   
            pose_st.pose.position.x = seg_msg.start.x;
            pose_st.pose.position.y = seg_msg.start.y;
            pose_st.pose.orientation.w = 1.0; 
            path_viz_msg.poses.push_back(pose_st);

      
            if (seg_msg.type != ros_x::msg::PathSegment::TYPE_SPIN) {
                pose_st.pose.position.x = seg_msg.end.x;
                pose_st.pose.position.y = seg_msg.end.y;
                path_viz_msg.poses.push_back(pose_st);
            }
        }
        
        if (!request->segments.empty()) { 
            path_ready_ = true;  
            response->success = true;
            los_controller_->reset(); 
           
            path_viz_pub_->publish(path_viz_msg);


            RCLCPP_INFO(this->get_logger(), "Loaded path. Published to /viz/path");
        } else {
            path_ready_ = false;
            response->success = false;
            RCLCPP_WARN(this->get_logger(), "Received empty path!");
        }
    }

    void stop_robot() {
        geometry_msgs::msg::Twist cmd;
        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollowingNode>());
    rclcpp::shutdown();
    return 0;
}