#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"


class Safety : public rclcpp::Node {
// The class that handles emergency braking

public:
    Safety()
    : rclcpp::Node("safety_node")
    , speed_(0)
    {
        /*
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */
       
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

        /// TODO: create ROS subscribers and publishers
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", qos,
            [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
                scan_callback(msg);
            }
        );
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", qos,
            [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                odom_callback(msg);
            }
        );
        drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

        time_pb1_ = declare_parameter<double>("time_pb1", 1.0);
        scale_pb1_ = declare_parameter<double>("scale_pb1", 0.8);
        time_pb2_ = declare_parameter<double>("time_pb2", 0.5);
        scale_pb2_ = declare_parameter<double>("scale_pb2", 0.5);
        time_fb_ = declare_parameter<double>("time_fb", 2.0);
        safety_buffer_dist_ = declare_parameter<double>("safety_buffer_dist", 0.5*0.3302);  // wheelbase/2
        min_speed_ = declare_parameter<double>("min_speed", 0.1);
    }

private:
    /// TODO: create ROS subscribers and publishers

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// TODO: update current speed
        speed_ = msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        if (std::abs(speed_) < min_speed_) {
            // don't do anything, we are recovering
            return;
        }
        
        auto const &angle_min = scan_msg->angle_min;
        auto const &angle_increment = scan_msg->angle_increment;
        
        /// TODO: calculate TTC
        auto ttc = std::numeric_limits<double>::max();

        auto const &ranges = scan_msg->ranges;
        for (size_t i = 0; i < ranges.size(); ++i) {
            auto const &range = ranges[i];
            if (range < safety_buffer_dist_) {
                ttc = 0;
                break;
            }
            auto const angle = angle_min + i * angle_increment; 
            auto const r_dot = speed_ * std::cos(angle);
            if (r_dot >= 0) {
                // car is not approaching the object
                continue;
            }
            auto const ttc_tmp = std::max(0., range / (-r_dot));

            if (ttc_tmp < ttc) {
                ttc = ttc_tmp;
            } 
        }

        RCLCPP_DEBUG(get_logger(), "TTC = %f seconds.", ttc);
        /// TODO: publish drive/brake message
        if (ttc < time_fb_) {
            RCLCPP_WARN(get_logger(), "Imminent collision detected: ttc = %f seconds.", ttc);
            ackermann_msgs::msg::AckermannDriveStamped msg;
            msg.header.stamp = now();
            msg.drive.speed = 0;
            drive_pub_->publish(msg);
        }
    }

    double speed_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    double time_pb1_;
    double scale_pb1_;
    double time_pb2_;
    double scale_pb2_;
    double time_fb_;
    double safety_buffer_dist_;
    double min_speed_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}