#include "nina_controller/noisy_controller.hpp"
#include <Eigen/Geometry>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <random>
#include <tf2/LinearMath/Quaternion.h>


using std::placeholders::_1;

NoisyController::NoisyController(const std::string & name)
    : Node(name)
    , left_wheel_prev_pos_(0.0)
    , right_wheel_prev_pos_(0.0)
    , x_(0.0)
    , y_(0.0)
    , theta_(0.0)
{ 
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("wheel_separation", 0.17);

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_separation_ = get_parameter("wheel_separation").as_double();

    RCLCPP_INFO_STREAM(get_logger(), "Using wheel_radius: " << wheel_radius_);
    RCLCPP_INFO_STREAM(get_logger(), "Using wheel_separation: " << wheel_separation_);

    time_prev_ = get_clock()->now();   

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10,
                std::bind(&NoisyController::jointCallback, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("nina_controller/odom_noisy", 10);
    

    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_footprint_ekf";
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0;
    odom_msg_.pose.pose.orientation.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 1.0;

    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "base_footprint_noisy";
}


void NoisyController::jointCallback(const sensor_msgs::msg::JointState & msg)
{   
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine noise_generator(seed);
    std::normal_distribution<double> left_encoder_noise(0.0, 0.005);
    std::normal_distribution<double> right_encoder_noise(0.0, 0.005);

    double wheel_encoder_left = msg.position.at(1) + left_encoder_noise(noise_generator);
    double wheel_encoder_right = msg.position.at(0) + right_encoder_noise(noise_generator);

    double del_left = wheel_encoder_left - left_wheel_prev_pos_;
    double del_right = wheel_encoder_right - right_wheel_prev_pos_;
  
    rclcpp::Time msg_time_ = msg.header.stamp;
    rclcpp::Duration dt = msg_time_ - time_prev_; 

    left_wheel_prev_pos_ = msg.position.at(1);
    right_wheel_prev_pos_ = msg.position.at(0);
    time_prev_ = msg_time_;

    double final_left = del_left / dt.seconds();
    double final_right = del_right / dt.seconds();

    double linear = (wheel_radius_ * final_left + wheel_radius_ * final_right) /2 ;
    double angular = (wheel_radius_ * final_right - wheel_radius_ * final_left) / wheel_separation_ ;

    double position = (wheel_radius_ * del_right + wheel_radius_ * del_left) /2;
    double orientation = (wheel_radius_ * del_right - wheel_radius_ * del_left) / wheel_separation_ ;

    theta_ += orientation;
    x_ += position * cos(theta_);
    y_ += position * sin(theta_);

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_msg_.pose.pose.orientation.x = q.x();
    odom_msg_.pose.pose.orientation.y = q.y();
    odom_msg_.pose.pose.orientation.z = q.z();
    odom_msg_.pose.pose.orientation.w = q.w();
    odom_msg_.header.stamp = get_clock()->now();
    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;
    odom_msg_.twist.twist.linear.x = linear;
    odom_msg_.twist.twist.angular.z = angular;

    odom_pub_->publish(odom_msg_);

     // TF
    transform_stamped_.transform.translation.x = x_;
    transform_stamped_.transform.translation.y = y_;
    transform_stamped_.transform.rotation.x = q.getX();
    transform_stamped_.transform.rotation.y = q.getY();
    transform_stamped_.transform.rotation.z = q.getZ();
    transform_stamped_.transform.rotation.w = q.getW();
    transform_stamped_.header.stamp = get_clock()->now();
    transform_broadcaster_->sendTransform(transform_stamped_);
}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NoisyController>("noisy_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
