#include "turtlebot_controller/simple_controller.h"
#include <std_msgs/Float64.h>
#include <Eigen/Geometry>


SimpleController::SimpleController(const ros::NodeHandle &nh, double radius, double separation) : nh_(nh)

{
    
    ROS_INFO_STREAM("Using Wheel radius " << radius);
    ROS_INFO_STREAM("Using Wheel Separation " << separation);

    right_cmd_pub_ = nh_.advertise<std_msgs::Float64>("Wheel_right_controller/command", 10);
    left_cmd_pub_ = nh_.advertise<std_msgs::Float64>("Wheel_left_controller/command", 10);


    vel_sub_ = nh_.subscribe("turtlebot_controller/cmd_vel", 1000 , &SimpleController::velCallback, this);

    speed_conversion_ << radius/2, radius/2, radius/separation, -radius/separation;

    ROS_INFO_STREAM("The conversion matrix is \n" << speed_conversion_);


}

void SimpleController::velCallback(const geometry_msgs::Twist &msg)

{

    Eigen::Vector2d robot_speed(msg.linear.x , msg.angular.z);
    Eigen::Vector2d wheel_speed = speed_conversion_.inverse() * robot_speed;
    std_msgs::Float64 right_speed;
    std_msgs::Float64 left_speed;
    right_speed.data = wheel_speed.coeff(0);
    left_speed.data = wheel_speed.coeff(1);

    right_cmd_pub_.publish(right_speed);
    left_cmd_pub_.publish(left_speed);  

}