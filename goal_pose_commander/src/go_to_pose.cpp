#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include <iostream>
#include <utility>
#include <chrono>
#include <memory>
#include <cmath>

using namespace std;

class GoToPose: public rclcpp::Node{
    private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    double goal_x, goal_y, goal_yaw;
    double curr_x, curr_y, curr_yaw;
    public:
    GoToPose():Node("go_to_pose_node"){
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 1, bind(&GoToPose::get_goal_pose, this, placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, bind(&GoToPose::get_bot_position, this, placeholders::_1));
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        timer_ = this->create_wall_timer(chrono::milliseconds(20), bind(&GoToPose::controller_loop, this));
    }
    ~GoToPose(){}

    void get_goal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        this->goal_x = msg->pose.position.x;
        this->goal_y = msg->pose.position.y;
        tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, this->goal_yaw);
        RCLCPP_INFO(this->get_logger(),"[x: %lf, y: %lf, z: %lf]", this->goal_x, this->goal_y, this->goal_yaw);
        

    }

    void get_bot_position(const nav_msgs::msg::Odometry::SharedPtr msg){
        this->curr_x = msg->pose.pose.position.x;
        this->curr_y = msg->pose.pose.position.y;
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, this->curr_yaw);
        // RCLCPP_INFO(this->get_logger(),"[x: %lf, y: %lf, z: %lf]", this->curr_x, this->curr_y, this->curr_yaw);
    }

    void controller_loop(){
        geometry_msgs::msg::Twist msg;

        double err_x = 5*(this->goal_x-this->curr_x);
        double err_y = 5*(this->goal_y-this->curr_y);
        double err_yaw = 5*(this->goal_yaw-this->curr_yaw);

        // RCLCPP_INFO(this->get_logger(), "err_x: %lf, err_y: %lf", err_x, err_y);

        msg.linear.x = cos(this->curr_yaw)*err_x+sin(this->curr_yaw)*err_y;
        msg.angular.z = err_yaw;

        if(abs(err_x) < 0.01 && abs(err_y) < 0.01 && abs(err_yaw) < 0.01){
            msg.linear.x = 0.0;
            msg.linear.y = 0.0;
            msg.angular.z = 0.0;
            cmd_vel_pub_->publish(msg);
        }

        cmd_vel_pub_->publish(msg);
    }



};

int main(int argc, char**  argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoToPose>());
    rclcpp::shutdown();
    return 0;
}