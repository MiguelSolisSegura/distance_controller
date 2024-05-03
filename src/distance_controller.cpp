#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <vector>
#include <chrono>

using namespace std::chrono_literals;

class DistanceController : public rclcpp::Node
{
public:
    DistanceController() : Node("distance_controller"), current_goal_index_(0)
    {
        // Initialize publisher
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Initialize subscriber
        odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/rosbot_xl_base_controller/odom", 10, std::bind(&DistanceController::odometryCallback, this, std::placeholders::_1));

        // Hardcoded distance goals in meters
        distance_goals_ = {1.0, 2.0, 3.0};

        // PID parameters
        this->declare_parameter<double>("kp", 1.5);
        this->declare_parameter<double>("ki", 0.001);
        this->declare_parameter<double>("kd", 1.0);

        // Use the declared parameters
        kp_ = this->get_parameter("kp").as_double();
        ki_ = this->get_parameter("ki").as_double();
        kd_ = this->get_parameter("kd").as_double();

        // Initialize PID control variables
        integral_ = 0.0;
        previous_error_ = 0.0;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    std::vector<double> distance_goals_;
    int current_goal_index_;
    double kp_, ki_, kd_;
    double integral_, previous_error_;

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double current_position = msg->pose.pose.position.x;  
        if (current_goal_index_ < int(distance_goals_.size()))
        {
            double goal = distance_goals_[current_goal_index_];
            double error = goal - current_position;
            double derivative = error - previous_error_;
            integral_ += error;
            previous_error_ = error;

            double control_signal = kp_ * error + ki_ * integral_ + kd_ * derivative;

            // Publish the velocity command
            auto cmd_vel = geometry_msgs::msg::Twist();
            cmd_vel.linear.x = control_signal;
            velocity_publisher_->publish(cmd_vel);

            // Check if goal is reached within a small threshold
            if (std::abs(error) < 0.001)
            {
                rclcpp::sleep_for(1s);
                RCLCPP_INFO(this->get_logger(), "New waypoint acquired");
                current_goal_index_++;
                integral_ = 0;  // Reset integral term
            }
        }
        else
        {
            // Stop the robot if all goals are reached
            auto stop_cmd = geometry_msgs::msg::Twist();
            stop_cmd.linear.x = 0;
            velocity_publisher_->publish(stop_cmd);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DistanceController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
