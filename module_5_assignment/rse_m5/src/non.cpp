#include <iostream>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <angles/angles.h>

#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Define the State class to represent position and orientation
class State {
public:
    double x;
    double y;
    double theta;

    State(double x_=0.0, double y_=0.0, double theta_=0.0) : x(x_), y(y_), theta(theta_) {}
};

// Define the input struct to represent control inputs
struct input {
    double v;
    double w;
    input(double v_=0.0, double w_=0.0) : v(v_), w(w_) {}
};

class mynode : public rclcpp::Node
{
public:
    mynode() : Node("cpp10"), dt_(0.03), tolerance(0.8), end_controller(false),
          max_linear_velocity(0.8), max_angular_velocity(M_PI / 2),
          current_waypoint(0), odom_received_(false),
          actual_state_(0.0, 0.0, 0.0), input_old(0.0, 0.0)
    {
        RCLCPP_INFO(this->get_logger(), "Starting the node");

        // Publishers and subscribers
        pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("landmark_topic", 10);
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&mynode::robotPoseCallback, this, std::placeholders::_1));

        // Timer
        timer_control_loop_ = this->create_wall_timer(
            std::chrono::milliseconds(30), std::bind(&mynode::controlLoopCallback, this));

        // Initialization
        waypoints_ = {
            State(1, 1, M_PI / 4), State(5, 3, M_PI / 2),
            State(3, 3, M_PI), State(4, 4, 3 * M_PI / 2),
            State(-1, 4, M_PI), State(-2, 3, -M_PI / 2),
            State(-3, 2, M_PI), State(-3, 1, M_PI / 2),
            State(0, 0, 0)
        };
        pathoptimization(waypoints_);
        optimiseHeading(waypoints_);
    }

private:
    void robotPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        actual_state_ = State(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
        odom_received_ = true;
    }

    void controlLoopCallback()
    {
        if (!odom_received_)
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for odometry message...");
            return;
        }
        if (end_controller)
        {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            timer_control_loop_->cancel();
            return;
        }

        State desired_state = waypoints_[current_waypoint];
        Eigen::Vector3d x_actual(actual_state_.x, actual_state_.y, actual_state_.theta);
        Eigen::Vector3d x_desired(desired_state.x, desired_state.y, desired_state.theta);
        state_error_ = x_actual - x_desired;

        publish_landmark(desired_state.x, desired_state.y);
        RCLCPP_INFO(this->get_logger(), "Current Waypoint:=%zu ", current_waypoint);
        RCLCPP_INFO(this->get_logger(), "Actual state: x=%f, y=%f, theta=%f",
                    x_actual(0), x_actual(1), x_actual(2));
        RCLCPP_INFO(this->get_logger(), "Desired state: x=%f, y=%f, theta=%f",
                    x_desired(0), x_desired(1), x_desired(2));
        RCLCPP_INFO(this->get_logger(), "State error: x=%f, y=%f, theta=%f",
                    state_error_(0), state_error_(1), state_error_(2));

        // Implement your custom controller here
        state_error_2 = x_desired - x_actual;

        Eigen::Matrix3f A1 = Eigen::Matrix3f::Zero();
        A1(0, 0) = std::cos(actual_state_.theta);
        A1(0, 1) = std::sin(actual_state_.theta);
        A1(1, 0) = -std::sin(actual_state_.theta);
        A1(1, 1) = std::cos(actual_state_.theta);
        A1(2, 2) = 1;

        Eigen::Vector3f B1;
        B1 << state_error_2[0], state_error_2[1],
              angles::shortest_angular_distance(actual_state_.theta, desired_state.theta);

        Eigen::Vector3f EE = A1 * B1;
        const double vd = 0.1;
        const double wd = 0.0;
        const double k11 = 1.0;
        const double k12 = 5.0;
        const double k13 = 1.0;

        double v1 = vd * std::cos(EE(2)) + k11 * EE(0);
        double w1 = wd + k12 * vd * EE(1) + k13 * vd * std::sin(EE(2));

        double v_command = std::clamp(v1, -max_linear_velocity, max_linear_velocity);
        double w_command = std::clamp(w1, -max_angular_velocity, max_angular_velocity);

        publishVelocity(v_command, w_command);

        double ssError = state_error_.head<2>().squaredNorm();
        if (ssError < tolerance)
        {
            RCLCPP_INFO(this->get_logger(), "Waypoint reached!");
            current_waypoint++;
            if (current_waypoint >= waypoints_.size())
            {
                end_controller = true;
                publishVelocity(0.0, 0.0);
            }
        }
    }

    void publishVelocity(double v, double w)
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = v;
        msg.angular.z = w;
        RCLCPP_INFO(this->get_logger(), "Publishing control input: v=%f, w=%f", v, w);
        control_input_ = input(v, w);
        input_old = input(v, w);
        pub_cmd_vel_->publish(msg);
    }

    void publish_landmark(double xv, double yv)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "landmarks";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = xv;
        marker.pose.position.y = yv;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        pub_marker_->publish(marker);
    }

    void optimiseHeading(std::vector<State> &waypoints)
    {
        RCLCPP_INFO(this->get_logger(), "Optimizing headings of waypoints.");
        for (size_t i = 0; i < waypoints.size() - 1; ++i)
        {
            double dx = waypoints[i + 1].x - waypoints[i].x;
            double dy = waypoints[i + 1].y - waypoints[i].y;
            waypoints[i].theta = std::atan2(dy, dx);
        }
        waypoints.back().theta = waypoints[waypoints.size() - 2].theta;
    }

    void pathoptimization(std::vector<State> &waypoints)
    {
        // Placeholder for path optimization code
        // The original code was commented out, so we'll leave it empty for now
    }

    // Member variables
    input input_old;
    input control_input_;

    double dt_;
    double tolerance;
    bool end_controller;
    double max_linear_velocity;
    double max_angular_velocity;
    size_t current_waypoint;
    bool odom_received_;
    State actual_state_;

    std::vector<State> waypoints_;

    Eigen::Vector3d state_error_;
    Eigen::Vector3d state_error_2;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::TimerBase::SharedPtr timer_control_loop_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mynode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}