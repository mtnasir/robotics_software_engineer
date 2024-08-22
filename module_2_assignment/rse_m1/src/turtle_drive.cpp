#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
using namespace std;

class control_turtle : public rclcpp::Node
{
public:
    control_turtle() : Node("n1")
    {
        this->declare_parameter<std::string>("t_cmd_vel", "/turtle1/cmd_vel");
        std::string t_cmd_vel = this->get_parameter("t_cmd_vel").as_string();
        this->declare_parameter<std::string>("t_cmd_fun", "a");
        this->t_cmd_fun = this->get_parameter("t_cmd_fun").as_string();
        _count = 0;
        t1 = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&control_turtle::_calback, this));
        p1 = this->create_publisher<geometry_msgs::msg::Twist>(t_cmd_vel, 10);
    }

private:
    rclcpp::TimerBase::SharedPtr t1;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr p1;
    int _count;
    std::string t_cmd_fun;
    void _calback()
    {
        _count++;
        double t = _count / 100.0; // Corrected to ensure floating-point division
        geometry_msgs::msg::Twist command;
        // float r=t*exp(t);
        RCLCPP_INFO(this->get_logger(), "Function type: %s", t_cmd_fun.c_str());
        // double dx = sin(t) + t * cos(t);
        // double dy = cos(t) - t * sin(t);
        // float v= sqrt(dx*dx+dy*dy)/100;
        if (this->t_cmd_fun == "a")
        {
            float c = 0.10;
            float c2 = 1;
            double dr = c2 * (c * t * exp(c * t) + exp(c * t));
            command.linear.x = dr;
            command.angular.z = t;
        }
        else if (this->t_cmd_fun == "b")
        {
            command.linear.x = 5;
            command.angular.z = 4;
        }
        else
        {
            command.linear.x = 1;
            command.angular.z = 4.5*cos(3* t);
        }
        p1->publish(command);
        RCLCPP_INFO(this->get_logger(), "sending command: %f and %f", command.linear.x, command.angular.z);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<control_turtle>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
