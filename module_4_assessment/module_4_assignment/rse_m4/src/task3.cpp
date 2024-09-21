#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <algorithm>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

enum class RobotState
{
    MOVING_STRAIGHT,
    TURNING_LEFT,
    TURNING_RIGHT,
    OUT_OF_MAZE
};

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("maze_solving")
    {
        s3 = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 10,
                                                                std::bind(&MyNode::imuCallback, this, std::placeholders::_1));
        s2 = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10,
                                                                std::bind(&MyNode::imageCallback, this, std::placeholders::_1));

        s1 = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10,
                                                                    std::bind(&MyNode::lidarCallback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr Imu_msg)
    {
        RCLCPP_INFO(this->get_logger(), "Accelration readings, x: %f, y: %f, z: %f",
                    Imu_msg->linear_acceleration.x,
                    Imu_msg->linear_acceleration.y,
                    Imu_msg->linear_acceleration.z);
    }
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg)
    {
        getLidarValues(lidar_msg);
        getOrentation();
        sendCommand();
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr s1;
    float _min_front;
    RobotState _state;
    geometry_msgs::msg::Twist _command;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    float _angleError;
    float _errorIntegration;
    float _errorDerivative;
    float _min_left;
    float _min_right;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr s2;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr s3;

    void getLidarValues(sensor_msgs::msg::LaserScan::SharedPtr lidar_msg)
    {
        auto _min_front1 = std::min_element(lidar_msg->ranges.begin() + 340, lidar_msg->ranges.end());
        auto _min_front2 = std::min_element(lidar_msg->ranges.begin(), lidar_msg->ranges.begin() + 20);
        _min_front = std::min((float)*_min_front1, (float)*_min_front2);
        _min_left = *std::min_element(lidar_msg->ranges.begin() + 60, lidar_msg->ranges.begin() + 100);
        _min_right = *std::min_element(lidar_msg->ranges.begin() + 260, lidar_msg->ranges.begin() + 300);
        _command.linear.x = 0;
        _command.linear.y = 0;
        _command.linear.z = 0;
        _command.angular.x = 0;
        _command.angular.y = 0;
        _command.angular.z = 0;

        RCLCPP_INFO(this->get_logger(), "lidar Data='%f' /'%f','%f'",
                    _min_right,
                    _min_front,
                    _min_left);
    }
    void getOrentation()
    {
        switch (_state)
        {
        case RobotState::MOVING_STRAIGHT:
            if (_min_front < 0.4)
            {
                if (_min_left < _min_right)
                {
                    _state = RobotState::TURNING_RIGHT;
                }
                else
                {
                    _state = RobotState::TURNING_LEFT;
                }
            }
            break;
        case RobotState::TURNING_LEFT:
            if (_min_front > 0.4)
            {
                _state = RobotState::MOVING_STRAIGHT;
            }

            break;
        case RobotState::TURNING_RIGHT:
            if (_min_front > 0.4)
            {
                _state = RobotState::MOVING_STRAIGHT;
            }
            break;
        }
    }
    void sendCommand()
    {
        if (_min_front > 1 && _min_left > 1 && _min_right > 1)
        {
            _state = RobotState::OUT_OF_MAZE;
        }

        if (_min_front > 1 && _min_left < 1 & _min_right < 1)
        {
            _state = RobotState::MOVING_STRAIGHT;
        }
        switch (_state)
        {
        case RobotState::MOVING_STRAIGHT:
            _command.linear.x = 0.1;
            _errorIntegration = (_min_left - _min_right) + _angleError;
            _errorDerivative = (_min_left - _min_right) - _angleError;

            _angleError = _min_left - _min_right;

            _command.angular.z = 0.2 * _angleError + 0.01 * _errorIntegration + 2 * _errorDerivative;

            break;
        case RobotState::TURNING_LEFT:
            _command.linear.x = 0;
            _command.angular.z = 1.3;
            break;
        case RobotState::TURNING_RIGHT:
            _command.linear.x = 0;
            _command.angular.z = -1.3;
            break;
        case RobotState::OUT_OF_MAZE:
            _command.linear.x = 0;
            _command.angular.z = 0;
        }

        publisher_->publish(_command);
    }
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr camImage)
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(camImage, "bgr8");

        // Convert to grayscale if needed
        cv::Mat gray_image;
        cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_RGB2GRAY);

        // Resize the image (reduce the size by half, for example)
        cv::Mat resized_image;
        double scale_factor = 0.5; // Adjust this to make the image smaller or larger
        cv::resize(cv_ptr->image, resized_image, cv::Size(), scale_factor, scale_factor, cv::INTER_LINEAR);

        // Display the resized image
        cv::imshow("Resized Image", resized_image);
        cv::waitKey(1);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}