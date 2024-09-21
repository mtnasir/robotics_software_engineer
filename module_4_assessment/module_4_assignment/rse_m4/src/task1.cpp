#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <algorithm>
#include <vector>
#include <numeric> // For std::accumulate
// #include "math.hpp"

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
        s1 = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10,
                                                                std::bind(&MyNode::imageCallback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        this->declare_parameter<int>("lower_threshold", 150);
        this->declare_parameter<int>("upper_threshold", 250);
        this->declare_parameter<int>("Filter", 5);
        this->declare_parameter<float>("kp", 0.003);
    }

private:
    void controlCommand(float center,cv::Mat img)
{
        float kp = this->get_parameter("kp").get_value<float>();
        int width = img.cols;
        int height = img.rows;
        _errorIntegration = (static_cast<float>(width) / 2 - center) + error_;
        _errorDerivative = (static_cast<float>(width) / 2 - center) - error_;
        error_ = static_cast<float>(width) / 2 - center;
        _command.linear.x = 0.1;
        _command.angular.z = kp * error_ + 0.0001 * _errorIntegration;
        publisher_->publish(_command);

}
    cv::Mat getImageCrop(const sensor_msgs::msg::Image::SharedPtr camImage)
    {
        int upper_threshold = this->get_parameter("upper_threshold").as_int();
        int lower_threshold = this->get_parameter("lower_threshold").as_int();
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(camImage, "bgr8");
        cv::Mat gray_image, canny_image;
        cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_RGB2GRAY);
        cv::Canny(gray_image, canny_image, lower_threshold, upper_threshold);
        cv::Mat img = canny_image(cv::Range(850, 1079), cv::Range(1, 1919));
        return img;
    }
    float getCenter(cv::Mat img)
    {
        std::vector<int> edge;
        for (int i = 0; i < 1920; ++i)
        {
            if (img.at<uchar>(160, i) == 255)
            {
                edge.push_back(i);
            }
        }
        float sum = std::accumulate(edge.begin(), edge.end(), 0);
        float center = static_cast<float>(sum) / edge.size();
        return center;
    }
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr camImage)
    {
        cv::Mat img= getImageCrop(camImage);
        float center = getCenter(img);
        controlCommand( center, img);
        cv::circle(img, cv::Point(center, 160), 5, cv::Scalar(255, 255, 255), -1);
        cv::imshow("Image", img);
        cv::waitKey(1);
    }
    float _errorIntegration;
    float _errorDerivative;
    float error_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr s1;
    float _min_front;
    RobotState _state;
    geometry_msgs::msg::Twist _command;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    std::vector<float> mid_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}