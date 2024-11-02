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
    float findMedian(std::vector<float> &vec)
    {
        // Sort the vector
        std::sort(vec.begin(), vec.end());

        // Since the number of elements is odd, return the middle element
        return vec[vec.size() / 2];
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr camImage)
    {

        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(camImage, "bgr8");
        cv::Mat gray_image, canny_image;
        cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_RGB2GRAY);
        int upper_threshold = this->get_parameter("upper_threshold").as_int();
        int lower_threshold = this->get_parameter("lower_threshold").as_int();
        float kp = this->get_parameter("kp").get_value<float>();
        int Filter = this->get_parameter("Filter").as_int();
        cv::Canny(gray_image, canny_image, lower_threshold, upper_threshold);
        //
        cv::Mat img = canny_image(cv::Range(850, 1079), cv::Range(1, 1919));

        int width = img.cols;
        int height = img.rows;

        // Proper logging of width and height
        RCLCPP_INFO(this->get_logger(), "Image Width: %d, Image Height: %d", width, height);
        std::vector<int> edge;
        for (int i = 0; i < 1920; ++i)
        {
            if (img.at<uchar>(160, i) == 255)
            {
                edge.push_back(i);
            }
        }
        float sum = std::accumulate(edge.begin(), edge.end(), 0);
        // Calculate the average
        float center = static_cast<float>(sum) / edge.size();
        // mid_.push_back(center);
        // if (mid_.size() >= Filter)
        // {
        //     mid_.erase(mid_.begin()); // Remove the first element
        // }
        // mid_.push_back(center); // Add new element at the end

        // // float sum_ = std::accumulate(mid_.begin(), mid_.end(), 0);
        // // sum_ = static_cast<float>(sum_) / Filter;
        // float sum2 = findMedian(mid_);
        // center=sum2;
        // float
        RCLCPP_INFO(this->get_logger(), "Edge at center at column : %f, the image center is = %f", center, static_cast<float>(width) / 2);
        _errorIntegration = (static_cast<float>(width) / 2 - center) + error_;
        _errorDerivative = (static_cast<float>(width) / 2 - center) - error_;
        error_ = static_cast<float>(width) / 2 - center;

        _command.linear.x = 0.1;
        _command.angular.z = kp * error_+0.0001*_errorIntegration;

        // if (error_ > 0)
        // {
        //     RCLCPP_INFO(this->get_logger(), "Turn left");
        //     _command.angular.z = 0.3;
        // }
        // else
        // {
        //     RCLCPP_INFO(this->get_logger(), "Turn right");
        //     _command.angular.z = -0.3;
        // }
        publisher_->publish(_command);

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