#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2/LinearMath/Quaternion.h>



using namespace std;
class mynode: public rclcpp::Node 
{
    
    public:
    mynode(): Node("tf11"),bcaster_(this)
    {
      t1=this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&mynode::timercallback,this));

    }
    private:
    rclcpp::TimerBase::SharedPtr t1;
    tf2_ros::StaticTransformBroadcaster bcaster_;
    void timercallback()
    {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp=this->get_clock()->now();
        t.header.frame_id="world";
        t.child_frame_id="my_frame";

        t.transform.translation.x= 1.0;
        t.transform.translation.y= 1.0;
        t.transform.translation.z= 0.0;

        tf2::Quaternion q;
        q.setRPY(0,0,0);
        t.transform.rotation.x= q.x();
        t.transform.rotation.y= q.y();
        t.transform.rotation.z= q.z();
        t.transform.rotation.w= q.w();

        bcaster_.sendTransform(t);
    }


};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    
    auto node=std::make_shared<mynode>();
    rclcpp::spin(node);
    
    rclcpp::shutdown();



}