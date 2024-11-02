#include <angles/angles.h>
#include <cmath>
#include <task2_lqr.hpp>
#include <rclcpp/logging.hpp>
#include <tuple>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

input input_old = input(0, 0);

LqrNode::LqrNode()
    : Node("LqrNode"), dt_(0.03), tolerance(0.8), end_controller(false),
      max_linear_velocity(0.8), max_angular_velocity(M_PI / 2),
      current_waypoint(0), odom_received_(false)
{
  publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("landmark_topic", 10);
  robot_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10,
                                                                       std::bind(&LqrNode::robotPoseCallback, this, std::placeholders::_1));
  control_input_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  control_loop_timer_ = this->create_wall_timer(std::chrono::milliseconds(30),
                                                std::bind(&LqrNode::controlLoopCallback, this));

  Q_ << 0.8, 0, 0, 0, 0.8, 0, 0, 0, 0.8;
  R_ << 0.8, 0, 0, 0.8;
  lqr_ = std::make_unique<LQR>(Q_, R_, 100);

  waypoints_ = {State(1, 1, M_PI / 4), State(5, 3, M_PI / 2),
                State(3, 3, M_PI), State(4, 4, 3 * M_PI / 2),
                State(-1, 4, M_PI), State(-2, 3, -M_PI / 2),
                State(-3, 2, M_PI), State(-3, 1, M_PI / 2),
                State(0, 0, 0)};
  actual_state_ = State(0, 0, 0);
  pathoptimization(waypoints_);
  optimiseHeading(waypoints_);
}

void LqrNode::robotPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{

  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  actual_state_ =
      State(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
  odom_received_ = true;
}

void LqrNode::publishVelocity(double v, double w)
{

  geometry_msgs::msg::Twist msg;
  msg.linear.x = v;
  msg.angular.z = w;
  RCLCPP_INFO(rclcpp::get_logger("LQR"), "Publishing control input: v=%f, w=%f",
              v, w);
  control_input_ = input(v, w);
  input_old = input(v, w);
  control_input_pub_->publish(msg);
}

void LqrNode::optimiseHeading(std::vector<State> &waypoints)
{
  RCLCPP_INFO(rclcpp::get_logger("waypoints optimizer"), "value is ");

  for (size_t i = 0; i < waypoints.size(); ++i)
  {
    double dx = waypoints[i + 1].x - waypoints[i].x;
    double dy = waypoints[i + 1].y - waypoints[i].y;
    waypoints[i].theta = std::atan2(dy, dx);
  }
  waypoints.back().theta = waypoints[waypoints.size() - 2].theta;
  // // /////////
  // std::vector<bool> visited(waypoints.size(), false);
  // std::vector<State> waypointsnew;
  // waypointsnew.push_back(waypoints[0]);
  // visited[0] = true;
  // for (size_t k = 0; k < waypoints.size(); k++)
  // {

  //   RCLCPP_INFO(rclcpp::get_logger("waypoints optimizer"), "value %d ", k);

  //   double min_distance = 1000000000;
  //   size_t nearest_index = 0;
  //   for (size_t i = 0; i < waypoints.size(); i++)
  //   {
  //     if (!visited[i])
  //     {
  //       double px = std::pow((waypoints[i].x - waypointsnew.back().x), 2);
  //       double py = std::pow((waypoints[i].y - waypointsnew.back().y), 2);
  //       double distance = px + py;
  //       if (distance < min_distance)
  //       {
  //         min_distance = distance;
  //         nearest_index = i;
  //       }
  //     }
  //   }
  //   RCLCPP_INFO(rclcpp::get_logger("waypoints optimizer"), "value %zu is x= %f, y=%f", k, waypointsnew[k].x, waypointsnew[k].y);
  //   visited[nearest_index] = true;
  //   waypointsnew.push_back(waypoints[nearest_index]);
  // }
  // waypoints = waypointsnew;
  // //   RCLCPP_INFO(rclcpp::get_logger("waypoints optimizer"), "value is 1 ,%d", waypoints[1].x);

  // // RCLCPP_INFO(rclcpp::get_logger("waypoints optimizer"), "value is 2 ,%d", waypoints[2].x);
  // // RCLCPP_INFO(rclcpp::get_logger("waypoints optimizer"), "value is 3,%d", waypoints[3].x);

  // std::cout << waypoints[1].x << std::endl;
}

void LqrNode::pathoptimization(std::vector<State> &waypoints){
//
  std::vector<bool> visited(waypoints.size(), false);
  std::vector<State> waypointsnew;
  waypointsnew.push_back(waypoints[0]);
  visited[0] = true;
  for (size_t k = 0; k < waypoints.size(); k++)
  {

    RCLCPP_INFO(rclcpp::get_logger("waypoints optimizer"), "value %d ", k);

    double min_distance = 1000000000;
    size_t nearest_index = 0;
    for (size_t i = 0; i < waypoints.size(); i++)
    {
      if (!visited[i])
      {
        double px = std::pow((waypoints[i].x - waypointsnew.back().x), 2);
        double py = std::pow((waypoints[i].y - waypointsnew.back().y), 2);
        double distance = px + py;
        if (distance < min_distance)
        {
          min_distance = distance;
          nearest_index = i;
        }
      }
    }
    RCLCPP_INFO(rclcpp::get_logger("waypoints optimizer"), "value %zu is x= %f, y=%f", k, waypointsnew[k].x, waypointsnew[k].y);
    visited[nearest_index] = true;
    waypointsnew.push_back(waypoints[nearest_index]);
  }
  waypoints = waypointsnew;
  //   RCLCPP_INFO(rclcpp::get_logger("waypoints optimizer"), "value is 1 ,%d", waypoints[1].x);

  // RCLCPP_INFO(rclcpp::get_logger("waypoints optimizer"), "value is 2 ,%d", waypoints[2].x);
  // RCLCPP_INFO(rclcpp::get_logger("waypoints optimizer"), "value is 3,%d", waypoints[3].x);

  std::cout << waypoints[1].x << std::endl;

}

void LqrNode::controlLoopCallback()
{

  if (!odom_received_)
  {
    RCLCPP_INFO(rclcpp::get_logger("LQR"), "Waiting for odometry message...");
    return;
  }
  if (end_controller)
  {
    RCLCPP_INFO(rclcpp::get_logger("LQR"), "Goal reached!");
    control_loop_timer_->cancel();
    return;
  }

  State desired_state = waypoints_[current_waypoint];
  Eigen::Vector3d x_actual(actual_state_.x, actual_state_.y,
                           actual_state_.theta);
  Eigen::Vector3d x_desired(desired_state.x, desired_state.y,
                            desired_state.theta);
  state_error_ = x_actual - x_desired;

  if (current_waypoint == 2)
  {
    waypoints_[current_waypoint + 1] = State(-1, 3, M_PI);
  }
publish_landmark(waypoints_[current_waypoint].x, waypoints_[current_waypoint].y);
  RCLCPP_INFO(rclcpp::get_logger("LQR"), "Current Waypoint:=%d ", current_waypoint);
  RCLCPP_INFO(rclcpp::get_logger("LQR"), "Actual state: x=%f, y=%f, theta=%f", x_actual(0), x_actual(1), x_actual(2));
  RCLCPP_INFO(rclcpp::get_logger("LQR"), "Desired state: x=%f, y=%f, theta=%f", x_desired(0), x_desired(1), x_desired(2));
  RCLCPP_INFO(rclcpp::get_logger("LQR"), "State error: x=%f, y=%f, theta=%f", state_error_(0), state_error_(1), state_error_(2));
  RCLCPP_INFO(rclcpp::get_logger("LQR"), "Current goal: x=%f, y=%f, theta=%f",
              waypoints_[current_waypoint].x, waypoints_[current_waypoint].y, waypoints_[current_waypoint].theta);

  auto A = lqr_->getA(actual_state_.theta, control_input_.v, dt_);
  auto B = lqr_->getB(actual_state_.theta, dt_);
  lqr_->updateMatrices(A, B);
  lqr_->computeRiccati(B, A);

  auto u = lqr_->computeOptimalInput(state_error_);
// //  //////////////////////// new controller ////////////////////////////////////////
//   state_error_2 =  x_desired-x_actual;

//     Eigen::Matrix<float, 3, 3> A1;
//     A1.setConstant(0);
//     A1(0, 0) = std::cos(actual_state_.theta);
//     A1(0, 1) = std::sin(actual_state_.theta);
//     A1(1, 0) = -1*std::sin(actual_state_.theta);
//     A1(1, 1) = std::cos(actual_state_.theta);
//     A1(2, 2) = 1;

//    Eigen::Matrix<float, 3, 1> B1;
//     B1.setConstant(0);
//     B1(0, 0) = state_error_2[0];
//     B1(1, 0) = state_error_2[1];
//     B1(2, 0) = std::atan2(state_error_2[1],state_error_2[0]);
  
//   Eigen::Matrix<float, 3, 1> EE=A1*B1;
//   const double vd=0.1;
//   const double wd=0;
//   const double k11=1;
//   const double k12=5;
//   const double k13=1;
//   double v1= vd*std::cos(EE(2,0))+k11*EE(0,0);
//   double w1= wd+k12*vd*EE(1,0)+k13*vd*std::sin(EE(2,0))/EE(2,0);
//   u(0)=v1;
//   u(1)=w1;
// // ////////// end new controller /////////////////////////

  Eigen::EigenSolver<Eigen::MatrixXd> solver(B * lqr_->K_ + A);
  auto eigenValues = solver.eigenvalues().real();
  RCLCPP_INFO(rclcpp::get_logger("LQR"), "Eigenvalues: %f, %f, %f",
              eigenValues(0), eigenValues(1), eigenValues(2));

  publishVelocity(
      std::clamp(u(0), -max_linear_velocity, max_linear_velocity),
      std::clamp(u(1), -max_angular_velocity, max_angular_velocity));
  double ssErorr=state_error_[0]*state_error_[0]+state_error_[1]*state_error_[1];
  if (ssErorr < tolerance)
  {
    RCLCPP_INFO(rclcpp::get_logger("LQR"), "Waypoint reached!");
    current_waypoint++;
    publish_landmark(waypoints_[current_waypoint].x, waypoints_[current_waypoint].y);
    if (current_waypoint >= waypoints_.size())
    {
      end_controller = true;
      publishVelocity(0.0, 0.0);
    }
  }
}

void LqrNode::publish_landmark(double xv, double yv)
{
  auto marker = visualization_msgs::msg::Marker();
  marker.header.frame_id = "odom";
  marker.header.stamp = this->get_clock()->now();
  marker.ns = "landmarks";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = xv;
  marker.pose.position.y = yv;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
 // Create and configure the second marker (line list)
  // auto marker2 = visualization_msgs::msg::Marker();
  // marker2.header.frame_id = "odom";
  // marker2.header.stamp = this->get_clock()->now();
  // marker2.ns = "landmarks";
  // marker2.id = 2;
  // marker2.type = visualization_msgs::msg::Marker::LINE_STRIP;
  // marker2.action = visualization_msgs::msg::Marker::ADD;
  // marker2.scale.x = 0.2;
  // marker2.scale.y = 0.2;
  // marker2.scale.z = 0.2;
  // marker2.color.a = 1.0;
  // marker2.color.r = 1.0;
  // marker2.color.g = 0.0;
  // marker2.color.b = 0.0;  // Add points to the line list
  //   geometry_msgs::msg::Point P1;
  //   P1.x= actual_state_.x;
  //   P1.y = actual_state_.y;
  //   P1.z = 0.0;
  //   marker2.points.push_back(P1);
  

  // Publish the second marker
  publisher_->publish(marker);
  // publisher_->publish(marker2);
}
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto controller = std::make_shared<LqrNode>();
  rclcpp::spin(controller);
  rclcpp::shutdown();
  return 0;
}
