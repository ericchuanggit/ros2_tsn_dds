#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class VehicleStopper : public rclcpp::Node
{
public:
  explicit VehicleStopper() : Node("stop_controller")
  {
    const std::string odomTopicName = "/odom";
    const std::string cmdTopic = "/cmd_vel";
    const size_t historyDepth = 5;

    // subscribe to the odometry data from the Gazebo vehicle model
    sub_ = create_subscription<nav_msgs::msg::Odometry>(odomTopicName, historyDepth, 
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        vehiclePose_ = *msg;
        checkAndStop();
      });

    // publish stop command
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmdTopic,
      rclcpp::QoS(rclcpp::KeepLast(historyDepth)));
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  nav_msgs::msg::Odometry vehiclePose_;

  void checkAndStop()
  {
    // check conditions to stop and navigate back to start
    // in this example, we stop the vehicle when it has moved more than 10 units
    if (vehiclePose_.pose.pose.position.x >= -15) {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.linear.z = 0.0;
      cmd.angular.x = 0.0;
      cmd.angular.y = 0.0;
      cmd.angular.z = 0.0;
      pub_->publish(cmd);
      
      // here you can also publish a command to navigate back to the start
      // but this is more complex and usually involves path planning and control
    }
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VehicleStopper>();
  rclcpp::spin(node);
  return rclcpp::shutdown() ? 0 : 1;
}
