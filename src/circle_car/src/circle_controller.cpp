#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals; // for "ms"

class VehicleController : public rclcpp::Node
{
public:
  explicit VehicleController() : Node("circle_controller")
  {
    const std::string cmdTopic = "/cmd_vel";
    const std::string odomTopic = "/odom";
    const std::string pathTopic = "/path";
    const size_t historyDepth = 5;

    // Publish vehicle command periodically
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmdTopic,
      rclcpp::QoS(rclcpp::KeepLast(historyDepth)));
    timer_ = this->create_wall_timer(10ms, [this] {
      VehicleControlPub();
    });

    // Publish path
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(pathTopic, historyDepth);

    // Subscribe to the odometry topic to get the vehicle's current position
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odomTopic, historyDepth,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        UpdatePath(msg);
      });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  nav_msgs::msg::Path path_;
  nav_msgs::msg::Odometry vehiclePose_;


  // Generate circle trajectory
  void VehicleControlPub()
  {
    const float linearX = 5.0; // Linear speed
    const float angularZ = 0.5; // Angular speed to generate a circle
    
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linearX;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = angularZ;

    pub_->publish(cmd);
  }

  void UpdatePath(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;

    path_.poses.push_back(pose);

    // You may want to limit the size of the path to avoid consuming too much memory.
    if (path_.poses.size() > 10000)
    {
      path_.poses.erase(path_.poses.begin());
    }

    path_.header.frame_id = "odom";
    path_.header.stamp = this->get_clock()->now();

    path_pub_->publish(path_);
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VehicleController>();
  rclcpp::spin(node);
  return rclcpp::shutdown() ? 0 : 1;
}
