#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals; // for "ms"

// The robot controller
class VehicleController : public rclcpp::Node
{
public:
  explicit VehicleController() : Node("vehicle_control")
  {
    const std::string odomTopicName = "/odom";
    const std::string cmdTopic = "/cmd_vel";
    const size_t historyDepth = 5;
    const double poseResetPositionX = -10000;

    // subscribe to the odometry data from the Gazebo vehicle model
    sub_ = create_subscription<nav_msgs::msg::Odometry>(odomTopicName,
      historyDepth, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        vehiclePose_ = *msg;
      });

    // publish vehicle command periodically based on odometry data
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmdTopic,
      rclcpp::QoS(rclcpp::KeepLast(historyDepth)));
    timer_ = this->create_wall_timer(10ms, [this] {
      VehicleControlPub();
    });

    // reset the vehicle pose until we subscribe to the odometry topic
    vehiclePose_.pose.pose.position.x = poseResetPositionX;
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  nav_msgs::msg::Odometry vehiclePose_;

  // predefined sequence of vehicle commands to pass the moose test
  void VehicleControlPub()
  {
    struct TwistCommand {
      std::string thresholdPoint;
      float  linearX;    // linear x component of the twist command
      float  angularZ;   // angular z componennt of the twist command
      double xThreshold; // x coordinate to switch to the next control command 
    };
    // fine-tuned steering commands to pass the moose test in the simulated world
    const std::vector<struct TwistCommand> steeringCommands = {
      {"FirstPart",    5.0,  0.5, -4.0},
      {"SecondPart",   5.0,  0.5, -3.0},
      {"ThirdPart",    5.0,  0.5, -2.0},
      {"FourthPart",   5.0,  0.5, -1.9},
      {"FifthPart",    5.0,  0.5, -1.8},
      {"SixthPart",    5.0,  0.5, -1.7},
      {"SeventhPart",  5.0,  0.5, -1.6},
      {"eighth",       5.0,  0.5, -1.5},
      {"ninth",        5.0,  0.5, -1.4},
      {"tenth",        5.0,  0.5, -1.3},
      {"eleventh",     5.0,  0.5, -1.2},
      {"SeventhPart",  5.0,  0.5, -1.1},
      {"eighth",       5.0,  0.5, -1.0},
      {"ninth",        5.0,  0.5, -0.9},
      {"tenth",        5.0,  0.5, -0.8},
      {"eleventh",     5.0,  0.5, -0.7},
      {"ninth",        5.0,  0.5, -0.6},
      {"tenth",        5.0,  0.5, -0.5},
      {"eleventh",     5.0,  0.5, -0.4},
      {"ninth",        5.0,  0.5, -0.3},
      {"tenth",        5.0,  0.5, -0.2},
      {"eleventh",     5.0,  0.5, -0.1},
      {"ninth",        5.0,  0.5,  0.0},
      {"tenth",        5.0,  0.5,  0.1},
      {"eleventh",     5.0,  0.5,  0.2},
      {"ninth",        5.0,  0.5,  0.3},
      {"tenth",        5.0,  0.5,  0.4},
      {"eleventh",     5.0,  0.5,  0.5},
      {"ninth",        5.0,  0.5,  0.6},
      {"tenth",        5.0,  0.5,  0.7},
      {"eleventh",     5.0,  0.5,  0.8},
      // {"eleventh",     5.0,  0.5,  0.9},
      // {"eleventh",     5.0,  0.5,  1.0},
      // {"eleventh",     5.0,  0.5,  1.1},
      // {"eleventh",     5.0,  0.5,  1.2},
      // {"eleventh",     5.0,  0.5,  1.3},
      // {"eleventh",     5.0,  0.5,  1.4},
      // {"eleventh",     5.0,  0.5,  1.5},
      // {"eleventh",     5.0,  0.5,  1.6},
      // {"eleventh",     5.0,  0.5,  1.7},
      // {"eleventh",     5.0,  0.5,  1.8},
      // {"eleventh",     5.0,  0.5,  1.9},
    };

    static unsigned int stateId = 0;

    if (stateId >= steeringCommands.size()) {
      return; // finished the sequence, stop publishing
    }

    // current state twist command 
    auto &state = steeringCommands[stateId];
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = state.linearX;
    cmd.linear.y  = .0;
    cmd.linear.z  = .0;
    cmd.angular.x = .0;
    cmd.angular.y = .0;
    cmd.angular.z = state.angularZ;

    // once the x coordinate threshold is passed, switch to the next state
    auto xCoordinate = vehiclePose_.pose.pose.position.x;
    if (xCoordinate >= state.xThreshold) {
      std::ostringstream s;
      s << "reached x=" << xCoordinate << ", "
        << steeringCommands[stateId].thresholdPoint.c_str()
        << " steering command: linear.x=" << cmd.linear.x << " angular.z=" << cmd.angular.z;
      RCLCPP_INFO(get_logger(), s.str().c_str());
      stateId++;
    }

    pub_->publish(cmd);
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VehicleController>();
  rclcpp::spin(node);
  return rclcpp::shutdown() ? 0 : 1;
}