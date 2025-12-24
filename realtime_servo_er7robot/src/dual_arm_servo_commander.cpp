#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>

using namespace std::chrono_literals;
size_t count_ = 0;
rclcpp::Node::SharedPtr node_;
rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr camera_joint_cmd_pub_;
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr camera_twist_cmd_pub_;
rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr laser_joint_cmd_pub_;
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr laser_twist_cmd_pub_;

void publishCommands()
{
  // First we will publish 100 joint jogging commands. The :code:`joint_names` field allows you to specify individual
  // joints to move, at the velocity in the corresponding :code:`velocities` field. It is important that the message
  // contains a recent timestamp, or Servo will think the command is stale and will not move the robot.
  if (count_ < 100)
  {
    auto c_msg = std::make_unique<control_msgs::msg::JointJog>();
    c_msg->header.stamp = node_->now();
    c_msg->joint_names.push_back("camera_joint_1");
    c_msg->velocities.push_back(0.3);
    camera_joint_cmd_pub_->publish(std::move(c_msg));
    auto l_msg = std::make_unique<control_msgs::msg::JointJog>();
    l_msg->header.stamp = node_->now();
    l_msg->joint_names.push_back("laser_joint_1");
    l_msg->velocities.push_back(0.3);
    laser_joint_cmd_pub_->publish(std::move(l_msg));
    ++count_;
  }

  // After a while, we switch to publishing twist commands. The provided frame is the frame in which the twist is
  // defined, not the robot frame that will follow the command. Again, we need a recent timestamp in the message
  else if(100 <= count_ && count_ <= 200)
  {
    auto c_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    c_msg->header.stamp = node_->now();
    c_msg->header.frame_id = "base_link";
    c_msg->twist.linear.x = 0.3;
    c_msg->twist.angular.z = 0.5;
    camera_twist_cmd_pub_->publish(std::move(c_msg));
    auto l_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    l_msg->header.stamp = node_->now();
    l_msg->header.frame_id = "base_link";
    l_msg->twist.linear.x = 0.3;
    l_msg->twist.angular.z = 0.5;
    laser_twist_cmd_pub_->publish(std::move(l_msg));
    ++count_;
  }
}



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;

  // This is false for now until we fix the QoS settings in moveit to enable intra process comms
  node_options.use_intra_process_comms(false);
  node_ = std::make_shared<rclcpp::Node>("dual_servo_commander", node_options);
  camera_joint_cmd_pub_ = node_->create_publisher<control_msgs::msg::JointJog>("/camera_servo_demo/camera_er7robot_servo/delta_joint_cmds", 10);
  camera_twist_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/camera_servo_demo/camera_er7robot_servo/delta_twist_cmds", 10);
  laser_joint_cmd_pub_ = node_->create_publisher<control_msgs::msg::JointJog>("/laser_servo_demo/laser_er7robot_servo/delta_joint_cmds", 10);
  laser_twist_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/laser_servo_demo/laser_er7robot_servo/delta_twist_cmds", 10);
  rclcpp::TimerBase::SharedPtr timer = node_->create_wall_timer(50ms, publishCommands);
  rclcpp::spin(node_);
  rclcpp::shutdown();
  return 0;
}
