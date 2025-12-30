#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>
#include <iomanip>
using namespace std::chrono_literals;

// --- 全局变量 ---
size_t count_ = 0;
rclcpp::Node::SharedPtr node_;
rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr camera_joint_cmd_pub_;
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr camera_twist_cmd_pub_;
rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr laser_joint_cmd_pub_;
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr laser_twist_cmd_pub_;
std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
static Eigen::Isometry3d T_rel_init;
// --- 函数前置声明 ---
// 1. 修改参数为 const引用，避免拷贝；修改参数类型匹配 unique_ptr 的解引用
geometry_msgs::msg::TwistStamped compute_slave_command(const geometry_msgs::msg::TwistStamped& msg);

void publish_master_commands()
{
  // --- 验证逻辑：获取当前位姿 ---
  Eigen::Isometry3d T_m_curr, T_s_curr;
  bool tf_success = false;
  try {
    auto tf_m = tf_buffer_->lookupTransform("base_link", "camera_link_6", tf2::TimePointZero);
    auto tf_s = tf_buffer_->lookupTransform("base_link", "laser_link_6", tf2::TimePointZero);
    T_m_curr = tf2::transformToEigen(tf_m);
    T_s_curr = tf2::transformToEigen(tf_s);
    tf_success = true;
  } catch (const tf2::TransformException &ex) {
    // 忽略TF错误，等待下一帧
  }
  if (tf_success) {
    // 1. 计算当前的相对变换矩阵 (从主臂看从臂)
    // T_rel = inv(T_master) * T_slave
    Eigen::Isometry3d T_rel_curr = T_m_curr.inverse() * T_s_curr;

    // 2. 使用静态变量存储初始时刻的相对关系
    static bool is_initialized = false;

    if (!is_initialized) {
        T_rel_init = T_rel_curr;
        is_initialized = true;
        RCLCPP_INFO(node_->get_logger(), ">>> 初始相对位姿已记录 <<<");
    }
    // 3. 计算误差 (相对于初始状态的漂移)
    // 位置误差 (米)
    double pos_err = (T_rel_curr.translation() - T_rel_init.translation()).norm();
    
    // 姿态误差 (弧度) - 计算两个旋转矩阵之间的角度差
    Eigen::Matrix3d R_diff = T_rel_curr.rotation() * T_rel_init.rotation().transpose();
    Eigen::AngleAxisd angle_axis(R_diff);
    double rot_err = angle_axis.angle();
    // 4. 打印状态
    // 如果 count >= 30，说明运动指令已停止，此时打印的是静止后的误差
    if (1 <= count_ && count_ <= 250) { 
        RCLCPP_INFO(node_->get_logger(), 
            "Step: %zu | Rel Pos Err: %.5f m | Rel Rot Err: %.5f rad", 
            count_, pos_err, rot_err);
        if (count_ >= 150)
        {
          ++count_;
        }
    }
  }

  if (count_ < 150)
  {
    // 创建主臂指令
    auto c_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    c_msg->header.stamp = node_->now();
    c_msg->header.frame_id = "base_link";
    c_msg->twist.linear.x = 0.1;
    c_msg->twist.linear.y = 0;
    c_msg->twist.linear.z = 0.1;
    c_msg->twist.angular.x = 0.02;
    c_msg->twist.angular.y = 0;
    c_msg->twist.angular.z = 0.05;

    // 2. 计算从臂指令
    // 注意：必须在 std::move(c_msg) 之前使用 c_msg 的数据
    // 我们将计算结果直接构造进 unique_ptr
    auto l_msg_val = compute_slave_command(*c_msg); 
    auto l_msg = std::make_unique<geometry_msgs::msg::TwistStamped>(l_msg_val);
    
    // 3. 发布
    // std::move 后 c_msg 变为空指针，所以必须最后做
    camera_twist_cmd_pub_->publish(std::move(c_msg));
    laser_twist_cmd_pub_->publish(std::move(l_msg));
    
    ++count_;
  }
}

geometry_msgs::msg::TwistStamped compute_slave_command(const geometry_msgs::msg::TwistStamped& msg)
{
  Eigen::Isometry3d T_base_master;
  Eigen::Isometry3d T_base_slave;
  
  // 准备一个默认的空消息用于错误返回
  geometry_msgs::msg::TwistStamped slave_msg;
  slave_msg.header.stamp = node_->now();
  slave_msg.header.frame_id = "base_link";

  try {
      // 获取最新变换
      auto tf_m = tf_buffer_->lookupTransform("base_link", "camera_link_6", tf2::TimePointZero);
      auto tf_s = tf_buffer_->lookupTransform("base_link", "laser_link_6", tf2::TimePointZero);
      
      T_base_master = tf2::transformToEigen(tf_m);
      T_base_slave = tf2::transformToEigen(tf_s);
  } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
          "Could not get transforms: %s", ex.what());
      // 5. 错误修复：必须返回一个对象，不能 return;
      return slave_msg; 
  }  

  // 6. 语法修复：msg是引用，使用 . 访问符
  Eigen::Vector3d v_m_base(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z);
  Eigen::Vector3d w_m_body(msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z);
  
  // --- 伴随变换计算 ---
  Eigen::Matrix3d R_m = T_base_master.rotation();
  Eigen::Vector3d p_m = T_base_master.translation();
  
  Eigen::Matrix3d R_s = T_base_slave.rotation();
  Eigen::Vector3d p_s = T_base_slave.translation();

  // B. 将主臂角速度转到 Base Frame 
  Eigen::Vector3d w_m_base = R_m * w_m_body;
  RCLCPP_INFO(node_->get_logger(), "w_m_base: [%.3f, %.3f, %.3f]", 
    w_m_base.x(), w_m_base.y(), w_m_base.z());

  // C. 计算力臂矢量
  Eigen::Vector3d r_ms = p_s - p_m;
  RCLCPP_INFO(node_->get_logger(), "Radius: [%.3f, %.3f, %.3f]", 
    r_ms.x(), r_ms.y(), r_ms.z());
  // D. 速度传递
  Eigen::Vector3d v_s_base = v_m_base + w_m_base.cross(r_ms);
  Eigen::Vector3d v_cross = w_m_base.cross(r_ms);
  RCLCPP_INFO(node_->get_logger(), "V_Cross: [%.3f, %.3f, %.3f]", 
    v_cross.x(), v_cross.y(), v_cross.z());
  // E. 角速度传递
  Eigen::Vector3d w_s_base = w_m_base;
  // F. 将从臂角速度转回 Body Frame
  Eigen::Vector3d w_s_body = R_s.transpose() * w_s_base;
  
  // 1. 计算理想位置 (Target)
  Eigen::Isometry3d T_target = T_base_master * T_rel_init; // T_rel_init 需要在外面存一下
  // 2. 计算误差向量
  Eigen::Vector3d p_err_vec = T_target.translation() - p_s;
  RCLCPP_INFO(node_->get_logger(), "Err Norm:[ %.4f,%.4f,%.4f]", p_err_vec.x(),p_err_vec.y(),p_err_vec.z());
  // 3. 施加反馈控制 (P增益设为 1.0 ~ 2.0 试试)
  v_s_base += 2.0 * p_err_vec;

  // --- 填充返回值 ---
  slave_msg.twist.linear.x = v_s_base.x();
  slave_msg.twist.linear.y = v_s_base.y();
  slave_msg.twist.linear.z = v_s_base.z();

  slave_msg.twist.angular.x = w_s_body.x();
  slave_msg.twist.angular.y = w_s_body.y();
  slave_msg.twist.angular.z = w_s_body.z();

  return slave_msg;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.use_intra_process_comms(false);
  
  node_ = std::make_shared<rclcpp::Node>("dual_servo_commander", node_options);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  camera_joint_cmd_pub_ = node_->create_publisher<control_msgs::msg::JointJog>("/camera_servo_demo/camera_er7robot_servo/delta_joint_cmds", 10);
  camera_twist_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/camera_servo_demo/camera_er7robot_servo/delta_twist_cmds", 10);
  laser_joint_cmd_pub_ = node_->create_publisher<control_msgs::msg::JointJog>("/laser_servo_demo/laser_er7robot_servo/delta_joint_cmds", 10);
  laser_twist_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/laser_servo_demo/laser_er7robot_servo/delta_twist_cmds", 10);

  auto timer = node_->create_wall_timer(30ms, publish_master_commands);
  
  rclcpp::spin(node_);
  rclcpp::shutdown();
  return 0;
}