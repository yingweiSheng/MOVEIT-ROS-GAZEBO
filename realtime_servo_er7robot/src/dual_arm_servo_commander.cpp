#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <mutex>

class DualArmSyncNode : public rclcpp::Node
{
public:
    // 构造函数：二段式初始化，只做参数声明
    DualArmSyncNode(const rclcpp::NodeOptions & options) : Node("dual_arm_sync_node", options)
    {
        // 声明参数，可以在 launch 文件中修改
        this->declare_parameter<std::string>("master_group_name", "camera_arm_group");
        this->declare_parameter<std::string>("slave_group_name", "laser_arm_group");
        this->declare_parameter<std::string>("master_tip_link", "camera_link_6");
        this->declare_parameter<std::string>("slave_tip_link", "laser_link_6");
        
        RCLCPP_INFO(this->get_logger(), "Node created, waiting for init()...");
    }

    // 初始化函数：加载模型，建立订阅和发布
    bool init()
    {
        try {
            // 1. 加载 Robot Model 
            robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
                shared_from_this(), "robot_description");
            robot_model_ = robot_model_loader_->getModel();

            // 2. 初始化 Robot State 
            robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
            robot_state_->setToDefaultValues(); 

            // 3. 获取关节组和 Link 名称
            std::string m_group_name = this->get_parameter("master_group_name").as_string();
            std::string s_group_name = this->get_parameter("slave_group_name").as_string();
            master_tip_link_ = this->get_parameter("master_tip_link").as_string();
            slave_tip_link_ = this->get_parameter("slave_tip_link").as_string();

            master_joint_group_ = robot_model_->getJointModelGroup(m_group_name);
            slave_joint_group_ = robot_model_->getJointModelGroup(s_group_name);

            if (!master_joint_group_ || !slave_joint_group_) {
                RCLCPP_ERROR(this->get_logger(), "Joint groups not found!");
                return false;
            }

            // 4. 创建发布者 
            master_cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
                "/camera_servo_demo/camera_er7robot_servo/delta_twist_cmds", 10);
            slave_cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
                "/laser_servo_demo/laser_er7robot_servo/delta_twist_cmds", 10);

            // 5. 创建订阅者 
            joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 1,
                std::bind(&DualArmSyncNode::updateStateCallback, this, std::placeholders::_1));
            timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&DualArmSyncNode::syncCallback, this));

            RCLCPP_INFO(this->get_logger(), "Dual Arm Sync Node Initialized Successfully.");
            return true;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Init failed: %s", e.what());
            return false;
        }
    }

private:
    void syncCallback()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        // 1. 计算主臂实际速度 (雅可比)
        Eigen::Vector3d v_m_curr, w_m_curr;
        if (!calcMasterVelocity(v_m_curr, w_m_curr)) return;

        // 2. 计算双臂变换矩阵 (FK)
        Eigen::Isometry3d T_m, T_s,T_m_prefix,T_s_prefix;
        calcTransforms(T_m, T_s,T_m_prefix,T_s_prefix);

        // 3. 计算从臂指令并发布
        calcAndPublishSlaveCommand(v_m_curr, w_m_curr, T_m, T_s, T_m_prefix,T_s_prefix);
    }

    void updateStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        // 1. 保存最新的消息指针 (或者深拷贝，shared_ptr 拷贝很快)
        last_joint_state_msg_ = msg;
        
        // 2. 更新 RobotState (这一步也可以放在定时器里，但放在这里能保证RobotState总是最新的)
        robot_state_->setVariableValues(*msg);
        robot_state_->updateLinkTransforms();
    }
    // --- 获取joint_state并转换为主臂末端速度 ---
    bool calcMasterVelocity(Eigen::Vector3d& v_out, Eigen::Vector3d& w_out)
    {
        Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
        Eigen::MatrixXd jacobian;
        
        // 
        bool success = robot_state_->getJacobian(
            master_joint_group_,
            robot_state_->getLinkModel(master_tip_link_),
            reference_point_position,
            jacobian
        );

        if (!success) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Jacobian computation failed");
            return false;
        }

        // 获取关节速度 q_dot 
        std::vector<double> q_dot_vec;
        robot_state_->copyJointGroupVelocities(master_joint_group_, q_dot_vec);
        Eigen::VectorXd q_dot = Eigen::Map<Eigen::VectorXd>(q_dot_vec.data(), q_dot_vec.size());

        // Twist = J * q_dot 
        Eigen::VectorXd twist = jacobian * q_dot;
        
        // 分离线速度和角速度
        v_out = twist.head(3);
        w_out = twist.tail(3);
        
        return true;
    }

    // --- 变换矩阵 ---
    void calcTransforms(Eigen::Isometry3d& T_m_out, Eigen::Isometry3d& T_s_out,Eigen::Isometry3d& T_m_prefix_out,Eigen::Isometry3d& T_s_prefix_out)
    {
        T_m_out = robot_state_->getGlobalLinkTransform(master_tip_link_);
        T_s_out = robot_state_->getGlobalLinkTransform(slave_tip_link_);
        T_m_prefix_out = robot_state_->getGlobalLinkTransform("camera_base_link");
        T_s_prefix_out = robot_state_->getGlobalLinkTransform("laser_base_link");
    }

    // --- 计算从臂末端速度 ---
    void calcAndPublishSlaveCommand(
        const Eigen::Vector3d& v_m, const Eigen::Vector3d& w_m,
        const Eigen::Isometry3d& T_m, const Eigen::Isometry3d& T_s,
        const Eigen::Isometry3d& T_m_prefix, const Eigen::Isometry3d& T_s_prefix
      )
    {
        if (!is_initialized_) 
        {
            T_rel_init = T_m.inverse() * T_s;
            is_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), ">>> Initial Relative Pose Recorded <<<");
            return;
        }
        // 计算当前时刻主从臂间的变换矩阵:T(master->slaver)
        Eigen::Isometry3d T_rel_curr = T_m.inverse() * T_s;
        // 位置误差 (米)
        double pos_err = (T_rel_curr.translation() - T_rel_init.translation()).norm();
        // 姿态误差 (弧度) - 计算两个旋转矩阵之间的角度差
        Eigen::Matrix3d R_diff = T_rel_curr.rotation() * T_rel_init.rotation().transpose();
        Eigen::AngleAxisd angle_axis(R_diff);
        double rot_err = angle_axis.angle();

        if (1 <= count && count <= 700) 
        { 
          if (count % 50 == 0)
          {
            RCLCPP_INFO(this->get_logger(), "Step: %zu | Rel Pos Err: %.5f m | Rel Rot Err: %.5f rad", count, pos_err, rot_err);
          }
          if (count > 500)
          {
            ++count;
          }
        }

        // vm/wm是 prefix_base_frame 下的了,
        // 目前的仿真环境下(srdf中的规划组设定决定)，servo的命令只能到prefix_base_link
        if (count <= 500)
        {
          Eigen::Vector3d p_m = T_m.translation();
          Eigen::Vector3d p_s = T_s.translation();
          Eigen::Matrix3d R_s = T_s.rotation();
          Eigen::Matrix3d R_s_prefix = T_s_prefix.rotation();
          Eigen::Matrix3d R_m_prefix = T_m_prefix.rotation();
          // base_link -----> prefix_base_link
          Eigen::Vector3d v_m_base = R_m_prefix * v_m;
          Eigen::Vector3d w_m_base = R_s_prefix * w_m; 



          // 计算力臂 ----base_link
          Eigen::Vector3d r_ms = p_s - p_m;
          // 速度传递 ---base_link
          Eigen::Vector3d v_feedforward = v_m_base + w_m_base.cross(r_ms);

          // 位置反馈 ----base_link
          Eigen::Isometry3d T_target = T_m * T_rel_init;
          Eigen::Vector3d p_err_vec = T_target.translation() - p_s;
          // 姿势反馈
          Eigen::Matrix3d w_err_vec = T_target.rotation() * R_s.transpose();
          Eigen::AngleAxisd angle_axis_err(w_err_vec);
          Eigen::Vector3d w_com = angle_axis_err.angle() * angle_axis_err.axis();
          // 反馈增益
          double Kp = 1.5;
          Eigen::Vector3d v_s_final = R_s_prefix.transpose() * (v_feedforward + (Kp * p_err_vec)); // 转换到prefix_base_link下
          Eigen::Vector3d w_s_final = R_s_prefix.transpose() * (w_m_base + (Kp * w_com)); 

          // G. 发布指令
          
          auto slave_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
          slave_msg->header.stamp = this->now();
          slave_msg->header.frame_id = "base_link"; 

          slave_msg->twist.linear.x = v_s_final.x();
          slave_msg->twist.linear.y = v_s_final.y();
          slave_msg->twist.linear.z = v_s_final.z();
          slave_msg->twist.angular.x = w_s_final.x();
          slave_msg->twist.angular.y = w_s_final.y();
          slave_msg->twist.angular.z = w_s_final.z();

          auto master_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
          master_msg->header.stamp = this->now();
          // master_msg->header.frame_id = robot_model_->getModelFrame();
          // all commands are based in "frame_id"------"base_link"
          master_msg->header.frame_id = "base_link";
          master_msg->twist.linear.x = 0;
          master_msg->twist.linear.y = 0.06;
          master_msg->twist.linear.z = 0;
          master_msg->twist.angular.x = 0;
          master_msg->twist.angular.y = 0.08;
          master_msg->twist.angular.z = 0.01;

          // 验证jacobian计算得到的w_base是否正确          
          slave_cmd_pub_->publish(std::move(slave_msg));
          master_cmd_pub_->publish(std::move(master_msg));
          if (count % 50 == 0)
          {
            RCLCPP_INFO(this->get_logger(), "主臂偏移----x:%.5f | y:%.5f | z:%.5f",p_m.x(),p_m.y(),p_m.z());
            RCLCPP_INFO(this->get_logger(), "力臂----x:%.5f | y:%.5f | z:%.5f",r_ms.x(),r_ms.y(),r_ms.z());
            RCLCPP_INFO(this->get_logger(), "速度传递----Vx:%.5f | Vy:%.5f | Vz:%.5f",v_feedforward.x(),v_feedforward.y(),v_feedforward.z());
            RCLCPP_INFO(this->get_logger(), "位置误差----err_px:%.5f | err_py:%.5f | err_pz:%.5f",p_err_vec.x(),p_err_vec.y(),p_err_vec.z());
            RCLCPP_INFO(this->get_logger(), "w_base:jacobian: wx:%.5f | wy:%.5f | wz:%.5f",w_m.x(),w_m.y(),w_m.z());
            RCLCPP_INFO(this->get_logger(), "v_base:jacobian: vx:%.5f | vy:%.5f | vz:%.5f",v_m.x(),v_m.y(),v_m.z());    
            RCLCPP_INFO(this->get_logger(), "w_base wx:%.5f | wy:%.5f | wz:%.5f",w_m_base.x(),w_m_base.y(),w_m_base.z());
            RCLCPP_INFO(this->get_logger(), "v_base vx:%.5f | vy:%.5f | vz:%.5f",v_m_base.x(),v_m_base.y(),v_m_base.z());                            
          }

          ++count;
        }
    }

    // 成员变量
    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    
    const moveit::core::JointModelGroup* master_joint_group_;
    const moveit::core::JointModelGroup* slave_joint_group_;
    std::string master_tip_link_;
    std::string slave_tip_link_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr master_cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr slave_cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    sensor_msgs::msg::JointState::SharedPtr last_joint_state_msg_;
    std::mutex state_mutex_;
    rclcpp::TimerBase::SharedPtr timer_;
    // 状态保持
    size_t count = 0;
    Eigen::Isometry3d T_rel_init;
    bool is_initialized_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // 必须启用参数服务以加载 robot_description 
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<DualArmSyncNode>(options);

    if (node->init()) {
        rclcpp::spin(node);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Initialization failed.");
    }

    rclcpp::shutdown();
    return 0;
}