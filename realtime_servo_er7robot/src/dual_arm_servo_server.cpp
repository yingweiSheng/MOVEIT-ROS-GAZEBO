#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <mutex>
#include <sstream>
#include "dual_arm_interfaces/action/dual_arm_lock_servo.hpp"

class DualArmSyncNode : public rclcpp::Node
{
public:
    using DualArmLockServo = dual_arm_interfaces::action::DualArmLockServo;
    using GoalHandleDualArmLockServo = rclcpp_action::ServerGoalHandle<DualArmLockServo>;
    
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

            // 创建 Action Server
            // 需要绑定三个核心回调：处理目标请求、处理取消请求、处理任务接受后的执行
            this->action_server_ = rclcpp_action::create_server<DualArmLockServo>(
                this,
                "dual_arm_lock_servo", // Action Topic 名称
                std::bind(&DualArmSyncNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&DualArmSyncNode::handle_cancel, this, std::placeholders::_1),
                std::bind(&DualArmSyncNode::handle_accepted, this, std::placeholders::_1)
            );

            RCLCPP_INFO(this->get_logger(), "DualArmLockServo Action Server Ready.");
            return true;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "DualArmLockServo Action Server Init failed: %s", e.what());
            return false;
        }
    }

private:
    // Action callback
    // handle_goal : 是否接受客户端的请求
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const DualArmLockServo::Goal> goal // 接受请求后才会生成goal_handle管理器
    )
    {   
        // 检查发送来的命令是否符合规范
        RCLCPP_INFO(this->get_logger(), "收到目标请求，正在进行数据校验...");
        
        if (goal->execute_time <= 0) 
        {
            RCLCPP_WARN(this->get_logger(), "拒绝目标: 运行时间必须大于0 (当前: %.2f)", goal->execute_time);
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (goal->frequency < 1.0 || goal->frequency > 500.0) 
        {
            RCLCPP_WARN(this->get_logger(), "拒绝目标: 频率 %.2f Hz 超出允许范围 [1, 500]", goal->frequency);
            return rclcpp_action::GoalResponse::REJECT;
        }

        for (size_t i = 0; i < 6; ++i) 
        {
        if (!std::isfinite(goal->master_velocity[i])) {
            RCLCPP_WARN(this->get_logger(), "拒绝目标: 速度指令包含无效数值(NaN或Inf)在索引 %ld", i);
            return rclcpp_action::GoalResponse::REJECT;
            }
        }
        const double MAX_LINEAR_SPEED = 2.7; 
        const double MAX_ANGULAR_SPEED = 0.8; 

        for (size_t i = 0; i < 3; ++i) {
            if (std::abs(goal->master_velocity[i]) > MAX_LINEAR_SPEED) {
                RCLCPP_WARN(this->get_logger(), "拒绝目标: 线速度 %.2f 超过安全阈值 %.2f", 
                    goal->master_velocity[i], MAX_LINEAR_SPEED);
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        for (size_t i = 3; i < 6; ++i) {
            if (std::abs(goal->master_velocity[i]) > MAX_ANGULAR_SPEED) {
                RCLCPP_WARN(this->get_logger(), "拒绝目标: 角速度 %.2f 超过安全阈值 %.2f", 
                    goal->master_velocity[i], MAX_ANGULAR_SPEED);
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        RCLCPP_INFO(this->get_logger(), "目标校验通过，准备执行");

        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // handle_cancel: 当客户端中途取消请求时的反馈（request已接受）
    // 这里取消请求的结果是：goal_handle的状态从“Executing”变更为“Canceling”;
    // 但是并不会停止execute机器人的线程，需要在execute线程中单独处理
    // 后续当主程序选择“解锁”，则可以向服务端发送“取消”，在线程中查询“取消”状态并停止
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleDualArmLockServo> goal_handle
    )
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // 当goal被接受后，开始执行
    void handle_accepted(
        const std::shared_ptr<GoalHandleDualArmLockServo> goal_handle
    )
    {   // execute是阻塞线程的，需要独立出来；detach则是防止handle_accepted结束时execute还未结束
        std::thread{std::bind(&DualArmSyncNode::execute,this,goal_handle)}.detach();
    }

    void execute(
        const std::shared_ptr<GoalHandleDualArmLockServo> goal_handle
    )
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal...");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<DualArmLockServo::Feedback>();
        auto result = std::make_shared<DualArmLockServo::Result>();
        is_initialized_ = false;

        // 计算总循环次数
        double period_sec = 1.0 / goal->frequency; 
        size_t total_steps = static_cast<size_t>(goal->execute_time * goal->frequency);
        
        rclcpp::Rate loop_rate(goal->frequency);
        
        // 这里我准备分函数发布主从臂速度，同时设定零速度发布函数，
        // 用于应对可能发生的错误，从而在真实情况下协调机械臂。
        //rclcpp::ok(),防止循环无法停止
        for (size_t i = 0; i < total_steps && rclcpp::ok(); ++i) 
        {   
            // 拷贝robot_state
            moveit::core::RobotStatePtr snapshot_state;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                snapshot_state = std::make_shared<moveit::core::RobotState>(*robot_state_);
            }
            
            // 检查客户端是否发起“取消”请求（解锁）
            // 取消时，应当传回当前状态
            if (goal_handle->is_canceling()) 
            {
                fill_result(snapshot_state,master_tip_link_,result->master_final_pose);
                fill_result(snapshot_state,slave_tip_link_,result->slave_final_pose);
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                // 发送零速停止
                publishZeroVelocity();
                return;
            }

            // 计算主臂实际速度 (雅可比)
            // 如果雅克比计算出错/不稳定，，而不是直接return
            // 先假定不出错，将流程写完
            Eigen::Vector3d v_m_curr, w_m_curr;
            Eigen::Isometry3d T_m, T_s, T_m_prefix, T_s_prefix;
            
            calcMasterVelocity(snapshot_state,v_m_curr, w_m_curr);
            calcTransforms(snapshot_state, T_m, T_s, T_m_prefix, T_s_prefix);
            calcAndPublishSlaveCommand(v_m_curr, w_m_curr, T_m, T_s, T_m_prefix,T_s_prefix,i);
            publishMasterVelocity(goal);
            feedback->time_elapsed = (i+1)*period_sec;
            goal_handle->publish_feedback(feedback);
            loop_rate.sleep();
        }

        publishZeroVelocity();
        if (rclcpp::ok()) {
            // 获取最终主臂位姿填入 Result
            std::lock_guard<std::mutex> lock(state_mutex_);

            result->success = true;
            fill_result(robot_state_,master_tip_link_,result->master_final_pose);
            fill_result(robot_state_,slave_tip_link_,result->slave_final_pose);

            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }

    }
    void fill_result(
        const moveit::core::RobotStatePtr& state_ptr,
        const std::string& link_name,
        std::array<double,7>& output_array
    )
    {
        const Eigen::Isometry3d& T_end = state_ptr->getGlobalLinkTransform(link_name);
        Eigen::Vector3d pos = T_end.translation();
        Eigen::Quaterniond q(T_end.rotation());
        q.normalize();

        // 赋值
        output_array[0] = pos.x();
        output_array[1] = pos.y();
        output_array[2] = pos.z();
        output_array[3] = q.x();
        output_array[4] = q.y();
        output_array[5] = q.z();
        output_array[6] = q.w();
    }
    void publishZeroVelocity()
    {
        auto msg_m = std::make_unique<geometry_msgs::msg::TwistStamped>();
        auto msg_s = std::make_unique<geometry_msgs::msg::TwistStamped>();
        msg_m->header.stamp = this->now();
        msg_m->header.frame_id = "base_link";
        msg_s->header.stamp = this->now();
        msg_s->header.frame_id = "base_link";
        master_cmd_pub_->publish(std::move(msg_m));
        slave_cmd_pub_->publish(std::move(msg_s));
    }

    void publishMasterVelocity(
        const std::shared_ptr<const DualArmLockServo::Goal>& goal)
    {   
        // make_unique是publish要求的格式
        auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        msg->header.stamp = this->now();
        msg->header.frame_id = "base_link"; 
        msg->twist.linear.x = goal->master_velocity[0];
        msg->twist.linear.y = goal->master_velocity[1];
        msg->twist.linear.z = goal->master_velocity[2];
        msg->twist.angular.x = goal->master_velocity[3];
        msg->twist.angular.y = goal->master_velocity[4];
        msg->twist.angular.z = goal->master_velocity[5];
        master_cmd_pub_->publish(std::move(msg));
    }


    void updateStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        // 1. 保存最新的消息指针
        last_joint_state_msg_ = msg;
        
        // 2. 更新 RobotState 
        robot_state_->setVariableValues(*msg);
        robot_state_->updateLinkTransforms();
    }
    
    // --- 获取joint_state并转换为主臂末端速度 ---
    bool calcMasterVelocity
    (
        moveit::core::RobotStatePtr state_ptr,
        Eigen::Vector3d& v_out, 
        Eigen::Vector3d& w_out
    )
    {
        Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
        Eigen::MatrixXd jacobian;
        bool success = state_ptr->getJacobian(
            master_joint_group_,
            state_ptr->getLinkModel(master_tip_link_),
            reference_point_position,
            jacobian
        );

        if (!success) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Jacobian computation failed");
            return false;
        }

        // 获取关节速度 q_dot 
        std::vector<double> q_dot_vec;
        state_ptr->copyJointGroupVelocities(master_joint_group_, q_dot_vec);
        Eigen::VectorXd q_dot = Eigen::Map<Eigen::VectorXd>(q_dot_vec.data(), q_dot_vec.size());
        
        // Twist = J * q_dot 
        Eigen::VectorXd twist = jacobian * q_dot;
        
        // 分离线速度和角速度
        v_out = twist.head(3);
        w_out = twist.tail(3);
        
        return true;
    }

    // --- 变换矩阵 ---
    bool calcTransforms(
        moveit::core::RobotStatePtr state_ptr,
        Eigen::Isometry3d& T_m_out, 
        Eigen::Isometry3d& T_s_out,
        Eigen::Isometry3d& T_m_prefix_out,
        Eigen::Isometry3d& T_s_prefix_out)
    {
        T_m_out = state_ptr->getGlobalLinkTransform(master_tip_link_);
        T_s_out = state_ptr->getGlobalLinkTransform(slave_tip_link_);
        T_m_prefix_out = state_ptr->getGlobalLinkTransform("camera_base_link");
        T_s_prefix_out = state_ptr->getGlobalLinkTransform("laser_base_link");
        return true;
    }

    // --- 计算从臂末端速度 ---
    // 如果真实情况下机械臂抖动严重，可以尝试平滑速度或者采用主臂指令速度作为前馈
    void calcAndPublishSlaveCommand(
        const Eigen::Vector3d& v_m, const Eigen::Vector3d& w_m,
        const Eigen::Isometry3d& T_m, const Eigen::Isometry3d& T_s,
        const Eigen::Isometry3d& T_m_prefix, const Eigen::Isometry3d& T_s_prefix,
        size_t count
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

        if (count % 50 == 0)
        {
        RCLCPP_INFO(this->get_logger(), "Step: %zu | Rel Pos Err: %.5f m | Rel Rot Err: %.5f rad", count, pos_err, rot_err);
        }

        // vm/wm是 prefix_base_frame 下的了,
        // 目前的仿真环境下(srdf中的规划组设定决定)，servo的命令只能到prefix_base_link
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

        // 验证jacobian计算得到的w_base是否正确          
        slave_cmd_pub_->publish(std::move(slave_msg));    
    }

    // 成员变量
    rclcpp_action::Server<DualArmLockServo>::SharedPtr action_server_;
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