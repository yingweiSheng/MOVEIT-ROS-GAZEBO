// 性质：该节点作为dual_arm_lock_servo的客户端，也是网关节点的订阅者
// 作用：订阅网关节点发布的话题，在subscription的回调函数中向dual_arm_lock_servo发布动作请求
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <vector>
#include "dual_arm_interfaces/action/dual_arm_lock_servo.hpp"

class DualArmTestClient : public rclcpp::Node
{
public:
    using DualArmLockServo = dual_arm_interfaces::action::DualArmLockServo;
    using GoalHandleDualArmLockServo = rclcpp_action::ClientGoalHandle<DualArmLockServo>;

    DualArmTestClient(): Node("dual_arm_test_client")
    {
        // 创建订阅者，订阅到命令时进入回调函数，作为客户端发送命令
        command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/bridge/master_arm_cmd",
            1,
            std::bind(&DualArmTestClient::send_goal,this,std::placeholders::_1)
        );

        // 创建客户端
        this->client_ptr_ = rclcpp_action::create_client<DualArmLockServo>(
            this,
            "/dual_commander_node/dual_arm_lock_servo" 
        );

        // 上传运动结果返回主程序
        feedback_to_main_ = this->create_publisher<std_msgs::msg::String>(
            "/bridge/cmd_feedback", 1);
            
        RCLCPP_INFO(this->get_logger(),"action client start and ready to sub command");
    }

private:
    rclcpp_action::Client<DualArmLockServo>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_to_main_;
    
    void send_goal(const std_msgs::msg::String& command)
    {
        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "收到原始 JSON: %s", command.data.c_str());
        auto data = nlohmann::json::parse(command.data);
        
        std::vector<float> master_vel = data["master_velocity"];
        float execute_t = data["execute_time"];
        float frequency = data["frequency"];

        auto goal_msg = DualArmLockServo::Goal();
        for (int i=0;i<6;++i)
        {
            goal_msg.master_velocity[i] = master_vel[i];
        }
        goal_msg.execute_time = execute_t; 
        goal_msg.frequency = frequency;

        auto send_goal_options = rclcpp_action::Client<DualArmLockServo>::SendGoalOptions();
        
        // 绑定回调函数
        send_goal_options.goal_response_callback =
            std::bind(&DualArmTestClient::goal_response_callback, this, std::placeholders::_1);
            
        send_goal_options.feedback_callback =
            std::bind(&DualArmTestClient::feedback_callback, this, std::placeholders::_1,std::placeholders::_2);
            
        send_goal_options.result_callback =
            std::bind(&DualArmTestClient::result_callback, this, std::placeholders::_1);

        // 发送异步请求
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    // --- 1. 收到服务端是否“接受”目标的响应 ---
    void goal_response_callback(const GoalHandleDualArmLockServo::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal 被服务端拒绝 (REJECTED)");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal 被服务端接受 (ACCEPTED)，开始执行...");
        }
    }

    // --- 2. 收到执行过程中的反馈 ---
    void feedback_callback(
        GoalHandleDualArmLockServo::SharedPtr,
        const std::shared_ptr<const DualArmLockServo::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "进度反馈: 已运行 %.2f 秒", feedback->time_elapsed);
    }

    // --- 3. 收到最终结果 ---
    void result_callback(const GoalHandleDualArmLockServo::WrappedResult & result)
    {
        nlohmann::json response_json;

        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "任务成功 (SUCCEEDED)");
                response_json["status"] = "success";
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "任务被拒绝 (ABORTED)");
                response_json["status"] = "aborted";
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "任务中途取消 (CANCELED)");
                response_json["status"] = "canceled";
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "未知结果代码");
                response_json["status"] = "unknown";
                break;
        }

        // 打印服务端传回的最终位置 (float64[7])
        if (result.result) {
            std::stringstream ss;
            ss << "[";
            for(auto val : result.result->master_final_pose) {
                ss << val << ", ";
            }
            ss << "]";
            RCLCPP_INFO(this->get_logger(), "最终位姿: %s", ss.str().c_str());
            response_json["master_final_pose"] = result.result->master_final_pose;
            response_json["slave_final_pose"] = result.result->slave_final_pose;
            
            std_msgs::msg::String msg_feedback;
            msg_feedback.data = response_json.dump();
            feedback_to_main_->publish(msg_feedback);
            RCLCPP_INFO(this->get_logger(), "发送给主控的反馈: %s", msg_feedback.data.c_str());
        }

    }
}; 

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DualArmTestClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}