#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>


// //PoseParams：坐标和姿态------------------------------------------------------------
// struct RPYPoseParams {
//     double pos_x, pos_y, pos_z;
//     double roll, pitch, yaw;
// };

// //输入欧拉角，输出四元数，因为欧拉角更便于想像---------------------------------------------------
// auto trans_rpyToquat(double roll,double pitch,double yaw){
//     tf2::Quaternion q;
//     q.setRPY(roll,pitch,yaw);
//     q.normalized();
//     return q;
// };

// //传入参考基准以及pose，返回Posestamped类型的目标-----------------------------------------------
// geometry_msgs::msg::PoseStamped create_rpyTargetPose(RPYPoseParams& params,const std::string frame_id="base_link") 
// {
//     auto q = trans_rpyToquat(params.roll,params.pitch,params.yaw);
    
//     geometry_msgs::msg::PoseStamped msg;
//     msg.header.frame_id = frame_id;
//     msg.pose.position.x = params.pos_x;
//     msg.pose.position.y = params.pos_y;
//     msg.pose.position.z = params.pos_z;  
//     msg.pose.orientation.x = q.getX();
//     msg.pose.orientation.y = q.getY(); 
//     msg.pose.orientation.z = q.getZ();
//     msg.pose.orientation.w = q.getW();
//     return msg;
// };

// geometry_msgs::msg::PoseStamped create_qTargetPose(geometry_msgs::msg::PoseStamped& params,const std::string frame_id="base_link") 
// {
//     geometry_msgs::msg::PoseStamped msg;
//     msg.header.frame_id = frame_id;
//     msg.pose.position.x = params.pose.position.x;
//     msg.pose.position.y = params.pose.position.y;
//     msg.pose.position.z = params.pose.position.z;  
//     msg.pose.orientation.x = params.pose.orientation.x;
//     msg.pose.orientation.y = params.pose.orientation.y; 
//     msg.pose.orientation.z = params.pose.orientation.z;
//     msg.pose.orientation.w = params.pose.orientation.w;
//     return msg;
// };

// //创建末端执行器的运动路径orientation限制-----------------------------------------------------
// moveit_msgs::msg::Constraints create_OrientationConstraint(
//     const std::string constraint_link_name,
//     const std::string base_link_name,
//     double x = 0,double y = 0,double z = 0,double w = 1,double tolerance=0.2)
// {   
//     moveit_msgs::msg::OrientationConstraint ocm;
//     ocm.link_name = constraint_link_name; //限制的link
//     ocm.header.frame_id = base_link_name; //相对base_link的限制
//     ocm.orientation.x = x;
//     ocm.orientation.y = y;
//     ocm.orientation.z = z;
//     ocm.orientation.w = w;
//     ocm.absolute_x_axis_tolerance = tolerance;
//     ocm.absolute_y_axis_tolerance = tolerance;
//     ocm.absolute_z_axis_tolerance = tolerance;
//     ocm.weight = 1.0; //权重为1.0,即优先考虑
//     moveit_msgs::msg::Constraints orientation_constraints;
//     orientation_constraints.orientation_constraints.push_back(ocm);
//     return orientation_constraints;
// };

// //arm初始化函数------------------------------------------------------------------------------
// //参数：
// //  arm：规划组实例；acc：加速度；vel:速度;pose/orien_tol:容差；plan__time:允许的误差
auto setArmFactors = [](moveit::planning_interface::MoveGroupInterface& arm, 
    double acceleration = 1.0, double velocity = 1.0, 
    double pose_tol = 0.05, double orientation_tol = 0.1,double plan_time = 30) {
    arm.setMaxAccelerationScalingFactor(acceleration);
    arm.setMaxVelocityScalingFactor(velocity);
    arm.setGoalPositionTolerance(pose_tol);
    arm.setGoalOrientationTolerance(orientation_tol);
    arm.setPlanningTime(plan_time);
};
//主函数-----------------------------------------------------------------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("gazebo_test_moveit",node_options);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

// MGI接口初始化-----------------------------------------------------------------------------------------
    bool use_sim_time = false;
    node->get_parameter("use_sim_time", use_sim_time);
    
    if (use_sim_time) {
        RCLCPP_INFO(node->get_logger(), "使用仿真时间");
    } else {
        RCLCPP_INFO(node->get_logger(), "使用系统时间");
    }

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "my_arm_group");

    setArmFactors(arm);


    auto target_pose = arm.getCurrentPose();


    target_pose.pose.position.x += 0.05;
    target_pose.pose.position.y -= 0.1;
    target_pose.pose.position.z -= 0.1;


    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose,"link6");

    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success1) {
        RCLCPP_INFO(node->get_logger(),"规划成功，准备执行");
        arm.execute(plan1);
    }
    else{
        RCLCPP_ERROR(node->get_logger(),"规划失败");
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}
