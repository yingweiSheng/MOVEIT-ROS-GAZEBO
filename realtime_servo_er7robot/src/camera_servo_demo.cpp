#include <rclcpp/rclcpp.hpp>
#include <moveit_servo/servo.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

static const rclcpp::Logger LOGGER =
  rclcpp::get_logger("camera_er7robot_servo_node");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(false);
  // options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("camera_er7robot_servo", options);

  /* ================= Planning Scene Monitor ================= */
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node, "robot_description", tf_buffer, "planning_scene_monitor");

  if (psm->getPlanningScene())
  {
    psm->startStateMonitor("/joint_states");
    psm->setPlanningScenePublishingFrequency(25);
    psm->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
        "/moveit_servo/publish_planning_scene");
    psm->startSceneMonitor();
    psm->providePlanningSceneService();
  }
  else{
        RCLCPP_ERROR(LOGGER, "PlanningSceneMonitor not configured");
        return EXIT_FAILURE;
  }
  
  /* ================= Servo Parameters ================= */
  auto servo_parameters =
      moveit_servo::ServoParameters::makeServoParameters(node,"camera_servo_config");

  if (!servo_parameters)
  {
    RCLCPP_FATAL(LOGGER, "Failed to load Servo parameters");
    return EXIT_FAILURE;
  }

  /* ================= Servo Instance ================= */
  auto servo = std::make_unique<moveit_servo::Servo>(
      node, servo_parameters, psm);

  servo->start();

  RCLCPP_INFO(LOGGER,
              "MoveIt Servo started for group [%s]",
              servo_parameters->move_group_name.c_str());

  /* ================= Spin ================= */
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
