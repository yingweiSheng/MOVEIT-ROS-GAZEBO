// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include <Elite/DashboardClient.hpp>
#include <Elite/DataType.hpp>
#include <Elite/EliteDriver.hpp>
#include <Elite/Log.hpp>
#include <Elite/RtsiIOInterface.hpp>

#include <boost/program_options.hpp>
#include <future>
#include <iostream>
#include <memory>
#include <thread>

using namespace ELITE;
namespace po = boost::program_options;

class TrajectoryControl {
   private:
    std::unique_ptr<EliteDriver> driver_;
    
    std::unique_ptr<DashboardClient> dashboard_;
    EliteDriverConfig config_;

   public:
    TrajectoryControl(const EliteDriverConfig& config) {
        config_ = config;
        driver_ = std::make_unique<EliteDriver>(config);
        dashboard_ = std::make_unique<DashboardClient>();

        ELITE_LOG_INFO("Connecting to the dashboard");
        if (!dashboard_->connect(config.robot_ip)) {
            ELITE_LOG_FATAL("Failed to connect to the dashboard.");
            throw std::runtime_error("Failed to connect to the dashboard.");
        }
        ELITE_LOG_INFO("Successfully connected to the dashboard");
    }

    ~TrajectoryControl() {
        if (dashboard_) {
            dashboard_->disconnect();
        }
        driver_->stopControl();
    }

    bool startControl() {
        ELITE_LOG_INFO("Start powering on...");
        if (!dashboard_->powerOn()) {
            ELITE_LOG_FATAL("Power-on failed");
            return false;
        }
        ELITE_LOG_INFO("Power-on succeeded");

        ELITE_LOG_INFO("Start releasing brake...");
        if (!dashboard_->brakeRelease()) {
            ELITE_LOG_FATAL("Brake release failed");
            return false;
        }
        ELITE_LOG_INFO("Brake released");

        if (config_.headless_mode) {
            if (!driver_->isRobotConnected()) {
                if (!driver_->sendExternalControlScript()) {
                    ELITE_LOG_FATAL("Fail to send external control script");
                    return false;
                }
            }
        } else {
            if (!dashboard_->playProgram()) {
                ELITE_LOG_FATAL("Fail to play program");
                return false;
            }
        }

        ELITE_LOG_INFO("Wait external control script run...");
        while (!driver_->isRobotConnected()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        ELITE_LOG_INFO("External control script is running");
        return true;
    }

    bool moveTrajectory(const std::vector<vector6d_t>& target_points, float point_time, float blend_radius, bool is_cartesian) {
        std::promise<TrajectoryMotionResult> move_done_promise;
        driver_->setTrajectoryResultCallback([&](TrajectoryMotionResult result) { move_done_promise.set_value(result); });

        ELITE_LOG_INFO("Trajectory motion start");
        if(!driver_->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::START, target_points.size(), 200)) {
            ELITE_LOG_ERROR("Failed to start trajectory motion");
            return false;
        }

        for (const auto& joints : target_points) {
            if (!driver_->writeTrajectoryPoint(joints, point_time, blend_radius, is_cartesian)) {
                ELITE_LOG_ERROR("Failed to write trajectory point");
                return false;
            }
            // Send NOOP command to avoid timeout.
            if(!driver_->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::NOOP, 0, 200)) {
                ELITE_LOG_ERROR("Failed to send NOOP command");
                return false;
            }
        }

        std::future<TrajectoryMotionResult> move_done_future = move_done_promise.get_future();
        while (move_done_future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
            // Wait for the trajectory motion to complete, and send NOOP command to avoid timeout.
            if(!driver_->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::NOOP, 0, 200)) {
                ELITE_LOG_ERROR("Failed to send NOOP command");
                return false;
            }
        }
        auto result = move_done_future.get();
        ELITE_LOG_INFO("Trajectory motion completed with result: %d", result);

        if(!driver_->writeIdle(0)) {
            ELITE_LOG_ERROR("Failed to write idle command");
            return false;
        }

        return result == TrajectoryMotionResult::SUCCESS;
    }

    bool moveTo(const vector6d_t& point, float time, bool is_cartesian) {
        return moveTrajectory({point}, time, 0, is_cartesian);
    }
};

int main(int argc, const char** argv) {
    EliteDriverConfig config;
    // Parser param
    po::options_description desc(
        "Usage:\n"
        "\t./trajectory_example <--robot-ip=ip> [--local-ip=\"\"] [--use-headless-mode=true]\n"
        "Parameters:");
    desc.add_options()("help,h", "Print help message")("robot-ip", po::value<std::string>(&config.robot_ip)->required(),
                                                       "\tRequired. IP address of the robot.")(
        "use-headless-mode", po::value<bool>(&config.headless_mode)->required()->implicit_value(true),
        "\tRequired. Use headless mode.")("local-ip", po::value<std::string>(&config.local_ip)->default_value(""),
                                          "\tOptional. IP address of the local network interface.");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help")) {
            std::cout << desc << std::endl;
            return 0;
        }

        po::notify(vm);
    } catch (const po::error& e) {
        std::cerr << "Argument error: " << e.what() << "\n\n";
        std::cerr << desc << "\n";
        return 1;
    }

    if (config.headless_mode) {
        ELITE_LOG_WARN("Use headless mode. Please ensure the robot is not in local mode.");
    } else {
        ELITE_LOG_WARN(
            "It needs to be correctly configured, and the External Control plugin should be inserted into the task tree.");
    }

    config.script_file_path = "external_control.script";
    std::unique_ptr<TrajectoryControl> trajectory_control = std::make_unique<TrajectoryControl>(config);
    std::unique_ptr<RtsiIOInterface> rtsi_client = std::make_unique<RtsiIOInterface>("output_recipe.txt", "input_recipe.txt", 250);

    ELITE_LOG_INFO("Connecting to the RTSI");
    if (!rtsi_client->connect(config.robot_ip)) {
        ELITE_LOG_FATAL("Fail to connect or config to the RTSI.");
        throw std::runtime_error("Fail to connect or config to the RTSI");
    }
    ELITE_LOG_INFO("Successfully connected to the RTSI");

    ELITE_LOG_INFO("Starting trajectory control...");
    if(!trajectory_control->startControl()) {
        ELITE_LOG_FATAL("Failed to start trajectory control.");
        return 1;
    }
    ELITE_LOG_INFO("Trajectory control started");

    vector6d_t actual_joints = rtsi_client->getActualJointPositions();
    actual_joints[3] = -1.57;

    ELITE_LOG_INFO("Moving joints to target: [%lf, %lf, %lf, %lf, %lf, %lf]",
                   actual_joints[0], actual_joints[1], actual_joints[2], actual_joints[3], actual_joints[4], actual_joints[5]);
    if(!trajectory_control->moveTo(actual_joints, 3, false)) {
        ELITE_LOG_FATAL("Failed to move joints to target.");
        return 1;
    }
    ELITE_LOG_INFO("Joints moved to target");


    vector6d_t actual_pose = rtsi_client->getActualTCPPose();
    std::vector<vector6d_t> trajectory;

    actual_pose[2] -= 0.2;
    trajectory.push_back(actual_pose);


    actual_pose[1] -= 0.2;
    trajectory.push_back(actual_pose);

    actual_pose[1] += 0.2;
    actual_pose[2] += 0.2;
    trajectory.push_back(actual_pose);

    ELITE_LOG_INFO("Moving joints to target");
    if(!trajectory_control->moveTrajectory(trajectory, 3, 0, true)) {
        ELITE_LOG_FATAL("Failed to move trajectory.");
        return 1;
    }
    ELITE_LOG_INFO("Joints moved to target");

    return 0;
}