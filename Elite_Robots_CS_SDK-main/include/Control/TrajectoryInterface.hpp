// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// TrajectoryInterface.hpp
// Provides the TrajectoryInterface class for robot trajectory control.
#ifndef __TRAJECTORY_INTERFACE_HPP__
#define __TRAJECTORY_INTERFACE_HPP__

#include <functional>
#include <memory>
#include "DataType.hpp"
#include "ReversePort.hpp"
#include "TcpServer.hpp"

namespace ELITE {

enum class TrajectoryMotionType : int {
    JOINT = 0,      // movej
    CARTESIAN = 1,  // movel
    SPLINE = 2      // spline
};

class TrajectoryInterface : public ReversePort {
   public:
    static const int TRAJECTORY_MESSAGE_LEN = 21;

    TrajectoryInterface() = delete;

    /**
     * @brief Construct a new Trajectory Interface object include TcpServer
     *
     * @param port Port the Server is started on
     * @param resource TCP resource shared pointer
     */
    TrajectoryInterface(int port, std::shared_ptr<TcpServer::StaticResource> resource);

    ~TrajectoryInterface();

    /**
     * @brief Register a callback for the robot-based trajectory execution completion.
     *
     *  One mode of robot control is to forward a complete trajectory to the robot for execution.
     *  When the execution is done, the callback function registered here will be triggered.
     *
     * @param cb Callback function that will be triggered in the event of finishing
     */
    void setMotionResultCallback(std::function<void(TrajectoryMotionResult)> cb) { motion_result_func_ = cb; }

    /**
     * @brief Writes a trajectory point onto the dedicated socket.
     *
     * @param positions Desired joint or cartesian positions
     * @param time Time for the robot to reach this point
     * @param blend_radius The radius to be used for blending between control points
     * @param cartesian True, if the point sent is cartesian, false if joint-based
     * @return true
     * @return false
     */
    bool writeTrajectoryPoint(const vector6d_t& positions, float time, float blend_radius, bool cartesian);

   private:
    std::function<void(TrajectoryMotionResult)> motion_result_func_;
};

}  // namespace ELITE

#endif
