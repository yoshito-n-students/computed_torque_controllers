#ifndef COMPUTED_TORQUE_CONTROLLERS_POSITION_TASK_SPACE_CONTROLLER_HPP
#define COMPUTED_TORQUE_CONTROLLERS_POSITION_TASK_SPACE_CONTROLLER_HPP

#include <map>
#include <string>

#include <computed_torque_controllers/common_namespaces.hpp>
#include <computed_torque_controllers/effort_joint_hardware.hpp>
#include <computed_torque_controllers/position_task_space_model.hpp>
#include <controller_interface/controller.h>
#include <geometry_msgs/Pose.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/time.h>

#include <Eigen/Core>

namespace computed_torque_controllers {

// controller frontend subscribing task space position commands & passing them to backend
class PositionTaskSpaceController : public ci::Controller< hi::EffortJointInterface > {
public:
  PositionTaskSpaceController() {}

  virtual ~PositionTaskSpaceController() {}

  // required interfaces as a Controller

  virtual bool init(hi::RobotHW *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
    // init the controller backend
    if (!eff_jnt_hw_.init(hw, controller_nh)) {
      ROS_ERROR("PositionTaskSpaceController::init(): Failed to init the joint hardware");
      return false;
    }
    if (!pos_ts_model_.init(controller_nh)) {
      ROS_ERROR("PositionTaskSpaceController::init(): Failed to init the controller backend");
      return false;
    }

    // subscribe command for task space dofs
    cmd_sub_ = controller_nh.subscribe("command", 1, &PositionTaskSpaceController::commandCB, this);

    return true;
  }

  virtual void starting(const ros::Time &time) {
    pos_ts_model_.reset();

    // reset position command by present position
    pos_ts_model_.update(eff_jnt_hw_.getPositions(), eff_jnt_hw_.getVelocities(), ros::Duration(0));
    cmd_buf_.writeFromNonRT(pos_ts_model_.getPositions());
  }

  virtual void update(const ros::Time &time, const ros::Duration &period) {
    // get current hardware joint states
    pos_ts_model_.update(eff_jnt_hw_.getPositions(), eff_jnt_hw_.getVelocities(), period);

    // compute effort commands based on states & setpoints
    const std::map< std::string, double > eff_commands(
        pos_ts_model_.computeEffortCommands(*cmd_buf_.readFromRT(), period));

    // set effort commands to the hardware
    eff_jnt_hw_.setCommands(eff_commands);
  }

  virtual void stopping(const ros::Time &time) {}

private:
  void commandCB(const geometry_msgs::PoseConstPtr &msg) {
    // convert command message for the backend controller
    std::map< std::string, double > cmd;
    const Eigen::AngleAxisd aa(Eigen::Quaterniond(msg->orientation.w, msg->orientation.x,
                                                  msg->orientation.y, msg->orientation.z));
    const Eigen::Vector3d angular(aa.angle() * aa.axis());
    cmd["linear_x"] = msg->position.x;
    cmd["linear_y"] = msg->position.y;
    cmd["linear_z"] = msg->position.z;
    cmd["angular_x"] = angular[0];
    cmd["angular_y"] = angular[1];
    cmd["angular_z"] = angular[2];

    // store converted command
    cmd_buf_.writeFromNonRT(cmd);
  }

private:
  EffortJointHardware eff_jnt_hw_;
  PositionTaskSpaceModel pos_ts_model_;
  rt::RealtimeBuffer< std::map< std::string, double > > cmd_buf_;
  ros::Subscriber cmd_sub_;
};
} // namespace computed_torque_controllers

#endif