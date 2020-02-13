#ifndef COMPUTED_TORQUE_CONTROLLERS_VELOCITY_TASK_SPACE_CONTROLLER_HPP
#define COMPUTED_TORQUE_CONTROLLERS_VELOCITY_TASK_SPACE_CONTROLLER_HPP

#include <map>
#include <string>

#include <computed_torque_controllers/common_namespaces.hpp>
#include <computed_torque_controllers/effort_joint_hardware.hpp>
#include <computed_torque_controllers/velocity_task_space_model.hpp>
#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/time.h>

namespace computed_torque_controllers {

// controller frontend subscribing task space velocity commands & passing them to backend
class VelocityTaskSpaceController : public ci::Controller< hi::EffortJointInterface > {
public:
  VelocityTaskSpaceController() {}

  virtual ~VelocityTaskSpaceController() {}

  // required interfaces as a Controller

  virtual bool init(hi::RobotHW *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
    // init the controller backend
    if (!eff_jnt_hw_.init(hw, controller_nh)) {
      ROS_ERROR("VelocityTaskSpaceController::init(): Failed to init the joint hardware");
      return false;
    }
    if (!vel_ts_model_.init(controller_nh)) {
      ROS_ERROR("VelocityTaskSpaceController::init(): Failed to init the controller backend");
      return false;
    }

    // subscribe command for task space dofs
    cmd_sub_ = controller_nh.subscribe("command", 1, &VelocityTaskSpaceController::commandCB, this);

    return true;
  }

  virtual void starting(const ros::Time &time) {
    vel_ts_model_.reset();

    // reset velocity command
    std::map< std::string, double > cmd;
    cmd["linear_x"] = 0.;
    cmd["linear_y"] = 0.;
    cmd["linear_z"] = 0.;
    cmd["angular_x"] = 0.;
    cmd["angular_y"] = 0.;
    cmd["angular_z"] = 0.;
    cmd_buf_.writeFromNonRT(cmd);
  }

  virtual void update(const ros::Time &time, const ros::Duration &period) {
    //
    vel_ts_model_.update(eff_jnt_hw_.getPositions(), eff_jnt_hw_.getVelocities(), period);

    // compute effort commands based on states & setpoints
    const std::map< std::string, double > eff_commands(
        vel_ts_model_.computeEffortCommands(*cmd_buf_.readFromRT(), period));

    // set effort commands to the hardware
    eff_jnt_hw_.setCommands(eff_commands);
  }

  virtual void stopping(const ros::Time &time) {}

private:
  void commandCB(const geometry_msgs::TwistConstPtr &msg) {
    // convert received message to velocity command
    std::map< std::string, double > cmd;
    cmd["linear_x"] = msg->linear.x;
    cmd["linear_y"] = msg->linear.y;
    cmd["linear_z"] = msg->linear.z;
    cmd["angular_x"] = msg->angular.x;
    cmd["angular_y"] = msg->angular.y;
    cmd["angular_z"] = msg->angular.z;

    // remember the command to use in update()
    cmd_buf_.writeFromNonRT(cmd);
  }

private:
  EffortJointHardware eff_jnt_hw_;
  VelocityTaskSpaceModel vel_ts_model_;
  rt::RealtimeBuffer< std::map< std::string, double > > cmd_buf_;
  ros::Subscriber cmd_sub_;
};
} // namespace computed_torque_controllers

#endif