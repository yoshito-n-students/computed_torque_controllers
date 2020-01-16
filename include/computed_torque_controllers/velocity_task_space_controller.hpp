#ifndef COMPUTED_TORQUE_CONTROLLERS_VELOCITY_TASK_SPACE_CONTROLLER_HPP
#define COMPUTED_TORQUE_CONTROLLERS_VELOCITY_TASK_SPACE_CONTROLLER_HPP

#include <map>
#include <string>

#include <computed_torque_controllers/common_namespaces.hpp>
#include <computed_torque_controllers/task_space_controller_core.hpp>
#include <controller_interface/multi_interface_controller.h>
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
class VelocityTaskSpaceController
    : public ci::MultiInterfaceController< hi::JointStateInterface, hi::EffortJointInterface > {
public:
  VelocityTaskSpaceController() {}

  virtual ~VelocityTaskSpaceController() {}

  // required interfaces as a Controller

  virtual bool init(hi::RobotHW *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
    // init the controller backend
    if (!controller_core_.init(hw, controller_nh)) {
      ROS_ERROR("VelocityTaskSpaceController::init(): Failed to init the controller backend");
      return false;
    }

    // subscribe command for task space dofs
    cmd_sub_ = controller_nh.subscribe("command", 1, &VelocityTaskSpaceController::commandCB, this);

    return true;
  }

  virtual void starting(const ros::Time &time) {
    controller_core_.starting();

    // reset velocity command
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.;
    cmd.linear.y = 0.;
    cmd.linear.z = 0.;
    cmd.angular.x = 0.;
    cmd.angular.y = 0.;
    cmd.angular.z = 0.;
    cmd_buf_.writeFromNonRT(cmd);
  }

  virtual void update(const ros::Time &time, const ros::Duration &period) {
    // populate setpoints from subscribed velocity commands
    std::map< std::string, double > setpoints;
    const geometry_msgs::Twist cmd(*cmd_buf_.readFromRT());
    setpoints["linear_x"] = cmd.linear.x;
    setpoints["linear_y"] = cmd.linear.y;
    setpoints["linear_z"] = cmd.linear.z;
    setpoints["angular_x"] = cmd.angular.x;
    setpoints["angular_y"] = cmd.angular.y;
    setpoints["angular_z"] = cmd.angular.z;

    // update the backend
    controller_core_.update(period,
                            /* pos_setpoints = */ std::map< std::string, double >(),
                            /* vel_setpoints = */ setpoints);
  }

  virtual void stopping(const ros::Time &time) { controller_core_.stopping(); }

private:
  void commandCB(const geometry_msgs::TwistConstPtr &cmd) { cmd_buf_.writeFromNonRT(*cmd); }

private:
  TaskSpaceControllerCore controller_core_;
  rt::RealtimeBuffer< geometry_msgs::Twist > cmd_buf_;
  ros::Subscriber cmd_sub_;
};
} // namespace computed_torque_controllers

#endif