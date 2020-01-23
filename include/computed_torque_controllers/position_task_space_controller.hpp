#ifndef COMPUTED_TORQUE_CONTROLLERS_POSITION_TASK_SPACE_CONTROLLER_HPP
#define COMPUTED_TORQUE_CONTROLLERS_POSITION_TASK_SPACE_CONTROLLER_HPP

#include <map>
#include <string>

#include <computed_torque_controllers/common_namespaces.hpp>
#include <computed_torque_controllers/task_space_controller_core.hpp>
#include <controller_interface/multi_interface_controller.h>
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
class PositionTaskSpaceController
    : public ci::MultiInterfaceController< hi::JointStateInterface, hi::EffortJointInterface > {
public:
  PositionTaskSpaceController() {}

  virtual ~PositionTaskSpaceController() {}

  // required interfaces as a Controller

  virtual bool init(hi::RobotHW *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
    // init the controller backend
    if (!controller_core_.init(hw, controller_nh)) {
      ROS_ERROR("PositionTaskSpaceController::init(): Failed to init the controller backend");
      return false;
    }

    // subscribe command for task space dofs
    cmd_sub_ = controller_nh.subscribe("command", 1, &PositionTaskSpaceController::commandCB, this);

    return true;
  }

  virtual void starting(const ros::Time &time) {
    controller_core_.starting();

    // reset position command by present position
    cmd_buf_.writeFromNonRT(controller_core_.getEndEffectorPosition());
  }

  virtual void update(const ros::Time &time, const ros::Duration &period) {
    // update the backend
    controller_core_.update(period,
                            /* pos_setpoints = */ *cmd_buf_.readFromRT(),
                            /* vel_setpoints = */ std::map< std::string, double >());
  }

  virtual void stopping(const ros::Time &time) { controller_core_.stopping(); }

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
  TaskSpaceControllerCore controller_core_;
  rt::RealtimeBuffer< std::map< std::string, double > > cmd_buf_;
  ros::Subscriber cmd_sub_;
};
} // namespace computed_torque_controllers

#endif