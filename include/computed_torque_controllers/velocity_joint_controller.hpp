#ifndef COMPUTED_TORQUE_CONTROLLERS_VELOCITY_JOINT_CONTROLLER_HPP
#define COMPUTED_TORQUE_CONTROLLERS_VELOCITY_JOINT_CONTROLLER_HPP

#include <map>
#include <string>

#include <computed_torque_controllers/common_namespaces.hpp>
#include <computed_torque_controllers/joint_controller_core.hpp>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>

namespace computed_torque_controllers {

// controller frontend subscribing joint velocity commands & passing them to backend
class VelocityJointController
    : public ci::MultiInterfaceController< hi::JointStateInterface, hi::EffortJointInterface > {
private:
  struct ControlledJointInfo {
    // in kinetic, rt::RealtimeBuffer<> does not have a const copy constructor
    //   --> ControlledJointInfo cannot have a const copy constructor
    //   --> BOOST_FOREACH(ControlledJointInfoMap::value_type &v, ctl_joints) does not compile
    // boost::optional<> is workaround to enable a const copy constructor.
    // actually, cmd_buf is always available.
    boost::optional< rt::RealtimeBuffer< double > > cmd_buf;
    ros::Subscriber cmd_sub;
  };
  typedef std::map< std::string, ControlledJointInfo > ControlledJointInfoMap;

public:
  VelocityJointController() {}

  virtual ~VelocityJointController() {}

  // required interfaces as a Controller

  virtual bool init(hi::RobotHW *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
    // init the controller backend
    if (!controller_core_.init(hw, controller_nh)) {
      ROS_ERROR("VelocityJointController::init(): Failed to init the controller backend");
      return false;
    }

    // subscribe commands for controlled joints
    BOOST_FOREACH (const std::string &name, controller_core_.getControlledJointNames()) {
      ControlledJointInfo &info(ctl_joints_[name]);
      info.cmd_buf.emplace();
      info.cmd_sub = controller_nh.subscribe< std_msgs::Float64 >(
          ros::names::append(name, "command"), 1,
          boost::bind(&VelocityJointController::commandCB, _1, info.cmd_buf.get_ptr()));
    }

    return true;
  }

  virtual void starting(const ros::Time &time) {
    controller_core_.starting();

    BOOST_FOREACH (ControlledJointInfoMap::value_type &joint_val, ctl_joints_) {
      ControlledJointInfo &info(joint_val.second);
      // reset velocity command
      info.cmd_buf->writeFromNonRT(0.);
    }
  }

  virtual void update(const ros::Time &time, const ros::Duration &period) {
    // populate setpoints from subscribed velocity commands
    std::map< std::string, double > setpoints;
    BOOST_FOREACH (ControlledJointInfoMap::value_type &joint_val, ctl_joints_) {
      const std::string &name(joint_val.first);
      ControlledJointInfo &info(joint_val.second);
      // copy velocity setpoint from the buffer
      setpoints[name] = *info.cmd_buf->readFromRT();
    }

    // update the backend
    controller_core_.update(period,
                            /* pos_setpoints = */ std::map< std::string, double >(),
                            /* vel_setpoints = */ setpoints);
  }

  virtual void stopping(const ros::Time &time) { controller_core_.stopping(); }

private:
  static void commandCB(const std_msgs::Float64ConstPtr &msg,
                        rt::RealtimeBuffer< double > *const buf) {
    buf->writeFromNonRT(msg->data);
  }

private:
  JointControllerCore controller_core_;
  ControlledJointInfoMap ctl_joints_;
};
} // namespace computed_torque_controllers

#endif