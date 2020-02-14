#ifndef COMPUTED_TORQUE_CONTROLLERS_VELOCITY_JOINT_CONTROLLER_HPP
#define COMPUTED_TORQUE_CONTROLLERS_VELOCITY_JOINT_CONTROLLER_HPP

#include <map>
#include <string>

#include <computed_torque_controllers/common_namespaces.hpp>
#include <computed_torque_controllers/effort_joint_hardware.hpp>
#include <computed_torque_controllers/velocity_joint_model.hpp>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
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
class VelocityJointController : public ci::Controller< hi::EffortJointInterface > {
private:
  struct RealtimeSubscriber {
    // in kinetic, rt::RealtimeBuffer<> does not have a const copy constructor
    //   --> RealtimeSubscriber cannot have a const copy constructor
    //   --> BOOST_FOREACH(RealtimeSubscriberMap::value_type &v, subscribers_) does not compile
    // boost::optional<> is workaround to enable a const copy constructor.
    // actually, buf is always available.
    boost::optional< rt::RealtimeBuffer< double > > buf;
    ros::Subscriber sub;
  };
  typedef std::map< std::string, RealtimeSubscriber > RealtimeSubscriberMap;

public:
  VelocityJointController() {}

  virtual ~VelocityJointController() {}

  // required interfaces as a Controller

  virtual bool init(hi::EffortJointInterface *hw, ros::NodeHandle &controller_nh) {
    // init the controller backend
    if (!eff_jnt_hw_.init(hw, controller_nh)) {
      ROS_ERROR("VelocityJointController::init(): Failed to init the joint hardware");
      return false;
    }
    if (!vel_jnt_model_.init(controller_nh)) {
      ROS_ERROR("VelocityJointController::init(): Failed to init the joint model");
      return false;
    }

    // subscribe commands for controlled joints
    BOOST_FOREACH (const std::string &name, eff_jnt_hw_.getJointNames()) {
      RealtimeSubscriber &sub(subscribers_[name]);
      sub.buf.emplace();
      sub.sub = controller_nh.subscribe< std_msgs::Float64 >(
          ros::names::append(name, "command"), 1,
          boost::bind(&VelocityJointController::commandCB, _1, sub.buf.get_ptr()));
    }

    return true;
  }

  virtual void starting(const ros::Time &time) {
    vel_jnt_model_.reset();

    BOOST_FOREACH (RealtimeSubscriberMap::value_type &sub_val, subscribers_) {
      RealtimeSubscriber &sub(sub_val.second);
      // reset velocity command
      sub.buf->writeFromNonRT(0.);
    }
  }

  virtual void update(const ros::Time &time, const ros::Duration &period) {
    //
    vel_jnt_model_.update(eff_jnt_hw_.getPositions(), eff_jnt_hw_.getVelocities(), period);

    // populate setpoints from subscribed velocity commands
    std::map< std::string, double > vel_setpoints;
    BOOST_FOREACH (RealtimeSubscriberMap::value_type &sub_val, subscribers_) {
      const std::string &name(sub_val.first);
      RealtimeSubscriber &sub(sub_val.second);
      // copy velocity setpoint from the buffer
      vel_setpoints[name] = *sub.buf->readFromRT();
    }

    // update the backend
    const std::map< std::string, double > eff_commands(
        vel_jnt_model_.computeEffortCommands(vel_setpoints, period));

    //
    eff_jnt_hw_.setCommands(eff_commands);
  }

  virtual void stopping(const ros::Time &time) {}

private:
  static void commandCB(const std_msgs::Float64ConstPtr &msg,
                        rt::RealtimeBuffer< double > *const buf) {
    buf->writeFromNonRT(msg->data);
  }

private:
  EffortJointHardware eff_jnt_hw_;
  VelocityJointModel vel_jnt_model_;
  RealtimeSubscriberMap subscribers_;
};
} // namespace computed_torque_controllers

#endif