#ifndef COMPUTED_TORQUE_CONTROLLERS_POSITION_JOINT_CONTROLLER_HPP
#define COMPUTED_TORQUE_CONTROLLERS_POSITION_JOINT_CONTROLLER_HPP

#include <limits>
#include <map>
#include <string>

#include <computed_torque_controllers/common_namespaces.hpp>
#include <computed_torque_controllers/joint_controller_core.hpp>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/hardware_interface.h> // for HardwareInterfaceException
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <urdf/model.h>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

namespace computed_torque_controllers {

// controller frontend subscribing joint position commands & passing them to backend
class PositionJointController
    : public ci::MultiInterfaceController< hi::JointStateInterface, hi::EffortJointInterface > {
private:
  struct ControlledJointInfo {
    // joint state handle from the hardware
    hi::JointStateHandle hw_state_handle;

    // position command subscription
    double pos_cmd;
    // in kinetic, rt::RealtimeBuffer<> does not have a const copy constructor
    //   --> ControlledJointInfo cannot have a const copy constructor
    //   --> BOOST_FOREACH(ControlledJointInfo::value_type &v, ctl_joints) does not compile
    // boost::shared_ptr<> is workaround to enable a const copy constructor
    boost::shared_ptr< rt::RealtimeBuffer< double > > pos_cmd_buf;
    ros::Subscriber pos_cmd_sub;
    boost::optional< jli::PositionJointSaturationHandle > pos_cmd_sat_handle;

    // previous setpoints
    double prev_pos_sp, prev_vel_sp;
  };
  typedef std::map< std::string, ControlledJointInfo > ControlledJointInfoMap;

public:
  PositionJointController() {}

  virtual ~PositionJointController() {}

  // required interfaces as a Controller

  virtual bool init(hi::RobotHW *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
    namespace rn = ros::names;
    namespace rp = ros::param;

    // get robot description in URDF from param
    std::string urdf_str;
    if (!controller_nh.getParam("robot_description", urdf_str) &&
        !rp::get("robot_description", urdf_str)) {
      ROS_ERROR_STREAM("PositionJointController::init(): Faild to get robot description from '"
                       << controller_nh.resolveName("robot_description") << "' nor '"
                       << rn::resolve("robot_description") << "'");
      return false;
    }

    // parse robot description to extract joint names & limits
    urdf::Model robot_desc;
    if (!robot_desc.initString(urdf_str)) {
      ROS_ERROR("PositionJointController::init(): Failed to parse robot description as an URDF");
      return false;
    }

    // populate controlled joint names & their param namespace
    typedef std::map< std::string, std::string > JointNamespaceMap;
    JointNamespaceMap ctl_joint_namespaces;
    typedef std::map< std::string, urdf::JointSharedPtr > JointDescMap;
    BOOST_FOREACH (const JointDescMap::value_type &joint_desc, robot_desc.joints_) {
      const std::string &name(joint_desc.first);
      const std::string ns(controller_nh.resolveName(rn::append("joints", name)));
      if (rp::has(ns)) {
        ctl_joint_namespaces[name] = ns;
      }
    }
    if (ctl_joint_namespaces.empty()) {
      ROS_ERROR_STREAM("PositionJointController::init(): No controlled joint loaded from param '"
                       << controller_nh.resolveName("joints") << "'");
      return false;
    }

    // init the controller backend
    if (!controller_core_.init(urdf_str, hw, ctl_joint_namespaces)) {
      ROS_ERROR("PositionJointController::init(): Failed to init controller backend");
      return false;
    }

    // compose info to subscribe & saturate joint position commands
    hi::JointStateInterface &hw_state_iface(*hw->get< hi::JointStateInterface >());
    BOOST_FOREACH (const JointNamespaceMap::value_type &ctl_joint_ns, ctl_joint_namespaces) {
      const std::string &name(ctl_joint_ns.first);
      ControlledJointInfo &info(ctl_joints_[name]);

      try {
        info.hw_state_handle = hw_state_iface.getHandle(name);
      } catch (const hi::HardwareInterfaceException &ex) {
        ROS_ERROR_STREAM("PositionJointController::init(): Failed to get the state handle of '"
                         << name << "': " << ex.what());
        return false;
      }

      // command subscription
      info.pos_cmd_buf.reset(new rt::RealtimeBuffer< double >());
      info.pos_cmd_sub = controller_nh.subscribe< std_msgs::Float64 >(
          rn::append(name, "command"), 1,
          boost::bind(&PositionJointController::positionCommandCB, _1, info.pos_cmd_buf));

      // satulation based on limits (optional)
      jli::JointLimits limits;
      if (jli::getJointLimits(robot_desc.getJoint(name), limits)) {
        const hi::JointHandle pos_cmd_handle(info.hw_state_handle, &info.pos_cmd);
        info.pos_cmd_sat_handle = jli::PositionJointSaturationHandle(pos_cmd_handle, limits);
      }
    }

    return true;
  }

  virtual void starting(const ros::Time &time) {
    BOOST_FOREACH (ControlledJointInfoMap::value_type &ctl_joint, ctl_joints_) {
      ControlledJointInfo &info(ctl_joint.second);
      // reset position command by present position
      info.pos_cmd_buf->writeFromNonRT(info.hw_state_handle.getPosition());
      // reset stateful command saturation handle
      if (info.pos_cmd_sat_handle) {
        info.pos_cmd_sat_handle->reset();
      }
      // reset setpoint memory by present state
      info.prev_pos_sp = info.hw_state_handle.getPosition();
      info.prev_vel_sp = 0.;
    }
  }

  virtual void update(const ros::Time &time, const ros::Duration &period) {
    // calc pos/vel/acc setpoints from subscribed position commands
    std::map< std::string, PosVelAcc > ctl_joint_setpoints;
    BOOST_FOREACH (ControlledJointInfoMap::value_type &joint, ctl_joints_) {
      const std::string &name(joint.first);
      ControlledJointInfo &info(joint.second);
      // copy position setpoint from the buffer
      info.pos_cmd = *info.pos_cmd_buf->readFromRT();
      // satualate the command. below may modify info.pos_cmd according to the joint limit
      if (info.pos_cmd_sat_handle) {
        info.pos_cmd_sat_handle->enforceLimits(period);
      }
      // setpoints as derivatives of (saturated) position command
      PosVelAcc &sp(ctl_joint_setpoints[name]);
      const double dt(period.toSec());
      sp.pos = info.pos_cmd;
      sp.vel = (dt > 0. ? (sp.pos - info.prev_pos_sp) / dt : 0.);
      sp.acc = (dt > 0. ? (sp.vel - info.prev_vel_sp) / dt : 0.);
      // remember this setpoint for the next control step
      info.prev_pos_sp = sp.pos;
      info.prev_vel_sp = sp.vel;
    }

    // update the backend
    controller_core_.update(time, period, ctl_joint_setpoints);
  }

  virtual void stopping(const ros::Time &time) {
    // nothing to do
    // (or set effort commands with zero control input??)
  }

private:
  static void positionCommandCB(const std_msgs::Float64ConstPtr &msg,
                                const boost::shared_ptr< rt::RealtimeBuffer< double > > &buf) {
    buf->writeFromNonRT(msg->data);
  }

private:
  JointControllerCore controller_core_;
  ControlledJointInfoMap ctl_joints_;
};
} // namespace computed_torque_controllers

#endif