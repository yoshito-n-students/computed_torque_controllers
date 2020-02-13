#ifndef COMPUTED_TORQUE_CONTROLLERS_EFFORT_JOINT_HARDWARE_HPP
#define COMPUTED_TORQUE_CONTROLLERS_EFFORT_JOINT_HARDWARE_HPP

#include <string>
#include <vector>

#include <computed_torque_controllers/common_namespaces.hpp>
#include <computed_torque_controllers/utils.hpp>
#include <hardware_interface/hardware_interface.h> // for hi::HardwareInterfaceException
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <urdf/model.h>

#include <boost/foreach.hpp>

namespace computed_torque_controllers {

class EffortJointHardware {
public:
  EffortJointHardware() {}

  virtual ~EffortJointHardware() {}

  bool init(hi::RobotHW *const hw, const ros::NodeHandle &param_nh) {
    urdf::Model robot_desc;
    if (!robot_desc.initString(getRobotDescription(param_nh))) {
      ROS_ERROR("EffortJointHardware::init(): Failed to parse robot description as an URDF");
      return false;
    }

    hi::EffortJointInterface *const eff_cmd_iface(hw->get< hi::EffortJointInterface >());
    if (!eff_cmd_iface) {
      ROS_ERROR("EffortJointHardware::init(): No effort joint interface in the hardware");
      return false;
    }

    typedef std::map< std::string, urdf::JointSharedPtr > JointDescMap;
    BOOST_FOREACH (const JointDescMap::value_type &joint_desc_val, robot_desc.joints_) {
      // skip a joint with no degrees of freedom
      const urdf::Joint &joint(*joint_desc_val.second);
      if (joint.type == urdf::Joint::UNKNOWN || joint.type == urdf::Joint::FIXED) {
        continue;
      }
      // effort command handle to the hardware
      const std::string &name(joint_desc_val.first);
      try {
        joints_.push_back(eff_cmd_iface->getHandle(name));
      } catch (const hi::HardwareInterfaceException &ex) {
        ROS_ERROR_STREAM(
            "EffortJointHardware::init(): Failed to get the effort command handle of the joint '"
            << name << "': " << ex.what());
        return false;
      }
    }

    return true;
  }

  std::vector< std::string > getJointNames() const {
    std::vector< std::string > names;
    BOOST_FOREACH (const hi::JointHandle &joint, joints_) { names.push_back(joint.getName()); }
    return names;
  }

  std::map< std::string, double > getPositions() const {
    std::map< std::string, double > positions;
    BOOST_FOREACH (const hi::JointHandle &joint, joints_) {
      positions[joint.getName()] = joint.getPosition();
    }
    return positions;
  }

  std::map< std::string, double > getVelocities() const {
    std::map< std::string, double > velocities;
    BOOST_FOREACH (const hi::JointHandle &joint, joints_) {
      velocities[joint.getName()] = joint.getVelocity();
    }
    return velocities;
  }

  void setCommands(const std::map< std::string, double > &eff_commands) {
    BOOST_FOREACH (hi::JointHandle &joint, joints_) {
      joint.setCommand(findValue(eff_commands, joint.getName()));
    }
  }

protected:
  std::vector< hi::JointHandle > joints_;
};
} // namespace computed_torque_controllers

#endif