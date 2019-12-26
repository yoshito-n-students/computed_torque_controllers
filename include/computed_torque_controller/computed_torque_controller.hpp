#ifndef COMPUTED_TORQUE_CONTROLLER_COMPUTED_TORQUE_CONTROLLER_HPP
#define COMPUTED_TORQUE_CONTROLLER_COMPUTED_TORQUE_CONTROLLER_HPP

#include <memory>
#include <string>
#include <vector>

#include <computed_torque_controller/ros_package_resource_retriever.hpp>
#include <control_toolbox/pid.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/hardware_interface.h> // for HardwareInterfaceException
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/time.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <dart/dynamics/Skeleton.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

#include <boost/foreach.hpp>
#include <boost/optional.hpp>

namespace computed_torque_controller {

class ComputedTorqueController
    : public controller_interface::MultiInterfaceController<
          hardware_interface::JointStateInterface, hardware_interface::EffortJointInterface > {
private:
  struct JointInfo {
    // required info for both observed/controlled joints
    dart::dynamics::Joint *model_joint;
    hardware_interface::JointStateHandle joint_state_handle;

    // optional info for controlled joints
    boost::optional< control_toolbox::Pid > pid;
    boost::optional< hardware_interface::JointHandle > joint_command_handle;
  };

public:
  ComputedTorqueController() {}

  virtual ~ComputedTorqueController() {}

  // required interfaces as a Controller

  virtual bool init(hardware_interface::RobotHW *hw, ros::NodeHandle &root_nh,
                    ros::NodeHandle &controller_nh) {
    namespace ct = control_toolbox;
    namespace dc = dart::common;
    namespace dd = dart::dynamics;
    namespace du = dart::utils;
    namespace hi = hardware_interface;
    namespace rn = ros::names;
    namespace rp = ros::param;

    // get required hardware interfaces
    // (no need to check existence of interfaces because the base class did it)
    hi::JointStateInterface *const joint_state_iface(hw->get< hi::JointStateInterface >());
    hi::EffortJointInterface *const joint_command_iface(hw->get< hi::EffortJointInterface >());

    // build a dynamics model from robot_description param
    {
      std::string urdf_str;
      if (!controller_nh.getParam("robot_description", urdf_str) &&
          !rp::get("robot_description", urdf_str)) {
        ROS_ERROR_STREAM("ComputedTorqueController::init(): Faild to get robot description from '"
                         << controller_nh.resolveName("robot_description") << "' nor '"
                         << rn::resolve("robot_description") << "'");
        return false;
      }
      model_ = du::DartLoader().parseSkeletonString(
          urdf_str,
          // base URI to resolve relative URIs in the URDF.
          // this won't be used because almost all URIs are absolute.
          dc::Uri("file:/"), std::make_shared< ROSPackageResourceRetriever >());
      if (!model_) {
        ROS_ERROR("ComputedTorqueController::init(): Failed to build a dynamics model from "
                  "robot description");
        return false;
      }
    }

    // compose joints information
    BOOST_FOREACH (dd::Joint *const model_joint, model_->getJoints()) {
      JointInfo joint_info;
      // get the joint in the dynamics model
      joint_info.model_joint = model_joint;
      // joint state handle
      const std::string joint_name(joint_info.model_joint->getName());
      try {
        joint_info.joint_state_handle = joint_state_iface->getHandle(joint_name);
      } catch (const hi::HardwareInterfaceException &ex) {
        ROS_ERROR_STREAM("ComputedTorqueController::init(): Failed to get the state handle of '"
                         << joint_name << "': " << ex.what());
        return false;
      }
      // optional info for a controlled joint
      const std::string joint_ns(controller_nh.resolveName(rn::append("joints", joint_name)));
      if (rp::has(joint_ns)) {
        // pid
        joint_info.pid = ct::Pid();
        if (!joint_info.pid->initParam(joint_ns)) {
          ROS_ERROR_STREAM("ComputedTorqueController::init(): Failed to init a PID by param '"
                           << joint_ns << "'");
          return false;
        }
        // joint command handle
        try {
          joint_info.joint_command_handle = joint_command_iface->getHandle(joint_name);
        } catch (const hi::HardwareInterfaceException &ex) {
          ROS_ERROR_STREAM("ComputedTorqueController::init(): Failed to get the command handle of '"
                           << joint_name << "': " << ex.what());
          return false;
        }
      }
      // store joint info
      joints_.push_back(joint_info);
    }

    return true;
  }

  virtual void starting(const ros::Time &time) {
    // reset PID states
    BOOST_FOREACH (JointInfo &joint, joints_) {
      if (joint.pid) {
        joint.pid->reset();
      }
    }
  }

  virtual void update(const ros::Time &time, const ros::Duration &period) {
    // TODO: zero the root joint pose

    //
    Eigen::VectorXd u(joints_.size());
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      JointInfo &joint(joints_[i]);
      // update dynamics model by hardware states
      joint.model_joint->setPosition(0, joint.joint_state_handle.getPosition());
      joint.model_joint->setVelocity(0, joint.joint_state_handle.getVelocity());
      // generate control input
      if (joint.pid) {
        // TODO: get desired position from command topics
        u(i) = joint.pid->computeCommand(0. - joint.joint_state_handle.getPosition(), period);
      } else {
        u(i) = 0.;
      }
    }

    // compute required torque
    const Eigen::VectorXd t(model_->getMassMatrix() * u + model_->getCoriolisAndGravityForces());

    // set torque commands
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      if (joints_[i].joint_command_handle) {
        joints_[i].joint_command_handle->setCommand(t(i));
      }
    }
  }

  virtual void stopping(const ros::Time &time) {
    // nothing to do
    // (or set torque commands with zero control input??)
  }

private:
  dart::dynamics::SkeletonPtr model_;
  std::vector< JointInfo > joints_;
};
} // namespace computed_torque_controller

#endif