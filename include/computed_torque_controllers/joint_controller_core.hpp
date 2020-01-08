#ifndef COMPUTED_TORQUE_CONTROLLERS_JOINT_CONTROLLER_CORE_HPP
#define COMPUTED_TORQUE_CONTROLLERS_JOINT_CONTROLLER_CORE_HPP

#include <map>
#include <memory>
#include <string>

#include <computed_torque_controllers/common_namespaces.hpp>
#include <computed_torque_controllers/ros_package_resource_retriever.hpp>
#include <control_toolbox/pid.h>
#include <hardware_interface/hardware_interface.h> // for HardwareInterfaceException
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/Joint.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

#include <Eigen/Core>

#include <boost/foreach.hpp>
#include <boost/pointer_cast.hpp>
#include <boost/shared_ptr.hpp>

namespace computed_torque_controllers {

// ================================================================
// convenient struct to represent setpoint for a 1-DoF joint
struct PosVel {
  double pos, vel;
};

// =========================================================
// core control implementation without command subscription
class JointControllerCore {
private:
  struct ObservedJointInfo {
    virtual ~ObservedJointInfo() {}

    // mapping to dynamics model
    std::size_t id_in_model;
    // state from the hardware
    hi::JointStateHandle hw_state_handle;
  };
  typedef boost::shared_ptr< ObservedJointInfo > ObservedJointInfoPtr;
  typedef std::map< std::string, ObservedJointInfoPtr > ObservedJointInfoMap;

  struct ControlledJointInfo : public ObservedJointInfo {
    // effort command (output) to the hardware
    hi::JointHandle hw_eff_cmd_handle;
    // PID controller to generate effort command based on position error
    ct::Pid pid;
  };
  typedef boost::shared_ptr< ControlledJointInfo > ControlledJointInfoPtr;
  typedef std::map< std::string, ControlledJointInfoPtr > ControlledJointInfoMap;

public:
  JointControllerCore() {}

  virtual ~JointControllerCore() {}

  // required interfaces as a Controller

  bool init(const std::string &urdf_str, hi::RobotHW *const hw,
            const std::map< /* controlled joint name */ std::string,
                            /* joint-specific param ns */ std::string > &ctl_joint_namespaces) {
    // build a dynamics model based on URDF description
    model_ = du::DartLoader().parseSkeletonString(
        urdf_str,
        // base URI to resolve relative URIs in the URDF.
        // this won't be used because almost all URIs are absolute.
        dc::Uri("file:/"), std::make_shared< ROSPackageResourceRetriever >());
    if (!model_) {
      ROS_ERROR("JointControllerCore::init(): Failed to build a dynamics model from "
                "robot description");
      return false;
    }

    // get model root joint between root link and world, which does not exist in the hardware
    model_root_joint_ = dynamic_cast< dd::FreeJoint * >(model_->getRootJoint());
    if (!model_root_joint_) {
      ROS_ERROR("JointControllerCore::init(): Faild to get a root joint of the dynamics model");
      return false;
    }

    // get required hardware interfaces
    hi::JointStateInterface &hw_state_iface(*hw->get< hi::JointStateInterface >());
    hi::EffortJointInterface &hw_eff_cmd_iface(*hw->get< hi::EffortJointInterface >());

    // compose joint info by matching joints in the dynamics model & hardware
    BOOST_FOREACH (dd::Joint *const model_joint, model_->getJoints()) {
      // skip the model root joint already handled
      if (model_joint == model_->getRootJoint()) {
        continue;
      }

      // assert single Dof model joint because hw joint only supports single Dof
      const std::string joint_name(model_joint->getName());
      if (model_joint->getNumDofs() != 1) {
        ROS_ERROR_STREAM("JointControllerCore::init(): Multi-Dof joint is not supported (name: '"
                         << joint_name << "', dofs: " << model_joint->getNumDofs() << ")");
        return false;
      }

      // allocate joint info according to joint types (observed or observed-and-controlled)
      ObservedJointInfoPtr joint_info;
      if (ctl_joint_namespaces.count(joint_name) == 0) {
        joint_info.reset(new ObservedJointInfo());
      } else {
        joint_info.reset(new ControlledJointInfo());
      }

      // mapping to dynamics model
      joint_info->id_in_model = model_joint->getIndexInSkeleton(/* 1st DoF */ 0);

      // info about hardware joint state handle
      try {
        joint_info->hw_state_handle = hw_state_iface.getHandle(joint_name);
      } catch (const hi::HardwareInterfaceException &ex) {
        ROS_ERROR_STREAM("JointControllerCore::init(): Failed to get the state handle of '"
                         << joint_name << "': " << ex.what());
        return false;
      }

      // save info as an observed joint
      all_joints_[joint_name] = joint_info;

      // optional steps for a controlled joint
      if (ControlledJointInfoPtr ctl_joint_info =
              boost::dynamic_pointer_cast< ControlledJointInfo >(joint_info)) {
        // effort command to the hardware
        try {
          ctl_joint_info->hw_eff_cmd_handle = hw_eff_cmd_iface.getHandle(joint_name);
        } catch (const hi::HardwareInterfaceException &ex) {
          ROS_ERROR_STREAM(
              "JointControllerCore::init(): Failed to get the effort command handle of '"
              << joint_name << "': " << ex.what());
          return false;
        }

        // PID controller to generate effort command based on position error
        const std::string &ctl_joint_ns(ctl_joint_namespaces.find(joint_name)->second);
        if (!ctl_joint_info->pid.initParam(ctl_joint_ns)) {
          ROS_ERROR_STREAM("JointControllerCore::init(): Failed to init a PID by param '"
                           << ctl_joint_ns << "'");
          return false;
        }

        // save info as a controlled joint
        ctl_joints_[joint_name] = ctl_joint_info;
      }
    }

    return true;
  }

  void starting(const ros::Time &time) {
    BOOST_FOREACH (const ControlledJointInfoMap::value_type &joint, ctl_joints_) {
      // reset PID controllers
      joint.second->pid.reset();
    }
  }

  void update(const ros::Time &time, const ros::Duration &period,
              const std::map< std::string, PosVel > &ctl_joint_setpoints) {
    // match controlled joint names
    BOOST_FOREACH (const ControlledJointInfoMap::value_type &joint, ctl_joints_) {
      const std::string &name(joint.first);
      if (ctl_joint_setpoints.count(name) == 0) {
        ROS_ERROR_STREAM(
            "JointControllerCore::update(): Missing setpoint for the controlled joint '" << name
                                                                                         << "'");
        return;
      }
    }

    // update state of the model
    // (zero for now. TODO: get model state from args)
    model_root_joint_->setTransform(Eigen::Isometry3d::Identity());
    model_root_joint_->setLinearVelocity(Eigen::Vector3d::Zero());
    model_root_joint_->setAngularVelocity(Eigen::Vector3d::Zero());

    // update all joints in the model by the hardware state
    BOOST_FOREACH (const ObservedJointInfoMap::value_type &joint, all_joints_) {
      // short ailias
      ObservedJointInfo &info(*joint.second);
      const std::size_t id(info.id_in_model);
      const double pos(info.hw_state_handle.getPosition());
      const double vel(info.hw_state_handle.getVelocity());

      // use joint state forwarded by one time step as setpoints
      model_->setPosition(id, pos + vel * period.toSec());
      model_->setVelocity(id, vel);
    }

    // generate control input 'u' from tracking errors of controlled joints
    Eigen::VectorXd u(Eigen::VectorXd::Zero(model_->getNumDofs()));
    BOOST_FOREACH (const ControlledJointInfoMap::value_type &joint, ctl_joints_) {
      // short ailias
      ControlledJointInfo &info(*joint.second);
      const PosVel &sp(ctl_joint_setpoints.find(joint.first)->second);

      // compute control input based on tracking errors
      u[info.id_in_model] =
          info.pid.computeCommand(sp.pos - info.hw_eff_cmd_handle.getPosition(),
                                  sp.vel - info.hw_eff_cmd_handle.getVelocity(), period);
    }

    // updated equations of motion
    const Eigen::MatrixXd &M(model_->getMassMatrix());
    const Eigen::VectorXd &Cg(model_->getCoriolisAndGravityForces());

    // set effort commands to the hardware
    BOOST_FOREACH (const ControlledJointInfoMap::value_type &joint, ctl_joints_) {
      ControlledJointInfo &info(*joint.second);
      const std::size_t id(info.id_in_model);
      info.hw_eff_cmd_handle.setCommand(M.row(id) * u + Cg[id]);
    }
  }

  void stopping(const ros::Time &time) {
    // nothing to do
    // (or set effort commands with zero control input??)
  }

private:
  dd::SkeletonPtr model_;
  dd::FreeJoint *model_root_joint_;
  ObservedJointInfoMap all_joints_;
  ControlledJointInfoMap ctl_joints_;
};

} // namespace computed_torque_controllers

#endif