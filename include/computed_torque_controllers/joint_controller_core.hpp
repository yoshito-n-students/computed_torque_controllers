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

namespace computed_torque_controllers {

// ================================================================
// convenient struct to represent setpoint for a 1-DoF joint
struct PosVelAcc {
  double pos, vel, acc;
};

// =========================================================
// core control implementation without command subscription
class JointControllerCore {
private:
  struct ControlledJointInfo {
    // mapping to dynamics model
    std::size_t id_in_model;
    // effort command (output) to the hardware
    hi::JointHandle hw_eff_cmd_handle;
    // PID controller to generate effort command based on position error
    ct::Pid pid;
  };
  typedef std::map< std::string, ControlledJointInfo > ControlledJointInfoMap;

  struct ObservedJointInfo {
    // mapping to dynamics model
    std::size_t id_in_model;
    // state from the hardware
    hi::JointStateHandle hw_state_handle;
    double prev_vel;
  };
  typedef std::map< std::string, ObservedJointInfo > ObservedJointInfoMap;

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

      // populate joint info
      const std::map< std::string, std::string >::const_iterator ctl_joint_ns(
          ctl_joint_namespaces.find(joint_name));
      if (ctl_joint_ns != ctl_joint_namespaces.end()) {
        // for a controlled joint
        ControlledJointInfo &ctl_joint_info(ctl_joints_[joint_name]);

        // mapping to dynamics model
        ctl_joint_info.id_in_model = model_joint->getIndexInSkeleton(/* 1st DoF */ 0);

        // effort command to the hardware
        try {
          ctl_joint_info.hw_eff_cmd_handle = hw_eff_cmd_iface.getHandle(joint_name);
        } catch (const hi::HardwareInterfaceException &ex) {
          ROS_ERROR_STREAM(
              "JointControllerCore::init(): Failed to get the effort command handle of '"
              << joint_name << "': " << ex.what());
          return false;
        }

        // PID controller to generate effort command based on position error
        if (!ctl_joint_info.pid.initParam(ctl_joint_ns->second)) {
          ROS_ERROR_STREAM("JointControllerCore::init(): Failed to init a PID by param '"
                           << ctl_joint_ns->second << "'");
          return false;
        }
      } else {
        // for an observed joint
        ObservedJointInfo &obs_joint_info(obs_joints_[joint_name]);

        // mapping to dynamics model
        obs_joint_info.id_in_model = model_joint->getIndexInSkeleton(/* 1st DoF */ 0);

        // info about hardware joint state handle
        try {
          obs_joint_info.hw_state_handle = hw_state_iface.getHandle(joint_name);
        } catch (const hi::HardwareInterfaceException &ex) {
          ROS_ERROR_STREAM("JointControllerCore::init(): Failed to get the state handle of '"
                           << joint_name << "': " << ex.what());
          return false;
        }
      }
    }

    return true;
  }

  void starting(const ros::Time &time) {
    BOOST_FOREACH (ControlledJointInfoMap::value_type &joint, ctl_joints_) {
      // reset PID controllers
      joint.second.pid.reset();
    }
    BOOST_FOREACH (ObservedJointInfoMap::value_type &joint, obs_joints_) {
      ObservedJointInfo &info(joint.second);
      info.prev_vel = info.hw_state_handle.getVelocity();
    }
  }

  void update(const ros::Time &time, const ros::Duration &period,
              const std::map< std::string, PosVelAcc > &ctl_joint_setpoints) {
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
    model_root_joint_->setLinearAcceleration(Eigen::Vector3d::Zero());
    model_root_joint_->setAngularAcceleration(Eigen::Vector3d::Zero());

    // update controlled joints in the model by the given setpoints
    Eigen::VectorXd u(Eigen::VectorXd::Zero(model_->getNumDofs()));
    BOOST_FOREACH (ControlledJointInfoMap::value_type &joint, ctl_joints_) {
      // short ailias
      const std::string &name(joint.first);
      ControlledJointInfo &info(joint.second);
      const std::size_t id(info.id_in_model);
      const PosVelAcc &sp(ctl_joint_setpoints.find(name)->second);

      // update model joints by given setpoints
      model_->setPosition(id, sp.pos);
      model_->setVelocity(id, sp.vel);
      model_->setAcceleration(id, sp.acc);

      // compute control input based on tracking errors
      u[id] = info.pid.computeCommand(sp.pos - info.hw_eff_cmd_handle.getPosition(),
                                      sp.vel - info.hw_eff_cmd_handle.getVelocity(), period);
    }

    // update observed joints in the model by the hardware state
    BOOST_FOREACH (ObservedJointInfoMap::value_type &joint, obs_joints_) {
      // short ailias
      ObservedJointInfo &info(joint.second);
      const std::size_t id(info.id_in_model);
      const double dt(period.toSec());
      const double pos(info.hw_state_handle.getPosition());
      const double vel(info.hw_state_handle.getVelocity());
      const double acc(dt > 0. ? (vel - info.prev_vel) / dt : 0.);

      // use joint state forwarded by one time step as setpoints
      model_->setPosition(id, pos + vel * dt);
      model_->setVelocity(id, vel);
      model_->setAcceleration(id, acc);
      info.prev_vel = vel;
    }

    // updated equations of motion
    const Eigen::MatrixXd &M(model_->getMassMatrix());
    const Eigen::VectorXd ddqu(model_->getAccelerations() + u);
    const Eigen::VectorXd &Cg(model_->getCoriolisAndGravityForces());

    // set effort commands to the hardware
    BOOST_FOREACH (ControlledJointInfoMap::value_type &joint, ctl_joints_) {
      ControlledJointInfo &info(joint.second);
      const std::size_t id(info.id_in_model);
      info.hw_eff_cmd_handle.setCommand(M.row(id) * ddqu + Cg[id]);
    }
  }

  void stopping(const ros::Time &time) {
    // nothing to do
    // (or set effort commands with zero control input??)
  }

private:
  dd::SkeletonPtr model_;
  dd::FreeJoint *model_root_joint_;
  std::map< std::string, ControlledJointInfo > ctl_joints_;
  std::map< std::string, ObservedJointInfo > obs_joints_;
};

} // namespace computed_torque_controllers

#endif