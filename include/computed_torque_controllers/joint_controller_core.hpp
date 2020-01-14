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

// ===========================================================================================
// core control implementation without command subscription
//   [ input] position and velocity setpoints of each controlled joint & states of all joints
//   [output] effort commands to controlled joints
class JointControllerCore {
protected:
  struct ObservedHardwareJoint {
    virtual ~ObservedHardwareJoint() {}

    // mapping to dynamics model
    std::size_t id_in_model;
    // state from the hardware
    hi::JointStateHandle state_handle;
  };
  typedef boost::shared_ptr< ObservedHardwareJoint > ObservedHardwareJointPtr;
  typedef std::map< std::string, ObservedHardwareJointPtr > ObservedHardwareJointMap;

  struct ControlledHardwareJoint : public ObservedHardwareJoint {
    // effort command (output) to the hardware
    hi::JointHandle eff_cmd_handle;
    // PID controller to generate effort command based on tracking error
    ct::Pid pid;
  };
  typedef boost::shared_ptr< ControlledHardwareJoint > ControlledHardwareJointPtr;
  typedef std::map< std::string, ControlledHardwareJointPtr > ControlledHardwareJointMap;

public:
  JointControllerCore() {}

  virtual ~JointControllerCore() {}

  // ============================================================================================
  bool init(const std::string &urdf_str, hi::RobotHW *const hw,
            const std::map< /* controlled joint name */ std::string,
                            /* joint-specific param ns */ std::string > &ctl_joint_namespaces) {
    return initModel(urdf_str) && initHardware(hw, ctl_joint_namespaces);
  }

  // =====================================================================================
  void starting(const ros::Time &time) {
    BOOST_FOREACH (const ControlledHardwareJointMap::value_type &joint, ctl_hw_joints_) {
      joint.second->pid.reset();
    }
  }

  // =======================================================================
  void update(const ros::Time &time, const ros::Duration &period,
              const std::map< std::string, PosVel > &ctl_joint_setpoints) {
    updateModel(period);
    updateHardware(period, ctl_joint_setpoints);
  }

  // ====================================
  void stopping(const ros::Time &time) {
    // nothing to do
    // (or set effort commands with zero control input??)
  }

protected:
  // =======================================================================
  // initialize a robot dynamics model based on robot description in URDF
  bool initModel(const std::string &urdf_str) {
    model_ = du::DartLoader().parseSkeletonString(
        urdf_str,
        // base URI to resolve relative URIs in the URDF.
        // this won't be used because almost all URIs are absolute.
        dc::Uri("file:/"), std::make_shared< ROSPackageResourceRetriever >());
    if (!model_) {
      ROS_ERROR("JointControllerCore::initModel(): Failed to build a dynamics model from "
                "robot description");
      return false;
    }

    // root joint between root link and world, which we need to access to update the model
    model_root_joint_ = dynamic_cast< dd::FreeJoint * >(model_->getRootJoint());
    if (!model_root_joint_) {
      ROS_ERROR(
          "JointControllerCore::initModel(): Faild to get a root joint of the dynamics model");
      return false;
    }

    return true;
  }

  // ======================================================================
  // initialize hardware joints. this cannot be called before initModel().
  bool initHardware(hi::RobotHW *const hw,
                    const std::map< std::string, std::string > &ctl_joint_namespaces) {
    // required hardware interfaces
    hi::JointStateInterface &state_iface(*hw->get< hi::JointStateInterface >());
    hi::EffortJointInterface &eff_cmd_iface(*hw->get< hi::EffortJointInterface >());

    // compose joint info by matching joints in the dynamics model & hardware
    BOOST_FOREACH (dd::Joint *const model_joint, model_->getJoints()) {
      // skip the model root joint that does not exist in the hardware
      if (model_joint == model_->getRootJoint()) {
        continue;
      }

      // assert single Dof model joint because hw joint only supports single Dof
      const std::string name(model_joint->getName());
      if (model_joint->getNumDofs() != 1) {
        ROS_ERROR_STREAM(
            "JointControllerCore::initHardware(): Multi-Dof joint is not supported (name: '"
            << name << "', dofs: " << model_joint->getNumDofs() << ")");
        return false;
      }

      // allocate joint info according to joint types (observed or observed-and-controlled)
      ObservedHardwareJointPtr joint;
      if (ctl_joint_namespaces.count(name) == 0) {
        joint.reset(new ObservedHardwareJoint());
      } else {
        joint.reset(new ControlledHardwareJoint());
      }

      // mapping to dynamics model
      joint->id_in_model = model_joint->getIndexInSkeleton(/* 1st DoF */ 0);

      // hardware joint state handle
      try {
        joint->state_handle = state_iface.getHandle(name);
      } catch (const hi::HardwareInterfaceException &ex) {
        ROS_ERROR_STREAM("JointControllerCore::initHardware(): Failed to get the state handle of '"
                         << name << "': " << ex.what());
        return false;
      }

      all_hw_joints_[name] = joint;

      // optional steps for a controlled joint
      if (ControlledHardwareJointPtr ctl_joint =
              boost::dynamic_pointer_cast< ControlledHardwareJoint >(joint)) {
        // effort command handle to the hardware
        try {
          ctl_joint->eff_cmd_handle = eff_cmd_iface.getHandle(name);
        } catch (const hi::HardwareInterfaceException &ex) {
          ROS_ERROR_STREAM(
              "JointControllerCore::initHardware(): Failed to get the effort command handle of '"
              << name << "': " << ex.what());
          return false;
        }

        // PID controller to generate effort command based on position error
        const std::string &ctl_joint_ns(ctl_joint_namespaces.find(name)->second);
        if (!ctl_joint->pid.initParam(ctl_joint_ns)) {
          ROS_ERROR_STREAM("JointControllerCore::initHardware(): Failed to init a PID by param '"
                           << ctl_joint_ns << "'");
          return false;
        }

        ctl_hw_joints_[name] = ctl_joint;
      }
    }

    return true;
  }

  // ==========================================================
  // update the dynamics model with the present hardware state
  void updateModel(const ros::Duration &period) {
    // update the root joint between the robot and world
    // (zero for now. TODO: get model state from args)
    model_root_joint_->setTransform(Eigen::Isometry3d::Identity());
    model_root_joint_->setLinearVelocity(Eigen::Vector3d::Zero());
    model_root_joint_->setAngularVelocity(Eigen::Vector3d::Zero());

    // update all joints in the robot
    BOOST_FOREACH (const ObservedHardwareJointMap::value_type &hw_joint_val, all_hw_joints_) {
      // short ailias
      const ObservedHardwareJoint &hw_joint(*hw_joint_val.second);
      const std::size_t id(hw_joint.id_in_model);
      const double hw_pos(hw_joint.state_handle.getPosition());
      const double hw_vel(hw_joint.state_handle.getVelocity());

      // use joint state forwarded by one time step for better control stability
      // (recommended in https://dartsim.github.io/tutorials_manipulator.html)
      model_->setPosition(id, hw_pos + hw_vel * period.toSec());
      model_->setVelocity(id, hw_vel);
    }
  }

  // ===================================================================
  // update the hardware joints by writing effort commands.
  // this expects the dynamics model has been updated by updateModel().
  void updateHardware(const ros::Duration &period,
                      const std::map< std::string, PosVel > &ctl_joint_setpoints) {
    // generate control input 'u' from tracking errors of controlled joints
    Eigen::VectorXd u(Eigen::VectorXd::Zero(model_->getNumDofs()));
    BOOST_FOREACH (const ControlledHardwareJointMap::value_type &joint_val, ctl_hw_joints_) {
      const std::string &name(joint_val.first);
      const std::map< std::string, PosVel >::const_iterator sp_it(ctl_joint_setpoints.find(name));
      if (sp_it == ctl_joint_setpoints.end()) {
        ROS_ERROR_STREAM(
            "JointControllerCore::updateHardware(): No setpoint for the controlled joint '" << name
                                                                                            << "'");
        continue;
      }

      // short ailias
      ControlledHardwareJoint &joint(*joint_val.second);
      const PosVel &sp(sp_it->second);

      // compute control input based on tracking errors
      u[joint.id_in_model] =
          joint.pid.computeCommand(sp.pos - joint.state_handle.getPosition(),
                                   sp.vel - joint.state_handle.getVelocity(), period);
    }

    // equations of motion
    const Eigen::MatrixXd &M(model_->getMassMatrix());
    const Eigen::VectorXd &Cg(model_->getCoriolisAndGravityForces());

    // set effort commands to the hardware
    BOOST_FOREACH (const ControlledHardwareJointMap::value_type &joint_val, ctl_hw_joints_) {
      ControlledHardwareJoint &joint(*joint_val.second);
      const std::size_t id(joint.id_in_model);
      joint.eff_cmd_handle.setCommand(M.row(id) * u + Cg[id]);
    }
  }

protected:
  dd::SkeletonPtr model_;
  dd::FreeJoint *model_root_joint_;
  ObservedHardwareJointMap all_hw_joints_;
  ControlledHardwareJointMap ctl_hw_joints_;
};

} // namespace computed_torque_controllers

#endif