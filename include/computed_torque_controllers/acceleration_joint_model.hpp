#ifndef COMPUTED_TORQUE_CONTROLLERS_ACCELERATION_JOINT_MODEL_HPP
#define COMPUTED_TORQUE_CONTROLLERS_ACCELERATION_JOINT_MODEL_HPP

#include <map>
#include <memory>
#include <string>

#include <computed_torque_controllers/common_namespaces.hpp>
#include <computed_torque_controllers/ros_package_resource_retriever.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>

#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/Joint.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

#include <Eigen/Core>

#include <boost/foreach.hpp>

namespace computed_torque_controllers {

// ===========================================================================================
// core control implementation without command subscription
//   [ input] position and velocity setpoints of each controlled joint & states of all joints
//   [output] effort commands to controlled joints
class AccelerationJointModel {
public:
  AccelerationJointModel() {}

  virtual ~AccelerationJointModel() {}

  // =======================================================
  // name-based interface for ros-controller implementations
  // =======================================================

  // ==========================================
  bool init(const ros::NodeHandle &param_nh) {
    model_ = du::DartLoader().parseSkeletonString(
        getRobotDescription(param_nh),
        // base URI to resolve relative URIs in the URDF.
        // this won't be used because almost all URIs are absolute.
        dc::Uri("file:/"), std::make_shared< ROSPackageResourceRetriever >());
    if (!model_) {
      ROS_ERROR("AccelerationJointModel::init(): Failed to build a dynamics model from "
                "robot description");
      return false;
    }
    if (model_->getRootJoint()->getNumDofs() > 0) {
      ROS_ERROR("AccelerationJointModel::init(): Non-fixed root joint");
      return false;
    }
    BOOST_FOREACH (const dd::Joint *const joint, model_->getJoints()) {
      if (joint->getNumDofs() > 1) {
        ROS_ERROR_STREAM("AccelerationJointModel::init(): multi-DoF joint '" << joint->getName()
                                                                             << "'");
        return false;
      }
    }

    return true;
  }

  // ===============================================================
  // update model state based on given joint positions & velocities
  void update(const std::map< std::string, double > &positions,
              const std::map< std::string, double > &velocities, const ros::Duration &dt) {
    update(jointMapToEigen(positions), jointMapToEigen(velocities), dt);
  }

  // =================================================================================
  // compute joint effort commands to realize given acc setpoints based on the model
  std::map< std::string, double >
  computeEffortCommands(const std::map< std::string, double > &acc_setpoints) const {
    return computeEffortCommands(jointMapToEigen(acc_setpoints));
  }

protected:
  // =====================================================
  // index-based interface for child model implementations
  // =====================================================

  void update(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const ros::Duration &dt) {
    // update all DoFs in the robot
    for (std::size_t i = 0; i < model_->getNumDofs(); ++i) {
      // use joint state forwarded by one time step for better control stability
      // (recommended in https://dartsim.github.io/tutorials_manipulator.html)
      dd::DegreeOfFreedom *const dof(model_->getDof(i));
      dof->setPosition(q[i] + dq[i] * dt.toSec());
      dof->setVelocity(dq[i]);
    }
  }

  std::map< std::string, double > computeEffortCommands(const Eigen::VectorXd &u) const {
    return eigenToJointMap(model_->getMassMatrix() * u + model_->getCoriolisAndGravityForces());
  }

  // ===============================
  // conversion between name & index
  // ===============================

  Eigen::VectorXd jointMapToEigen(const std::map< std::string, double > &m) const {
    Eigen::VectorXd e(model_->getNumDofs());
    for (std::size_t i = 0; i < model_->getNumDofs(); ++i) {
      e[i] = findValue(m, model_->getDof(i)->getJoint()->getName());
    }
    return e;
  }

  std::map< std::string, double > eigenToJointMap(const Eigen::VectorXd &e) const {
    ROS_ASSERT_MSG(e.size() == model_->getNumDofs(),
                   "AccelerationJointMap::eigenToJointMap(): Invalid vector size");
                   
    std::map< std::string, double > m;
    for (std::size_t i = 0; i < model_->getNumDofs(); ++i) {
      m[model_->getDof(i)->getJoint()->getName()] = e[i];
    }
    return m;
  }

protected:
  dd::SkeletonPtr model_;
};

} // namespace computed_torque_controllers

#endif