#ifndef COMPUTED_TORQUE_CONTROLLERS_ACCELERATION_JOINT_MODEL_HPP
#define COMPUTED_TORQUE_CONTROLLERS_ACCELERATION_JOINT_MODEL_HPP

#include <map>
#include <memory>
#include <string>

#include <computed_torque_controllers/common_namespaces.hpp>
#include <computed_torque_controllers/ros_package_resource_retriever.hpp>
#include <ros/assert.h>
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

// =======================================================
// dynamics model which calculates required joint efforts
// on the basis of desired joint accelerations
class AccelerationJointModel {
public:
  AccelerationJointModel() {}

  virtual ~AccelerationJointModel() {}

  // ========================================================
  // name-based interface for ros-controller implementations
  // ========================================================

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

    // make sure the root joint between the world and model is fixed
    // & all other joints are fixed or single-DoF.
    // this ensures all DoFs in the model belong to different joints
    // so that we can convert DoF indices & joint names.
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
  // ======================================================
  // index-based interface for child model implementations
  // ======================================================

  void update(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const ros::Duration &dt) {
    // update all actuated joints in the model
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

  // ================================
  // conversion between name & index
  // ================================

  const std::string &jointIdToName(const std::size_t id) const {
    return model_->getDof(id)->getJoint()->getName();
  }

  Eigen::VectorXd jointMapToEigen(const std::map< std::string, double > &m) const {
    Eigen::VectorXd e(model_->getNumDofs());
    for (std::size_t i = 0; i < model_->getNumDofs(); ++i) {
      e[i] = findValue(m, jointIdToName(i));
    }
    return e;
  }

  std::map< std::string, double > eigenToJointMap(const Eigen::VectorXd &e) const {
    ROS_ASSERT_MSG(e.size() == model_->getNumDofs(),
                   "AccelerationJointMap::eigenToJointMap(): Invalid vector size");

    std::map< std::string, double > m;
    for (std::size_t i = 0; i < model_->getNumDofs(); ++i) {
      m[jointIdToName(i)] = e[i];
    }
    return m;
  }

protected:
  dd::SkeletonPtr model_;
};

} // namespace computed_torque_controllers

#endif