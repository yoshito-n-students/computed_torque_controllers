#ifndef COMPUTED_TORQUE_CONTROLLERS_ACCELERATION_TASK_SPACE_MODEL_HPP
#define COMPUTED_TORQUE_CONTROLLERS_ACCELERATION_TASK_SPACE_MODEL_HPP

#include <map>
#include <string>

#include <computed_torque_controllers/acceleration_joint_model.hpp>
#include <computed_torque_controllers/common_namespaces.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>

#include <dart/dynamics/BodyNode.hpp>

#include <Eigen/Core>

namespace computed_torque_controllers {

// =======================================================
// dynamics model which calculates required joint efforts
// on the basis of desired end-effector acceleration
class AccelerationTaskSpaceModel : protected AccelerationJointModel {
public:
  AccelerationTaskSpaceModel() {}

  virtual ~AccelerationTaskSpaceModel() {}

  // ========================================================
  // name-based interface for ros-controller implementations
  // ========================================================

  // ==========================================
  bool init(const ros::NodeHandle &param_nh) {
    // init the base model first
    if (!AccelerationJointModel::init(param_nh)) {
      return false;
    }

    // TODO: end effector info from param
    model_end_link_ = model_->getBodyNode(model_->getNumBodyNodes() - 1);
    end_link_offset_ = Eigen::Vector3d::Zero();

    return true;
  }

  // ===============================================================
  // update model state based on given joint positions & velocities
  void update(const std::map< std::string, double > &jnt_positions,
              const std::map< std::string, double > &jnt_velocities, const ros::Duration &dt) {
    update(jointMapToEigen(jnt_positions), jointMapToEigen(jnt_velocities), dt);
  }

  // =============================================================================================
  // compute joint effort commands to realize given end-effector acc setpoints based on the model
  std::map< std::string, double >
  computeEffortCommands(const std::map< std::string, double > &acc_setpoints) const {
    return computeEffortCommands(dofMapToEigen(acc_setpoints));
  }

protected:
  // ======================================================
  // index-based interface for child model implementations
  // ======================================================

  void update(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const ros::Duration &dt) {
    AccelerationJointModel::update(q, dq, dt);
  }

  std::map< std::string, double > computeEffortCommands(const Eigen::Vector6d &ux) const {
    // convert acc setpoint vector from the task space to the joint space
    const dm::Jacobian J(model_end_link_->getWorldJacobian(end_link_offset_));
    const Eigen::MatrixXd pinv_J(
        J.transpose() * (J * J.transpose() + 0.0025 * Eigen::Matrix6d::Identity()).inverse());
    const dm::Jacobian dJ(model_end_link_->getJacobianClassicDeriv(end_link_offset_));
    const Eigen::VectorXd dq(model_->getVelocities());
    const Eigen::VectorXd uq(pinv_J * (ux - dJ * dq));

    //
    return AccelerationJointModel::computeEffortCommands(uq);
  }

  // ================================
  // conversion between name & index
  // ================================

  static std::string dofIdToName(const std::size_t id) {
    ROS_ASSERT_MSG(id >= 0 && id < 6, "AccelerationTaskSpaceModel::dofIdToName(): Invalid id (%d)",
                   static_cast< int >(id));

    switch (id) {
    case 0:
      return "angular_x";
    case 1:
      return "angular_y";
    case 2:
      return "angular_z";
    case 3:
      return "linear_x";
    case 4:
      return "linear_y";
    case 5:
      return "linear_z";
    }
  }

  static Eigen::Vector6d dofMapToEigen(const std::map< std::string, double > &m) {
    Eigen::Vector6d e;
    for (std::size_t i = 0; i < 6; ++i) {
      e[i] = findValue(m, dofIdToName(i));
    }
    return e;
  }

  static std::map< std::string, double > eigenToDofMap(const Eigen::Vector6d &e) {
    std::map< std::string, double > m;
    for (std::size_t i = 0; i < 6; ++i) {
      m[dofIdToName(i)] = e[i];
    }
    return m;
  }

protected:
  dd::BodyNodePtr model_end_link_;
  Eigen::Vector3d end_link_offset_;
};

} // namespace computed_torque_controllers

#endif