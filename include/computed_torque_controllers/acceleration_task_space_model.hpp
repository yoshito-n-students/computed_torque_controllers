#ifndef COMPUTED_TORQUE_CONTROLLERS_ACCELERATION_TASK_SPACE_MODEL_HPP
#define COMPUTED_TORQUE_CONTROLLERS_ACCELERATION_TASK_SPACE_MODEL_HPP

#include <map>
#include <string>

#include <computed_torque_controllers/acceleration_joint_model.hpp>
#include <computed_torque_controllers/common_namespaces.hpp>
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <urdf/model.h>

#include <dart/dynamics/Joint.hpp>

#include <boost/foreach.hpp>
#include <boost/optional.hpp>

namespace computed_torque_controllers {

// ===========================================================================================
// core control implementation without command subscription
//   [ input] position and velocity setpoints of each controlled joint & states of all joints
//   [output] effort commands to controlled joints
class AccelerationTaskSpaceModel : protected AccelerationJointModel {
public:
  AccelerationTaskSpaceModel() {}

  virtual ~AccelerationTaskSpaceModel() {}

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

  // =====================================================================
  void update(const std::map< std::string, double > &jnt_positions,
              const std::map< std::string, double > &jnt_velocities, const ros::Duration &dt) {
    update(jointMapToEigen(jnt_positions), jointMapToEigen(jnt_velocities), dt);
  }

  // ========================================================================
  std::map< std::string, double >
  computeEffortCommands(const std::map< std::string, double > &acc_setpoints) const {
    return computeEffortCommands(dofMapToEigen(acc_setpoints));
  }

protected:
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

  static Eigen::Vector6d dofMapToEigen(const std::map< std::string, double > &m) {
    Eigen::Vector6d e;
    e[0] = findValue(m, "angular_x");
    e[1] = findValue(m, "angular_y");
    e[2] = findValue(m, "angular_z");
    e[3] = findValue(m, "linear_x");
    e[4] = findValue(m, "linear_y");
    e[5] = findValue(m, "linear_z");
    return e;
  }

  static std::map< std::string, double > eigenToDofMap(const Eigen::Vector6d &e) {
    std::map< std::string, double > m;
    m["angular_x"] = e[0];
    m["angular_y"] = e[1];
    m["angular_z"] = e[2];
    m["linear_x"] = e[3];
    m["linear_y"] = e[4];
    m["linear_z"] = e[5];
    return m;
  }

protected:
  dd::BodyNodePtr model_end_link_;
  Eigen::Vector3d end_link_offset_;
};

} // namespace computed_torque_controllers

#endif