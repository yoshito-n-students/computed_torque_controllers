#ifndef COMPUTED_TORQUE_CONTROLLERS_VELOCITY_JOINT_MODEL_HPP
#define COMPUTED_TORQUE_CONTROLLERS_VELOCITY_JOINT_MODEL_HPP

#include <map>
#include <string>

#include <computed_torque_controllers/common_namespaces.hpp>
#include <computed_torque_controllers/position_joint_model.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>

#include <boost/foreach.hpp>
#include <boost/math/special_functions/fpclassify.hpp> // for boost::math::isnan()

namespace computed_torque_controllers {

// ===========================================================================================
// core control implementation without command subscription
//   [ input] position and velocity setpoints of each controlled joint & states of all joints
//   [output] effort commands to controlled joints
class VelocityJointModel : protected PositionJointModel {
public:
  VelocityJointModel() {}

  virtual ~VelocityJointModel() {}

  // ==========================================
  bool init(const ros::NodeHandle &param_nh) { return PositionJointModel::init(param_nh); }

  // ============
  void reset() { PositionJointModel::reset(); }

  // ===================================================================
  void update(const std::map< std::string, double > &positions,
              const std::map< std::string, double > &velocities, const ros::Duration &dt) {
    update(jointMapToEigen(positions), jointMapToEigen(velocities), dt);
  }

  // ========================================================================
  std::map< std::string, double >
  computeEffortCommands(const std::map< std::string, double > &vel_setpoints,
                        const ros::Duration &dt) {
    computeEffortCommands(jointMapToEigen(vel_setpoints), dt);
  }

protected:
  void update(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const ros::Duration &dt) {
    PositionJointModel::update(q, dq, dt);
  }

  std::map< std::string, double > computeEffortCommands(const Eigen::VectorXd &dqd,
                                                        const ros::Duration &dt) {
    // calc position setpoints by appending previous position setpoints to velocity setpoints.
    // previous position setpoints may be NaNs on the first control step.
    Eigen::VectorXd qd(model_->getNumDofs());
    const Eigen::VectorXd prev_qd(getSetpointsEigen()), q(getPositionsEigen());
    for (std::size_t i = 0; i < model_->getNumDofs(); ++i) {
      qd[i] = (boost::math::isnan(prev_qd[i]) ? q[i] : prev_qd[i]) + dqd[i] * dt.toSec();
    }

    return PositionJointModel::computeEffortCommands(qd, dt);
  }
};

} // namespace computed_torque_controllers

#endif