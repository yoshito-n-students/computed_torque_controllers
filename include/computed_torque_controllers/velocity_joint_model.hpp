#ifndef COMPUTED_TORQUE_CONTROLLERS_VELOCITY_JOINT_MODEL_HPP
#define COMPUTED_TORQUE_CONTROLLERS_VELOCITY_JOINT_MODEL_HPP

#include <map>
#include <string>

#include <computed_torque_controllers/common_namespaces.hpp>
#include <computed_torque_controllers/position_joint_model.hpp>
#include <ros/duration.h>
#include <ros/node_handle.h>

#include <Eigen/Core>

#include <boost/math/special_functions/fpclassify.hpp> // for boost::math::isnan()

namespace computed_torque_controllers {

// =======================================================
// dynamics model which calculates required joint efforts
// on the basis of desired joint velocities
class VelocityJointModel : protected PositionJointModel {
public:
  VelocityJointModel() {}

  virtual ~VelocityJointModel() {}

  // ========================================================
  // name-based interface for ros-controller implementations
  // ========================================================

  // ======================================================
  // initialize the model by params in the given namespace
  bool init(const ros::NodeHandle &param_nh) { return PositionJointModel::init(param_nh); }

  // ===============================================
  // reset the model states (not destroy the model)
  void reset() { PositionJointModel::reset(); }

  // ===============================================================
  // update model state based on given joint positions & velocities
  void update(const std::map< std::string, double > &positions,
              const std::map< std::string, double > &velocities, const ros::Duration &dt) {
    update(jointMapToEigen(positions), jointMapToEigen(velocities), dt);
  }

  // ================================================================================
  // compute joint effort commands to realize given vel setpoints based on the model
  std::map< std::string, double >
  computeEffortCommands(const std::map< std::string, double > &vel_setpoints,
                        const ros::Duration &dt) {
    return computeEffortCommands(jointMapToEigen(vel_setpoints), dt);
  }

protected:
  // ======================================================
  // index-based interface for child model implementations
  // ======================================================

  void update(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const ros::Duration &dt) {
    PositionJointModel::update(q, dq, dt);
  }

  std::map< std::string, double > computeEffortCommands(const Eigen::VectorXd &dqd,
                                                        const ros::Duration &dt) {
    // calc position setpoints by appending previous position setpoints to velocity setpoints.
    // if previous position setpoints are NaNs (e.g. the first control step),
    // use latest positions instead of position setpoints.
    Eigen::VectorXd qd(model_->getNumDofs());
    const Eigen::VectorXd prev_qd(getSetpointsEigen()), q(getPositionsEigen());
    for (std::size_t i = 0; i < model_->getNumDofs(); ++i) {
      qd[i] = (boost::math::isnan(prev_qd[i]) ? q[i] : prev_qd[i]) + dqd[i] * dt.toSec();
    }

    // calc effort commands based on the position setpoints
    return PositionJointModel::computeEffortCommands(qd, dt);
  }
};

} // namespace computed_torque_controllers

#endif