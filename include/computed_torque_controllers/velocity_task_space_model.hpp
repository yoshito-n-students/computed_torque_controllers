#ifndef COMPUTED_TORQUE_CONTROLLERS_VELOCITY_TASK_SPACE_MODEL_HPP
#define COMPUTED_TORQUE_CONTROLLERS_VELOCITY_TASK_SPACE_MODEL_HPP

#include <map>
#include <string>

#include <computed_torque_controllers/common_namespaces.hpp>
#include <computed_torque_controllers/position_task_space_model.hpp>
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
class VelocityTaskSpaceModel : protected PositionTaskSpaceModel {
public:
  VelocityTaskSpaceModel() {}

  virtual ~VelocityTaskSpaceModel() {}

  // ==========================================
  bool init(const ros::NodeHandle &param_nh) { return PositionTaskSpaceModel::init(param_nh); }

  // ============
  void reset() { PositionTaskSpaceModel::reset(); }

  // ===================================================================
  void update(const std::map< std::string, double > &jnt_positions,
              const std::map< std::string, double > &jnt_velocities, const ros::Duration &dt) {
    update(jointMapToEigen(jnt_positions), jointMapToEigen(jnt_velocities), dt);
  }

  // ========================================================================
  std::map< std::string, double >
  computeEffortCommands(const std::map< std::string, double > &vel_setpoints,
                        const ros::Duration &dt) {
    computeEffortCommands(dofMapToEigen(vel_setpoints), dt);
  }

protected:
  void update(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const ros::Duration &dt) {
    PositionTaskSpaceModel::update(q, dq, dt);
  }

  std::map< std::string, double > computeEffortCommands(const Eigen::VectorXd &dxd,
                                                        const ros::Duration &dt) {
    // calc position setpoints by appending previous position setpoints to velocity setpoints.
    // previous position setpoints may be NaNs on the first control step.
    Eigen::Vector6d xd;
    const Eigen::Vector6d prev_xd(getSetpointsEigen()), x(getPositionsEigen());
    for (std::size_t i = 0; i < 6; ++i) {
      xd[i] = (boost::math::isnan(prev_xd[i]) ? x[i] : prev_xd[i]) + dxd[i] * dt.toSec();
      // TODO: fix angular accumuration
    }

    return PositionTaskSpaceModel::computeEffortCommands(xd, dt);
  }
};

} // namespace computed_torque_controllers

#endif