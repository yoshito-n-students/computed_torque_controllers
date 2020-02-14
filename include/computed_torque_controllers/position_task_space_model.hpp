#ifndef COMPUTED_TORQUE_CONTROLLERS_POSITION_TASK_SPACE_MODEL_HPP
#define COMPUTED_TORQUE_CONTROLLERS_POSITION_TASK_SPACE_MODEL_HPP

#include <limits>
#include <map>
#include <string>

#include <computed_torque_controllers/acceleration_task_space_model.hpp>
#include <computed_torque_controllers/common_namespaces.hpp>
#include <control_toolbox/pid.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/node_handle.h>

#include <boost/assign/list_of.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>

namespace computed_torque_controllers {

// =======================================================
// dynamics model which calculates required joint efforts
// on the basis of desired end-effector position
class PositionTaskSpaceModel : protected AccelerationTaskSpaceModel {
private:
  struct DofInfo {
    double pos, vel, eff;
    double pos_sp;
    boost::optional< jli::PositionJointSaturationHandle > pos_sp_sat_handle;

    ct::Pid pid;
  };

public:
  PositionTaskSpaceModel() {}

  virtual ~PositionTaskSpaceModel() {}

  // ========================================================
  // name-based interface for ros-controller implementations
  // ========================================================

  // ==========================================
  bool init(const ros::NodeHandle &param_nh) {
    namespace ba = boost::assign;
    namespace rn = ros::names;

    // init the base model first
    if (!AccelerationTaskSpaceModel::init(param_nh)) {
      return false;
    }

    // load PIDs if gains are specified. this is an optional step
    // because PIDs are not required unless position setpoints are given.
    const std::string angular_ns(param_nh.resolveName(rn::append("task_space", "angular")));
    if (!initDofPid(&dof_infos_[0], angular_ns) || !initDofPid(&dof_infos_[1], angular_ns) ||
        !initDofPid(&dof_infos_[2], angular_ns)) {
      return false;
    }
    const std::string linear_ns(param_nh.resolveName(rn::append("task_space", "linear")));
    if (!initDofPid(&dof_infos_[3], linear_ns) || !initDofPid(&dof_infos_[4], linear_ns) ||
        !initDofPid(&dof_infos_[5], linear_ns)) {
      return false;
    }

    // load joint limits for each DoF if given
    ros::NodeHandle limits_nh(param_nh.getNamespace(),
                              /* remappings = */ ba::map_list_of< std::string, std::string >(
                                  "joint_limits/angular", "task_space/angular")(
                                  "joint_limits/linear", "task_space/linear"));
    // try loading limits from 'task_space/angular', which is an alias of 'joint_limits/angular'
    jli::JointLimits angular_limits;
    if (jli::getJointLimits("angular", limits_nh, angular_limits)) {
      if (!initDofLimits(&dof_infos_[0], angular_limits) ||
          !initDofLimits(&dof_infos_[1], angular_limits) ||
          !initDofLimits(&dof_infos_[2], angular_limits)) {
        return false;
      }
    }
    // try loading limits from 'task_space/linear', which is an alias of 'joint_limits/linear'
    jli::JointLimits linear_limits;
    if (jli::getJointLimits("linear", limits_nh, linear_limits)) {
      if (!initDofLimits(&dof_infos_[3], linear_limits) ||
          !initDofLimits(&dof_infos_[4], linear_limits) ||
          !initDofLimits(&dof_infos_[5], linear_limits)) {
        return false;
      }
    }

    return true;
  }

  // ===============================================
  // reset the model states (not destroy the model)
  void reset() {
    BOOST_FOREACH (DofInfo &dof, dof_infos_) {
      dof.pos_sp = std::numeric_limits< double >::quiet_NaN();
      if (dof.pos_sp_sat_handle) {
        dof.pos_sp_sat_handle->reset();
      }
      dof.pid.reset();
    }
  }

  // ===========================================
  // returns end-effector position of the model
  std::map< std::string, double > getPositions() const {
    return eigenToDofMap(getPositionsEigen());
  }

  // ======================================================
  // returns end-effector positions setpoints of the model
  std::map< std::string, double > getSetpoints() const {
    return eigenToDofMap(getSetpointsEigen());
  }

  // ===============================================================
  // update model state based on given joint positions & velocities
  void update(const std::map< std::string, double > &jnt_positions,
              const std::map< std::string, double > &jnt_velocities, const ros::Duration &dt) {
    update(jointMapToEigen(jnt_positions), jointMapToEigen(jnt_velocities), dt);
  }

  // =============================================================================================
  // compute joint effort commands to realize given end-effector pos setpoints based on the model
  std::map< std::string, double >
  computeEffortCommands(const std::map< std::string, double > &pos_setpoints,
                        const ros::Duration &dt) {
    return computeEffortCommands(dofMapToEigen(pos_setpoints), dt);
  }

protected:
  // ======================================================
  // index-based interface for child model implementations
  // ======================================================

  Eigen::Vector6d getPositionsEigen() const {
    Eigen::Vector6d x;
    for (std::size_t i = 0; i < 6; ++i) {
      x[i] = dof_infos_[i].pos;
    }
    return x;
  }

  Eigen::Vector6d getSetpointsEigen() const {
    Eigen::Vector6d xd;
    for (std::size_t i = 0; i < 6; ++i) {
      xd[i] = dof_infos_[i].pos_sp;
    }
    return xd;
  }

  void update(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const ros::Duration &dt) {
    AccelerationTaskSpaceModel::update(q, dq, dt);

    // angular positions
    const Eigen::Isometry3d T(model_end_link_->getWorldTransform());
    const Eigen::AngleAxisd aa(T.linear());
    const Eigen::Vector3d angular_pos(aa.angle() * aa.axis());
    dof_infos_[0].pos = angular_pos[0];
    dof_infos_[1].pos = angular_pos[1];
    dof_infos_[2].pos = angular_pos[2];

    // linear positions
    const Eigen::Vector3d linear_pos(T * end_link_offset_);
    dof_infos_[3].pos = linear_pos[0];
    dof_infos_[4].pos = linear_pos[1];
    dof_infos_[5].pos = linear_pos[2];

    // angular velocities
    const Eigen::Vector3d angular_vel(model_end_link_->getAngularVelocity());
    dof_infos_[0].vel = angular_vel[0];
    dof_infos_[1].vel = angular_vel[1];
    dof_infos_[2].vel = angular_vel[2];

    // linear velocities
    const Eigen::Vector3d linear_vel(model_end_link_->getLinearVelocity(end_link_offset_));
    dof_infos_[3].vel = linear_vel[0];
    dof_infos_[4].vel = linear_vel[1];
    dof_infos_[5].vel = linear_vel[2];

    // efforts (not used)
    for (std::size_t i = 0; i < 6; ++i) {
      dof_infos_[i].eff = std::numeric_limits< double >::quiet_NaN();
    }
  }

  std::map< std::string, double > computeEffortCommands(const Eigen::Vector6d &xd,
                                                        const ros::Duration &dt) {
    // desired position vector
    Eigen::Vector6d xd_clamped;
    for (std::size_t i = 0; i < 6; ++i) {
      DofInfo &dof(dof_infos_[i]);
      // desired position
      dof.pos_sp = xd[i];
      if (dof.pos_sp_sat_handle) {
        dof.pos_sp_sat_handle->enforceLimits(dt);
      }
      xd_clamped[i] = dof.pos_sp;
    }

    // position error vector
    Eigen::Vector6d ex;
    const Eigen::Vector6d x(getPositionsEigen());
    ex = xd_clamped - x; // TODO: fix angular error calculation

    // acc setpoints based on position errors
    Eigen::Vector6d ux;
    for (std::size_t i = 0; i < 6; ++i) {
      DofInfo &dof(dof_infos_[i]);
      ux[i] = dof.pid.computeCommand(ex[i], dt);
    }

    return AccelerationTaskSpaceModel::computeEffortCommands(ux);
  }

private:
  static bool initDofPid(DofInfo *const dof, const std::string &dof_ns) {
    if (!dof->pid.initParam(dof_ns)) {
      ROS_ERROR_STREAM("PositionTaskSpaceModel::initDofPid(): Failed to init a PID by the param '"
                       << dof_ns << "'");
      return false;
    }
    return true;
  }

  static bool initDofLimits(DofInfo *const dof, const jli::JointLimits &limits) {
    const hi::JointStateHandle state_handle("dof", &dof->pos, &dof->vel, &dof->eff);
    dof->pos_sp_sat_handle =
        jli::PositionJointSaturationHandle(hi::JointHandle(state_handle, &dof->pos_sp), limits);
    return true;
  }

private:
  DofInfo dof_infos_[6];
};
} // namespace computed_torque_controllers

#endif