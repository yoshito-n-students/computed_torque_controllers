#ifndef COMPUTED_TORQUE_CONTROLLERS_POSITION_JOINT_MODEL_HPP
#define COMPUTED_TORQUE_CONTROLLERS_POSITION_JOINT_MODEL_HPP

#include <limits>
#include <map>
#include <string>
#include <vector>

#include <computed_torque_controllers/acceleration_joint_model.hpp>
#include <computed_torque_controllers/common_namespaces.hpp>
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <urdf/model.h>

#include <Eigen/Core>

#include <boost/foreach.hpp>
#include <boost/optional.hpp>

namespace computed_torque_controllers {

// =======================================================
// dynamics model which calculates required joint efforts
// on the basis of desired joint positions
class PositionJointModel : protected AccelerationJointModel {
public:
  PositionJointModel() {}

  virtual ~PositionJointModel() {}

  // ========================================================
  // name-based interface for ros-controller implementations
  // ========================================================

  // ======================================================
  // initialize the model by params in the given namespace
  bool init(const ros::NodeHandle &param_nh) {
    // init the base model first
    if (!AccelerationJointModel::init(param_nh)) {
      return false;
    }

    // parse robot description to extract joint names & limits
    urdf::Model robot_desc;
    if (!robot_desc.initString(getRobotDescription(param_nh))) {
      ROS_ERROR("PositionJointModel::init(): Failed to parse robot description as an URDF");
      return false;
    }

    // compose joint info
    joint_infos_.resize(model_->getNumDofs());
    for (std::size_t i = 0; i < model_->getNumDofs(); ++i) {
      const std::string &name(jointIdToName(i));
      JointInfo &info(joint_infos_[i]);

      // load joint limits from robot description if limits exist
      jli::JointLimits limits;
      if (jli::getJointLimits(robot_desc.getJoint(name), limits)) {
        const hi::JointStateHandle state_handle(name, &info.pos, &info.vel, &info.eff);
        const hi::JointHandle pos_sp_handle(state_handle, &info.pos_sp);
        info.pos_sp_sat_handle = jli::PositionJointSaturationHandle(pos_sp_handle, limits);
      }

      // PID controller to generate effort command based on position error
      const std::string pid_ns(param_nh.resolveName(ros::names::append("joints", name)));
      if (!info.pid.initParam(pid_ns)) {
        ROS_ERROR_STREAM("PositionJointModel::init(): Failed to init a PID by param '" << pid_ns
                                                                                       << "'");
        return false;
      }
    }

    return true;
  }

  // ===============================================
  // reset the model states (not destroy the model)
  void reset() {
    BOOST_FOREACH (JointInfo &info, joint_infos_) {
      info.pos_sp = std::numeric_limits< double >::quiet_NaN();
      if (info.pos_sp_sat_handle) {
        info.pos_sp_sat_handle->reset();
      }
      info.pid.reset();
    }
  }

  // =====================================
  // returns joint positions of the model
  std::map< std::string, double > getPositions() const {
    return eigenToJointMap(getPositionsEigen());
  }

  // =====================================
  // returns joint setpoints of the model
  std::map< std::string, double > getSetpoints() const {
    return eigenToJointMap(getSetpointsEigen());
  }

  // ===============================================================
  // update model state based on given joint positions & velocities
  void update(const std::map< std::string, double > &positions,
              const std::map< std::string, double > &velocities, const ros::Duration &dt) {
    update(jointMapToEigen(positions), jointMapToEigen(velocities), dt);
  }

  // ================================================================================
  // compute joint effort commands to realize given pos setpoints based on the model
  std::map< std::string, double >
  computeEffortCommands(const std::map< std::string, double > &pos_setpoints,
                        const ros::Duration &dt) {
    return computeEffortCommands(jointMapToEigen(pos_setpoints), dt);
  }

protected:
  // ======================================================
  // index-based interface for child model implementations
  // ======================================================

  Eigen::VectorXd getPositionsEigen() const {
    Eigen::VectorXd q(joint_infos_.size());
    for (std::size_t i = 0; i < joint_infos_.size(); ++i) {
      q[i] = joint_infos_[i].pos;
    }
    return q;
  }

  Eigen::VectorXd getSetpointsEigen() const {
    Eigen::VectorXd qd(joint_infos_.size());
    for (std::size_t i = 0; i < joint_infos_.size(); ++i) {
      qd[i] = joint_infos_[i].pos_sp;
    }
    return qd;
  }

  void update(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const ros::Duration &dt) {
    // update the base model
    AccelerationJointModel::update(q, dq, dt);

    for (std::size_t i = 0; i < model_->getNumDofs(); ++i) {
      JointInfo &info(joint_infos_[i]);
      info.pos = q[i];
      info.vel = dq[i];
      info.eff = std::numeric_limits< double >::quiet_NaN(); // no info
    }
  }

  std::map< std::string, double > computeEffortCommands(const Eigen::VectorXd &qd,
                                                        const ros::Duration &dt) {
    // convert position setpoints to acceleration setpoints
    // by saturating position setpoints according to the joint limits
    // & applying PID controllers.
    Eigen::VectorXd u(model_->getNumDofs());
    for (std::size_t i = 0; i < model_->getNumDofs(); ++i) {
      JointInfo &info(joint_infos_[i]);
      info.pos_sp = qd[i];
      if (info.pos_sp_sat_handle) {
        info.pos_sp_sat_handle->enforceLimits(dt);
      }
      u[i] = info.pid.computeCommand(info.pos_sp - info.pos, dt);
    }

    // calculate effort commands based on the acceleration setpoints
    return AccelerationJointModel::computeEffortCommands(u);
  }

private:
  struct JointInfo {
    double pos, vel, eff;
    double pos_sp;
    boost::optional< jli::PositionJointSaturationHandle > pos_sp_sat_handle;

    ct::Pid pid;
  };
  std::vector< JointInfo > joint_infos_;
};

} // namespace computed_torque_controllers

#endif