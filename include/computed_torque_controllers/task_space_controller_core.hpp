#ifndef COMPUTED_TORQUE_CONTROLLERS_TASK_SPACE_CONTROLLER_CORE_HPP
#define COMPUTED_TORQUE_CONTROLLERS_TASK_SPACE_CONTROLLER_CORE_HPP

#include <map>
#include <string>

#include <computed_torque_controllers/common_namespaces.hpp>
#include <computed_torque_controllers/joint_controller_core.hpp>
#include <control_toolbox/pid.h>
#include <hardware_interface/robot_hw.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <Eigen/Core>

#include <boost/array.hpp>
#include <boost/foreach.hpp>

namespace computed_torque_controllers {

// ===========================================================================================
// core control implementation without command subscription
//   [ input] position and velocity setpoints of each controlled joint & states of all joints
//   [output] effort commands to controlled joints
class TaskSpaceControllerCore : protected JointControllerCore {
public:
  TaskSpaceControllerCore() {}

  virtual ~TaskSpaceControllerCore() {}

  bool init(const std::string &urdf_str, hi::RobotHW *const hw,
            const std::map< /* controlled joint name */ std::string,
                            /* joint-specific param ns */ std::string > &ctl_joint_namespaces,
            const std::string &linear_dof_ns, const std::string &angular_dof_ns) {
    return JointControllerCore::initModel(urdf_str) &&
           JointControllerCore::initHardware(hw, ctl_joint_namespaces) &&
           initDofs(linear_dof_ns, angular_dof_ns);
  }

  void starting(const ros::Time &time) {
    JointControllerCore::starting(time);

    // reset PIDs
    BOOST_FOREACH (ct::Pid &pid, pids_) { pid.reset(); }
  }

  void update(const ros::Time &time, const ros::Duration &period,
              const boost::array< PosVel, 3 > &linear_dof_setpoints,
              const boost::array< PosVel, 3 > &angular_dof_setpoints) {
    JointControllerCore::updateModel(period);

    const std::map< std::string, PosVel > joint_setpoints(
        updateDofs(period, linear_dof_setpoints, angular_dof_setpoints));

    JointControllerCore::updateHardware(period, joint_setpoints);
  }

  void stopping(const ros::Time &time) { JointControllerCore::stopping(time); }

protected:
  bool initDofs(const std::string &linear_dof_ns, const std::string &angular_dof_ns) {
    // TODO: load the end link name & offset from params
    model_end_link_ = model_->getBodyNode(model_->getNumBodyNodes() - 1);
    end_link_offset_ = Eigen::Vector3d::Zero();

    // init PIDs
    for (std::size_t i = 0; i < 3; ++i) {
      if (!pids_[i].initParam(linear_dof_ns)) {
        ROS_ERROR_STREAM(
            "TaskSpaceControllerCore::initDofs(): Failed to init a PID by the param '"
            << linear_dof_ns << "'");
        return false;
      }
    }
    for (std::size_t i = 3; i < 6; ++i) {
      if (!pids_[i].initParam(angular_dof_ns)) {
        ROS_ERROR_STREAM(
            "TaskSpaceControllerCore::initDofs(): Failed to init a PID by the param '"
            << angular_dof_ns << "'");
        return false;
      }
    }

    return true;
  }

  std::map< std::string, PosVel >
  updateDofs(const ros::Duration &period, const boost::array< PosVel, 3 > &linear_dof_setpoints,
             const boost::array< PosVel, 3 > &angular_dof_setpoints) {
    // compute reference velocity in task space
    Eigen::Vector6d v_r;
    for (std::size_t i = 0; i < 3; ++i) {
      const PosVel &sp(angular_dof_setpoints[i]);
      v_r[i] = sp.vel + pids_[i].computeCommand(sp.pos, period);
    }
    for (std::size_t i_sp = 0; i_sp < 3; ++i_sp) {
      const std::size_t i_pid(i_sp + 3);
      const PosVel &sp(linear_dof_setpoints[i_sp]);
      v_r[i_sp] = sp.vel + pids_[i_pid].computeCommand(sp.pos, period);
    }

    // compute Jacobian and its pseudo-inverse
    const dm::Jacobian J(model_end_link_->getWorldJacobian(end_link_offset_));
    const Eigen::MatrixXd pinv_J(
        J.transpose() * (J * J.transpose() + 0.0025 * Eigen::Matrix6d::Identity()).inverse());

    // convert reference velocity from task to joint spaces
    std::map< std::string, PosVel > ctl_joint_setpoints;
    BOOST_FOREACH (const ControlledHardwareJointMap::value_type &joint_val, ctl_hw_joints_) {
      const std::string &name(joint_val.first);
      const ControlledHardwareJoint &joint(*joint_val.second);
      const std::size_t id(joint.id_in_model);

      PosVel &sp(ctl_joint_setpoints[name]);
      sp.vel = pinv_J.row(id) * v_r /* + opt function */;
      sp.pos = joint.state_handle.getPosition() + sp.vel * period.toSec();
    }

    return ctl_joint_setpoints;
  }

protected:
  dd::BodyNodePtr model_end_link_;
  Eigen::Vector3d end_link_offset_;
  ct::Pid pids_[6];
};

} // namespace computed_torque_controllers

#endif