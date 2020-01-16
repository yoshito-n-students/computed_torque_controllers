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

#include <boost/foreach.hpp>

namespace computed_torque_controllers {

// ===========================================================================================
// core control implementation without command subscription
//   [ input] position and velocity setpoints of each controlled joint & states of all joints
//   [output] effort commands to controlled joints
class TaskSpaceControllerCore : protected JointControllerCore {
protected:
  typedef std::map< std::string, ct::Pid > PidMap;

public:
  TaskSpaceControllerCore() {}

  virtual ~TaskSpaceControllerCore() {}

  bool init(hi::RobotHW *const hw, ros::NodeHandle &param_nh) {
    return JointControllerCore::init(hw, param_nh) && initDofs(param_nh);
  }

  void starting() {
    JointControllerCore::starting();

    // reset PIDs
    BOOST_FOREACH (PidMap::value_type &pid, pids_) { pid.second.reset(); }
  }

  void update(const ros::Duration &period, const std::map< std::string, double > &pos_setpoints,
              const std::map< std::string, double > &vel_setpoints) {
    const std::map< std::string, double > joint_setpoints(
        updateDofs(period, pos_setpoints, vel_setpoints));

    JointControllerCore::update(period,
                                /* pos_joint_setpoints = */ std::map< std::string, double >(),
                                /* vel_joint_setpoints = */ joint_setpoints);
  }

  void stopping() { JointControllerCore::stopping(); }

protected:
  bool initDofs(ros::NodeHandle &param_nh) {
    // TODO: load the end link name & offset from params
    model_end_link_ = model_->getBodyNode(model_->getNumBodyNodes() - 1);
    end_link_offset_ = Eigen::Vector3d::Zero();

    // init PIDs
    const std::string linear_ns(param_nh.resolveName(ros::names::append("task_space", "linear")));
    if (!pids_["linear_x"].initParam(linear_ns) || !pids_["linear_y"].initParam(linear_ns) ||
        !pids_["linear_z"].initParam(linear_ns)) {
      ROS_ERROR_STREAM("TaskSpaceControllerCore::initDofs(): Failed to init a PID by the param '"
                       << linear_ns << "'");
      return false;
    }
    const std::string angular_ns(param_nh.resolveName(ros::names::append("task_space", "angular")));
    if (!pids_["angular_x"].initParam(angular_ns) || !pids_["angular_y"].initParam(angular_ns) ||
        !pids_["angular_z"].initParam(angular_ns)) {
      ROS_ERROR_STREAM("TaskSpaceControllerCore::initDofs(): Failed to init a PID by the param '"
                       << angular_ns << "'");
      return false;
    }

    return true;
  }

  std::map< std::string, double > updateDofs(const ros::Duration &period,
                                             const std::map< std::string, double > &pos_setpoints,
                                             const std::map< std::string, double > &vel_setpoints) {
    // set reference velocity in task space
    Eigen::Vector6d v_r;
    v_r[0] = integrateSetpoints(period, "angular_x", pos_setpoints, vel_setpoints);
    v_r[1] = integrateSetpoints(period, "angular_y", pos_setpoints, vel_setpoints);
    v_r[2] = integrateSetpoints(period, "angular_z", pos_setpoints, vel_setpoints);
    v_r[3] = integrateSetpoints(period, "linear_x", pos_setpoints, vel_setpoints);
    v_r[4] = integrateSetpoints(period, "linear_y", pos_setpoints, vel_setpoints);
    v_r[5] = integrateSetpoints(period, "linear_z", pos_setpoints, vel_setpoints);

    // compute Jacobian and its pseudo-inverse
    const dm::Jacobian J(model_end_link_->getWorldJacobian(end_link_offset_));
    const Eigen::MatrixXd pinv_J(
        J.transpose() * (J * J.transpose() + 0.0025 * Eigen::Matrix6d::Identity()).inverse());

    // convert reference velocity from task to joint spaces
    std::map< std::string, double > ctl_joint_setpoints;
    BOOST_FOREACH (const ControlledHardwareJointMap::value_type &joint_val, ctl_hw_joints_) {
      const std::string &name(joint_val.first);
      const ControlledHardwareJoint &joint(*joint_val.second);
      const std::size_t id(joint.id_in_model);

      ctl_joint_setpoints[name] = pinv_J.row(id) * v_r /* + opt function */;
    }

    return ctl_joint_setpoints;
  }

  double integrateSetpoints(const ros::Duration &period, const std::string &dof_name,
                            const std::map< std::string, double > &pos_setpoints,
                            const std::map< std::string, double > &vel_setpoints) {
    const double *const pos_sp(findValue(pos_setpoints, dof_name));
    const double *const vel_sp(findValue(vel_setpoints, dof_name));
    ct::Pid *const pid(findValue(pids_, dof_name));

    if (pos_sp && vel_sp && pid) {
      return *vel_sp + pid->computeCommand(*pos_sp /* - present_dof_pos */, period);
    } else if (pos_sp && vel_sp && !pid) {
      ROS_ERROR_STREAM(
          "TaskSpaceControllerCore::integrateSetpoints(): No pid controller found for Dof '"
          << dof_name << "'. Will ignore position setpoint in the task space.");
      return *vel_sp;
    } else if (pos_sp && !vel_sp && pid) {
      return pid->computeCommand(*pos_sp /* - present_dof_pos */, period);
    } else if (pos_sp && !vel_sp && !pid) {
      ROS_ERROR_STREAM(
          "TaskSpaceControllerCore::integrateSetpoints(): No pid controller found for Dof '"
          << dof_name << "'. Will ignore position setpoint in the task space.");
      return 0.;
    } else if (!pos_sp && vel_sp) {
      return *vel_sp;
    } else if (!pos_sp && !vel_sp) {
      return 0.;
    } else {
      ROS_ERROR_STREAM("TaskSpaceControllerCore::integrateSetpoints(): Bug...");
      return 0.;
    }
  }

protected:
  dd::BodyNodePtr model_end_link_;
  Eigen::Vector3d end_link_offset_;
  PidMap pids_;
};

} // namespace computed_torque_controllers

#endif