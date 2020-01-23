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
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/param.h>
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

  std::map< std::string, double > getEndEffectorPosition() const {
    std::map< std::string, double > p;

    const Eigen::Isometry3d T(model_end_link_->getWorldTransform());
    const Eigen::Vector3d linear(T * end_link_offset_);
    p["linear_x"] = linear[0];
    p["linear_y"] = linear[1];
    p["linear_z"] = linear[2];

    const Eigen::AngleAxisd aa(T.linear());
    const Eigen::Vector3d angular(aa.angle() * aa.axis());
    p["angular_x"] = angular[0];
    p["angular_y"] = angular[1];
    p["angular_z"] = angular[2];

    return p;
  }

protected:
  bool initDofs(ros::NodeHandle &param_nh) {
    namespace rn = ros::names;
    namespace rp = ros::param;

    // TODO: load the end link name & offset from params
    model_end_link_ = model_->getBodyNode(model_->getNumBodyNodes() - 1);
    end_link_offset_ = Eigen::Vector3d::Zero();

    // load PIDs if gains are specified. this is an optional step
    // because PIDs are not required unless position setpoints are given.
    const std::string linear_ns(param_nh.resolveName(rn::append("task_space", "linear")));
    if (rp::has(rn::append(linear_ns, "p"))) {
      if (!pids_["linear_x"].initParam(linear_ns) || !pids_["linear_y"].initParam(linear_ns) ||
          !pids_["linear_z"].initParam(linear_ns)) {
        ROS_ERROR_STREAM("TaskSpaceControllerCore::initDofs(): Failed to init a PID by the param '"
                         << linear_ns << "'");
        return false;
      }
    }
    const std::string angular_ns(param_nh.resolveName(rn::append("task_space", "angular")));
    if (rp::has(rn::append(angular_ns, "p"))) {
      if (!pids_["angular_x"].initParam(angular_ns) || !pids_["angular_y"].initParam(angular_ns) ||
          !pids_["angular_z"].initParam(angular_ns)) {
        ROS_ERROR_STREAM("TaskSpaceControllerCore::initDofs(): Failed to init a PID by the param '"
                         << angular_ns << "'");
        return false;
      }
    }

    return true;
  }

  std::map< std::string, double > updateDofs(const ros::Duration &period,
                                             const std::map< std::string, double > &pos_setpoints,
                                             const std::map< std::string, double > &vel_setpoints) {
    // set reference velocity in task space by integrating position & velocity setpoints
    // according to 'v_r = v_sp + PID(p_sp - p)'
    Eigen::Vector6d v_r;
    const std::map< std::string, double > pos(getEndEffectorPosition());
    v_r[0] = integrateSetpoints(period, "angular_x", vel_setpoints, pos_setpoints, pos);
    v_r[1] = integrateSetpoints(period, "angular_y", vel_setpoints, pos_setpoints, pos);
    v_r[2] = integrateSetpoints(period, "angular_z", vel_setpoints, pos_setpoints, pos);
    v_r[3] = integrateSetpoints(period, "linear_x", vel_setpoints, pos_setpoints, pos);
    v_r[4] = integrateSetpoints(period, "linear_y", vel_setpoints, pos_setpoints, pos);
    v_r[5] = integrateSetpoints(period, "linear_z", vel_setpoints, pos_setpoints, pos);

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

  // make new velocity setpoint by integrating the original velocity & position setpoints.
  // the original setpoints may be missing.
  double integrateSetpoints(const ros::Duration &period, const std::string &dof_name,
                            const std::map< std::string, double > &vel_setpoints,
                            const std::map< std::string, double > &pos_setpoints,
                            const std::map< std::string, double > &positions) {
    // find values to be integrated
    const double *const vel_sp(findValue(vel_setpoints, dof_name));
    ct::Pid *const pid(findValue(pids_, dof_name));
    const double *const pos_sp(findValue(pos_setpoints, dof_name));
    const double *const pos(findValue(positions, dof_name));

    // print error if position setpoint cannot be integrated
    if (pos_sp && !pid) {
      ROS_ERROR_STREAM(
          "TaskSpaceControllerCore::integrateSetpoints(): No pid controller found for Dof '"
          << dof_name << "'. Will ignore position setpoint in the task space.");
    }

    // return 'vel_sp + PID(pos_sp - pos)'
    return (vel_sp ? *vel_sp : 0.) +
           (pid && pos_sp && pos ? pid->computeCommand(*pos_sp - *pos, period) : 0.);
  }

protected:
  dd::BodyNodePtr model_end_link_;
  Eigen::Vector3d end_link_offset_;
  PidMap pids_;
};

} // namespace computed_torque_controllers

#endif