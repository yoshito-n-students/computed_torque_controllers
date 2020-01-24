#ifndef COMPUTED_TORQUE_CONTROLLERS_TASK_SPACE_CONTROLLER_CORE_HPP
#define COMPUTED_TORQUE_CONTROLLERS_TASK_SPACE_CONTROLLER_CORE_HPP

#include <limits>
#include <map>
#include <string>

#include <computed_torque_controllers/common_namespaces.hpp>
#include <computed_torque_controllers/joint_controller_core.hpp>
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/time.h>

#include <Eigen/Core>

#include <boost/assign/list_of.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>

namespace computed_torque_controllers {

// ===========================================================================================
// core control implementation without command subscription
//   [ input] position and velocity setpoints of each DoF in task space
//   [output] velocity commands to controlled joints
class TaskSpaceControllerCore : protected JointControllerCore {
protected:
  // task space DoF
  struct Dof {
    // states
    double pos, vel, eff;
    // setpoints, command, and their limits
    double pos_sp, vel_sp, vel_cmd;
    boost::optional< jli::PositionJointSaturationHandle > pos_sp_sat_handle;
    boost::optional< jli::VelocityJointSaturationHandle > vel_sp_sat_handle, vel_cmd_sat_handle;
    // PID for integrating position setpoint into velocity command
    boost::optional< ct::Pid > pid;
  };
  typedef std::map< std::string, Dof > DofMap;

public:
  TaskSpaceControllerCore() {}

  virtual ~TaskSpaceControllerCore() {}

  bool init(hi::RobotHW *const hw, ros::NodeHandle &param_nh) {
    return JointControllerCore::init(hw, param_nh) && initDofs(param_nh);
  }

  void starting() {
    JointControllerCore::starting();

    // reset stateful objects
    BOOST_FOREACH (DofMap::value_type &dof_val, dofs_) {
      Dof &dof(dof_val.second);
      if (dof.pos_sp_sat_handle) {
        dof.pos_sp_sat_handle->reset();
      }
      if (dof.pid) {
        dof.pid->reset();
      }
    }
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
    namespace ba = boost::assign;
    namespace rn = ros::names;
    namespace rp = ros::param;

    // TODO: load the end link name & offset from params
    model_end_link_ = model_->getBodyNode(model_->getNumBodyNodes() - 1);
    end_link_offset_ = Eigen::Vector3d::Zero();

    // allocate task space DoF infos
    dofs_["linear_x"] = Dof();
    dofs_["linear_y"] = Dof();
    dofs_["linear_z"] = Dof();
    dofs_["angular_x"] = Dof();
    dofs_["angular_y"] = Dof();
    dofs_["angular_z"] = Dof();

    // load PIDs if gains are specified. this is an optional step
    // because PIDs are not required unless position setpoints are given.
    const std::string linear_ns(param_nh.resolveName(rn::append("task_space", "linear")));
    if (rp::has(rn::append(linear_ns, "p"))) {
      if (!initDofPid("linear_x", linear_ns) || !initDofPid("linear_y", linear_ns) ||
          !initDofPid("linear_z", linear_ns)) {
        return false;
      }
    }
    const std::string angular_ns(param_nh.resolveName(rn::append("task_space", "angular")));
    if (rp::has(rn::append(angular_ns, "p"))) {
      if (!initDofPid("angular_x", angular_ns) || !initDofPid("angular_y", angular_ns) ||
          !initDofPid("angular_z", angular_ns)) {
        return false;
      }
    }

    // load joint limits for each DoF if given
    ros::NodeHandle limits_nh(param_nh.getNamespace(),
                              /* remappings = */ ba::map_list_of< std::string, std::string >(
                                  "joint_limits/linear", "task_space/linear")(
                                  "joint_limits/angular", "task_space/angular"));
    // try loading limits from 'task_space/linear', which is an alias of 'joint_limits/linear'
    jli::JointLimits linear_limits;
    if (jli::getJointLimits("linear", limits_nh, linear_limits)) {
      if (!initDofLimits("linear_x", linear_limits) || !initDofLimits("linear_y", linear_limits) ||
          !initDofLimits("linear_z", linear_limits)) {
        return false;
      }
    }
    // try loading limits from 'task_space/angular', which is an alias of 'joint_limits/angular'
    jli::JointLimits angular_limits;
    if (jli::getJointLimits("angular", limits_nh, angular_limits)) {
      if (!initDofLimits("angular_x", angular_limits) ||
          !initDofLimits("angular_y", angular_limits) ||
          !initDofLimits("angular_z", angular_limits)) {
        return false;
      }
    }

    return true;
  }

  bool initDofPid(const std::string &dof_name, const std::string &dof_ns) {
    Dof *const dof(findValue(dofs_, dof_name));
    if (!dof) {
      ROS_ERROR_STREAM("TaskSpaceControllerCore::initDofPid(): Unknown DoF '" << dof_name << "'");
      return false;
    }

    dof->pid.emplace();
    if (!dof->pid->initParam(dof_ns)) {
      ROS_ERROR_STREAM("TaskSpaceControllerCore::initDofPid(): Failed to init a PID by the param '"
                       << dof_ns << "'");
      return false;
    }

    return true;
  }

  bool initDofLimits(const std::string &dof_name, const jli::JointLimits &limits) {
    Dof *const dof(findValue(dofs_, dof_name));
    if (!dof) {
      ROS_ERROR_STREAM("TaskSpaceControllerCore::initDofLimits(): Unknown DoF '" << dof_name
                                                                                 << "'");
      return false;
    }

    const hi::JointStateHandle state_handle(dof_name, &dof->pos, &dof->vel, &dof->eff);
    dof->pos_sp_sat_handle =
        jli::PositionJointSaturationHandle(hi::JointHandle(state_handle, &dof->pos_sp), limits);
    dof->vel_sp_sat_handle =
        jli::VelocityJointSaturationHandle(hi::JointHandle(state_handle, &dof->vel_sp), limits);
    dof->vel_cmd_sat_handle =
        jli::VelocityJointSaturationHandle(hi::JointHandle(state_handle, &dof->vel_cmd), limits);

    return true;
  }

  std::map< std::string, double > updateDofs(const ros::Duration &period,
                                             const std::map< std::string, double > &pos_setpoints,
                                             const std::map< std::string, double > &vel_setpoints) {
    // update DoF states
    updateDofStates();

    // set velocity command in task space by integrating position & velocity setpoints
    // according to 'v_r = v_sp + PID(p_sp - p)'
    Eigen::Vector6d v_r;
    v_r[0] = updateDofCommand(period, "angular_x", vel_setpoints, pos_setpoints);
    v_r[1] = updateDofCommand(period, "angular_y", vel_setpoints, pos_setpoints);
    v_r[2] = updateDofCommand(period, "angular_z", vel_setpoints, pos_setpoints);
    v_r[3] = updateDofCommand(period, "linear_x", vel_setpoints, pos_setpoints);
    v_r[4] = updateDofCommand(period, "linear_y", vel_setpoints, pos_setpoints);
    v_r[5] = updateDofCommand(period, "linear_z", vel_setpoints, pos_setpoints);

    // compute Jacobian and its pseudo-inverse
    const dm::Jacobian J(model_end_link_->getWorldJacobian(end_link_offset_));
    const Eigen::MatrixXd pinv_J(
        J.transpose() * (J * J.transpose() + 0.0025 * Eigen::Matrix6d::Identity()).inverse());

    // convert velocity command from task to joint spaces
    std::map< std::string, double > ctl_joint_setpoints;
    BOOST_FOREACH (const ControlledHardwareJointMap::value_type &joint_val, ctl_hw_joints_) {
      const std::string &name(joint_val.first);
      const ControlledHardwareJoint &joint(*joint_val.second);
      const std::size_t id(joint.id_in_model);

      ctl_joint_setpoints[name] = pinv_J.row(id) * v_r /* + opt function */;
    }

    return ctl_joint_setpoints;
  }

  void updateDofStates() {
    // references to DoFs
    Dof &linear_x_dof(*findValue(dofs_, std::string("linear_x"))),
        &linear_y_dof(*findValue(dofs_, "linear_y")), &linear_z_dof(*findValue(dofs_, "linear_z")),
        &angular_x_dof(*findValue(dofs_, "angular_x")),
        &angular_y_dof(*findValue(dofs_, "angular_y")),
        &angular_z_dof(*findValue(dofs_, "angular_z"));

    // linear positions
    const Eigen::Isometry3d T(model_end_link_->getWorldTransform());
    const Eigen::Vector3d linear_pos(T * end_link_offset_);
    linear_x_dof.pos = linear_pos[0];
    linear_y_dof.pos = linear_pos[1];
    linear_z_dof.pos = linear_pos[2];

    // angular positions
    const Eigen::AngleAxisd aa(T.linear());
    const Eigen::Vector3d angular_pos(aa.angle() * aa.axis());
    angular_x_dof.pos = angular_pos[0];
    angular_y_dof.pos = angular_pos[1];
    angular_z_dof.pos = angular_pos[2];

    // linear velocities
    const Eigen::Vector3d linear_vel(model_end_link_->getLinearVelocity(end_link_offset_));
    linear_x_dof.vel = linear_vel[0];
    linear_y_dof.vel = linear_vel[1];
    linear_z_dof.vel = linear_vel[2];

    // angular velocities
    const Eigen::Vector3d angular_vel(model_end_link_->getAngularVelocity());
    angular_x_dof.vel = angular_vel[0];
    angular_y_dof.vel = angular_vel[1];
    angular_z_dof.vel = angular_vel[2];

    // efforts (not used)
    linear_x_dof.eff = std::numeric_limits< double >::quiet_NaN();
    linear_y_dof.eff = std::numeric_limits< double >::quiet_NaN();
    linear_z_dof.eff = std::numeric_limits< double >::quiet_NaN();
    angular_x_dof.eff = std::numeric_limits< double >::quiet_NaN();
    angular_y_dof.eff = std::numeric_limits< double >::quiet_NaN();
    angular_z_dof.eff = std::numeric_limits< double >::quiet_NaN();
  }

  // make new velocity setpoint by integrating the original velocity & position setpoints.
  // the original setpoints may be missing.
  double updateDofCommand(const ros::Duration &period, const std::string &dof_name,
                          const std::map< std::string, double > &vel_setpoints,
                          const std::map< std::string, double > &pos_setpoints) {
    //
    Dof *const dof(findValue(dofs_, dof_name));
    if (!dof) {
      ROS_ERROR_STREAM(
          "TaskSpaceControllerCore::integrateSetpoints(): Unknown task space DoF name '" << dof_name
                                                                                         << "'");
      return 0.;
    }

    // start integration
    dof->vel_cmd = 0.;

    // integrate velocity setpoint into command
    const double *const vel_sp(findValue(vel_setpoints, dof_name));
    if (vel_sp) {
      dof->vel_sp = *vel_sp;
      if (dof->vel_sp_sat_handle) {
        dof->vel_sp_sat_handle->enforceLimits(period);
      }
      dof->vel_cmd += dof->vel_sp;
    }

    // integrate position setpoint into command
    const double *const pos_sp(findValue(pos_setpoints, dof_name));
    if (pos_sp) {
      dof->pos_sp = *pos_sp;
      if (dof->pos_sp_sat_handle) {
        dof->pos_sp_sat_handle->enforceLimits(period);
      }
      if (dof->pid) {
        dof->vel_cmd += dof->pid->computeCommand(dof->pos_sp - dof->pos, period);
      } else {
        ROS_ERROR_STREAM(
            "TaskSpaceControllerCore::integrateSetpoints(): No PID controller found for DoF '"
            << dof_name << "'. Will ignore position setpoint in task space.");
      }
    }

    // saturate integrated command
    if (dof->vel_cmd_sat_handle) {
      dof->vel_cmd_sat_handle->enforceLimits(period);
    }

    return dof->vel_cmd;
  }

protected:
  dd::BodyNodePtr model_end_link_;
  Eigen::Vector3d end_link_offset_;
  DofMap dofs_;
};

} // namespace computed_torque_controllers

#endif