#ifndef COMPUTED_TORQUE_CONTROLLER_COMPUTED_TORQUE_CONTROLLER_HPP
#define COMPUTED_TORQUE_CONTROLLER_COMPUTED_TORQUE_CONTROLLER_HPP

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <computed_torque_controller/common_namespaces.hpp>
#include <computed_torque_controller/ros_package_resource_retriever.hpp>
#include <control_toolbox/pid.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/hardware_interface.h> // for HardwareInterfaceException
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <urdf/model.h>

#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/Joint.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

#include <Eigen/Core>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

namespace computed_torque_controller {

class ComputedTorqueController
    : public ci::MultiInterfaceController< hi::JointStateInterface, hi::EffortJointInterface > {
private:
  struct ObservedJointInfo {
    // [dynamics model]
    dd::Joint *model_joint;
    std::size_t id_in_model;
    // [state from the hardware]
    hi::JointStateHandle hw_state_handle;
  };
  typedef boost::shared_ptr< ObservedJointInfo > ObservedJointInfoPtr;
  typedef boost::shared_ptr< const ObservedJointInfo > ObservedJointInfoConstPtr;

  struct ControlledJointInfo : public ObservedJointInfo {
    // [position setpoint (input) from ROS topic]
    double pos_sp;
    rt::RealtimeBuffer< double > pos_sp_buf;
    ros::Subscriber pos_sp_sub;
    boost::optional< jli::PositionJointSaturationHandle > pos_sp_sat_handle;
    // [effort command (output) to the hardware]
    hi::JointHandle hw_eff_cmd_handle;
    // [PID controller to generate effort command based on position error]
    ct::Pid pid;
  };
  typedef boost::shared_ptr< ControlledJointInfo > ControlledJointInfoPtr;

public:
  ComputedTorqueController() {}

  virtual ~ComputedTorqueController() {}

  // required interfaces as a Controller

  virtual bool init(hi::RobotHW *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
    namespace rn = ros::names;
    namespace rp = ros::param;

    // get robot description in URDF from param
    std::string urdf_str;
    if (!controller_nh.getParam("robot_description", urdf_str) &&
        !rp::get("robot_description", urdf_str)) {
      ROS_ERROR_STREAM("ComputedTorqueController::init(): Faild to get robot description from '"
                       << controller_nh.resolveName("robot_description") << "' nor '"
                       << rn::resolve("robot_description") << "'");
      return false;
    }

    // build a dynamics model based on URDF description
    model_ = du::DartLoader().parseSkeletonString(
        urdf_str,
        // base URI to resolve relative URIs in the URDF.
        // this won't be used because almost all URIs are absolute.
        dc::Uri("file:/"), std::make_shared< ROSPackageResourceRetriever >());
    if (!model_) {
      ROS_ERROR("ComputedTorqueController::init(): Failed to build a dynamics model from "
                "robot description");
      return false;
    }

    // get model root joint between root link and world, which does not exist in the hardware
    model_root_joint_ = dynamic_cast< dd::FreeJoint * >(model_->getRootJoint());
    if (!model_root_joint_) {
      ROS_ERROR(
          "ComputedTorqueController::init(): Faild to get a root joint of the dynamics model");
      return false;
    }

    // parse robot description to extract joint limits
    urdf::Model urdf_desc;
    if (!urdf_desc.initString(urdf_str)) {
      ROS_ERROR("ComputedTorqueController::init(): Failed to parse robot description as an URDF");
      return false;
    }

    // get required hardware interfaces
    // (no need to check existence of interfaces because the base class did it)
    hi::JointStateInterface &hw_state_iface(*hw->get< hi::JointStateInterface >());
    hi::EffortJointInterface &hw_eff_cmd_iface(*hw->get< hi::EffortJointInterface >());

    // compose joint info by matching joints in the dynamics model & hardware
    BOOST_FOREACH (dd::Joint *const model_joint, model_->getJoints()) {
      // skip the model root joint already handled
      if (model_joint == model_->getRootJoint()) {
        continue;
      }

      // assert single Dof model joint because hw joint only supports single Dof
      const std::string joint_name(model_joint->getName());
      if (model_joint->getNumDofs() != 1) {
        ROS_ERROR_STREAM(
            "ComputedTorqueController::init(): Multi-Dof joint is not supported (name: '"
            << joint_name << "', dofs: " << model_joint->getNumDofs() << ")");
        return false;
      }

      // allocate joint info (observed or observed-and-controlled)
      ObservedJointInfoPtr joint_info;
      ControlledJointInfoPtr ctl_joint_info;
      const std::string joint_ns(controller_nh.resolveName(rn::append("joints", joint_name)));
      if (rp::has(joint_ns)) {
        ctl_joint_info.reset(new ControlledJointInfo());
        joint_info = ctl_joint_info;
      } else {
        joint_info.reset(new ObservedJointInfo());
      }

      // for each joint: info about the dynamics model
      joint_info->model_joint = model_joint;
      joint_info->id_in_model = model_joint->getIndexInSkeleton(/* 1st DoF */ 0);

      // for each joint: info about hardware joint state handle
      try {
        joint_info->hw_state_handle = hw_state_iface.getHandle(joint_name);
      } catch (const hi::HardwareInterfaceException &ex) {
        ROS_ERROR_STREAM("ComputedTorqueController::init(): Failed to get the state handle of '"
                         << joint_name << "': " << ex.what());
        return false;
      }

      // store info required for all joints
      all_joints_.push_back(joint_info);

      // optional procedure for contolled joints
      if (ctl_joint_info) {
        // [info about position setpoint from ROS topic]
        // subscription
        ctl_joint_info->pos_sp_sub = controller_nh.subscribe< std_msgs::Float64 >(
            rn::append(joint_name, "command"), 1,
            boost::bind(&ComputedTorqueController::positionSetpointCB, _1,
                        &ctl_joint_info->pos_sp_buf));
        // satulation based on limits (optional)
        jli::JointLimits pos_sp_limits;
        if (jli::getJointLimits(urdf_desc.getJoint(joint_name), pos_sp_limits)) {
          const hi::JointHandle pos_sp_handle(ctl_joint_info->hw_state_handle,
                                              &ctl_joint_info->pos_sp);
          ctl_joint_info->pos_sp_sat_handle =
              jli::PositionJointSaturationHandle(pos_sp_handle, pos_sp_limits);
        }

        // [effort command to the hardware]
        try {
          ctl_joint_info->hw_eff_cmd_handle = hw_eff_cmd_iface.getHandle(joint_name);
        } catch (const hi::HardwareInterfaceException &ex) {
          ROS_ERROR_STREAM(
              "ComputedTorqueController::init(): Failed to get the effort command handle of '"
              << joint_name << "': " << ex.what());
          return false;
        }

        // [PID controller to generate effort command based on position error]
        if (!ctl_joint_info->pid.initParam(joint_ns)) {
          ROS_ERROR_STREAM("ComputedTorqueController::init(): Failed to init a PID by param '"
                           << joint_ns << "'");
          return false;
        }

        // store info required only for controlled joints
        controlled_joints_.push_back(ctl_joint_info);
      }
    }

    // assert one controlled joint at least
    if (controlled_joints_.empty()) {
      ROS_ERROR_STREAM("ComputedTorqueController::init(): No controlled joint loaded from param '"
                       << controller_nh.resolveName("joints") << "'");
      return false;
    }

    return true;
  }

  virtual void starting(const ros::Time &time) {
    BOOST_FOREACH (const ControlledJointInfoPtr &ctl_joint, controlled_joints_) {
      // reset position setpoints by present positions
      ctl_joint->pos_sp_buf.writeFromNonRT(ctl_joint->hw_state_handle.getPosition());
      // reset stateful objects
      if (ctl_joint->pos_sp_sat_handle) {
        ctl_joint->pos_sp_sat_handle->reset();
      }
      ctl_joint->pid.reset();
    }
  }

  virtual void update(const ros::Time &time, const ros::Duration &period) {
    // update state of the model
    // (zero for now. TODO: get model state from ROS message)
    model_root_joint_->setTransform(Eigen::Isometry3d::Identity());
    model_root_joint_->setLinearVelocity(Eigen::Vector3d::Zero());
    model_root_joint_->setAngularVelocity(Eigen::Vector3d::Zero());

    // update model joint states from the hardware
    BOOST_FOREACH (const ObservedJointInfoConstPtr &joint, all_joints_) {
      // set position forwarded by one time step to make control more stable
      // (recommended in https://dartsim.github.io/tutorials_manipulator.html)
      const double pos(joint->hw_state_handle.getPosition());
      const double vel(joint->hw_state_handle.getVelocity());
      model_->setPosition(joint->id_in_model, pos + vel * period.toSec());
      model_->setVelocity(joint->id_in_model, vel);
      // TODO: set acceleration to consider inertial force ??
    }

    // generate control input for each joint
    Eigen::VectorXd u(Eigen::VectorXd::Zero(model_->getNumDofs()));
    BOOST_FOREACH (const ControlledJointInfoPtr &ctl_joint, controlled_joints_) {
      // copy position setpoint from the buffer
      ctl_joint->pos_sp = *ctl_joint->pos_sp_buf.readFromRT();
      // satualate the position setpoint (optional)
      if (ctl_joint->pos_sp_sat_handle) {
        ctl_joint->pos_sp_sat_handle->enforceLimits(period);
      }
      // compute control input based on position error
      u[ctl_joint->id_in_model] = ctl_joint->pid.computeCommand(
          ctl_joint->pos_sp - ctl_joint->hw_state_handle.getPosition(), period);
    }

    // compute required torque
    const Eigen::VectorXd t(model_->getMassMatrix() * u + model_->getCoriolisAndGravityForces());

    // set torque commands
    BOOST_FOREACH (const ControlledJointInfoPtr &ctl_joint, controlled_joints_) {
      ctl_joint->hw_eff_cmd_handle.setCommand(t[ctl_joint->id_in_model]);
    }
  }

  virtual void stopping(const ros::Time &time) {
    // nothing to do
    // (or set torque commands with zero control input??)
  }

private:
  static void positionSetpointCB(const std_msgs::Float64ConstPtr &msg,
                                 rt::RealtimeBuffer< double > *const buf) {
    buf->writeFromNonRT(msg->data);
  }

private:
  dd::SkeletonPtr model_;
  dd::FreeJoint *model_root_joint_;
  std::vector< ObservedJointInfoConstPtr > all_joints_;
  std::vector< ControlledJointInfoPtr > controlled_joints_;
};
} // namespace computed_torque_controller

#endif