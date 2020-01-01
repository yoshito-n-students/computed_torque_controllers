#ifndef COMPUTED_TORQUE_CONTROLLER_COMPUTED_TORQUE_CONTROLLER_HPP
#define COMPUTED_TORQUE_CONTROLLER_COMPUTED_TORQUE_CONTROLLER_HPP

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
#include <realtime_tools/realtime_buffer.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>

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
  struct JointInfo {
    // required info for both observed/controlled joints
    dd::Joint *model_joint;
    std::size_t id_in_model;
    hi::JointStateHandle hw_state_handle;

    // optional info for controlled joints
    boost::optional< ct::Pid > pid;
    boost::optional< hi::JointHandle > hw_cmd_handle;
    // natively like optional<>
    ros::Subscriber cmd_sub;
    // use shared_ptr<> instead of optional<> to make it copy-safe
    boost::shared_ptr< rt::RealtimeBuffer< double > > cmd_buf;
  };

public:
  ComputedTorqueController() {}

  virtual ~ComputedTorqueController() {}

  // required interfaces as a Controller

  virtual bool init(hi::RobotHW *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
    namespace rn = ros::names;
    namespace rp = ros::param;

    // build a dynamics model from robot_description param
    {
      std::string urdf_str;
      if (!controller_nh.getParam("robot_description", urdf_str) &&
          !rp::get("robot_description", urdf_str)) {
        ROS_ERROR_STREAM("ComputedTorqueController::init(): Faild to get robot description from '"
                         << controller_nh.resolveName("robot_description") << "' nor '"
                         << rn::resolve("robot_description") << "'");
        return false;
      }
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
    }

    // get model root joint between root link and world, which does not exist in the hardware
    model_root_joint_ = dynamic_cast< dd::FreeJoint * >(model_->getRootJoint());
    if (!model_root_joint_) {
      ROS_ERROR(
          "ComputedTorqueController::init(): Faild to get a root joint of the dynamics model");
      return false;
    }

    // get required hardware interfaces
    // (no need to check existence of interfaces because the base class did it)
    hi::JointStateInterface *const hw_state_iface(hw->get< hi::JointStateInterface >());
    hi::EffortJointInterface *const hw_cmd_iface(hw->get< hi::EffortJointInterface >());

    // match joint in the dynamics model & hardware (except the model root joint)
    std::size_t n_controlled_joints(0);
    BOOST_FOREACH (dd::Joint *const model_joint, model_->getJoints()) {
      // skip if the model root joint
      if (model_joint == model_->getRootJoint()) {
        continue;
      }
      // assert single Dof joint
      if (model_joint->getNumDofs() != 1) {
        ROS_ERROR_STREAM(
            "ComputedTorqueController::init(): Multi-Dof joint is not supported (name: '"
            << model_joint->getName() << "', dofs: " << model_joint->getNumDofs() << ")");
        return false;
      }
      // start composing joint info
      JointInfo joint_info;
      // get the joint in the dynamics model
      joint_info.model_joint = model_joint;
      joint_info.id_in_model = model_joint->getIndexInSkeleton(/* 1st DoF */ 0);
      // hardware joint state handle
      const std::string joint_name(joint_info.model_joint->getName());
      try {
        joint_info.hw_state_handle = hw_state_iface->getHandle(joint_name);
      } catch (const hi::HardwareInterfaceException &ex) {
        ROS_ERROR_STREAM("ComputedTorqueController::init(): Failed to get the state handle of '"
                         << joint_name << "': " << ex.what());
        return false;
      }
      // optional info for a controlled joint
      const std::string joint_ns(controller_nh.resolveName(rn::append("joints", joint_name)));
      if (rp::has(joint_ns)) {
        // pid
        joint_info.pid = ct::Pid();
        if (!joint_info.pid->initParam(joint_ns)) {
          ROS_ERROR_STREAM("ComputedTorqueController::init(): Failed to init a PID by param '"
                           << joint_ns << "'");
          return false;
        }
        // hardware joint command handle
        try {
          joint_info.hw_cmd_handle = hw_cmd_iface->getHandle(joint_name);
        } catch (const hi::HardwareInterfaceException &ex) {
          ROS_ERROR_STREAM("ComputedTorqueController::init(): Failed to get the command handle of '"
                           << joint_name << "': " << ex.what());
          return false;
        }
        // command subscription
        joint_info.cmd_buf.reset(new rt::RealtimeBuffer< double >());
        joint_info.cmd_sub = controller_nh.subscribe< std_msgs::Float64 >(
            rn::append(joint_name, "command"), 1,
            boost::bind(&ComputedTorqueController::commandCB, _1, joint_info.cmd_buf));
        // increment count of controlled joints
        ++n_controlled_joints;
      }
      // store joint info
      joints_.push_back(joint_info);
    }

    // kind warning if no controlled joints
    if (n_controlled_joints == 0) {
      ROS_WARN("ComputedTorqueController::init(): No controlled joint given. Will do nothing");
    }

    return true;
  }

  virtual void starting(const ros::Time &time) {
    BOOST_FOREACH (JointInfo &joint, joints_) {
      // reset PID states
      if (joint.pid) {
        joint.pid->reset();
      }
      // reset position commands by present positions
      if (joint.cmd_buf) {
        joint.cmd_buf->writeFromNonRT(joint.hw_state_handle.getPosition());
      }
    }
  }

  virtual void update(const ros::Time &time, const ros::Duration &period) {
    // update state of the model
    // (zero for now. TODO: get model state from ROS message)
    model_root_joint_->setTransform(Eigen::Isometry3d::Identity());
    model_root_joint_->setLinearVelocity(Eigen::Vector3d::Zero());
    model_root_joint_->setAngularVelocity(Eigen::Vector3d::Zero());
    model_root_joint_->setLinearAcceleration(Eigen::Vector3d::Zero());
    model_root_joint_->setAngularAcceleration(Eigen::Vector3d::Zero());

    // update model joint states from the hardware
    BOOST_FOREACH (JointInfo &joint, joints_) {
      model_->setPosition(joint.id_in_model, joint.hw_state_handle.getPosition());
      model_->setVelocity(joint.id_in_model, joint.hw_state_handle.getVelocity());
    }

    // generate control input for each joint
    Eigen::VectorXd u(Eigen::VectorXd::Zero(model_->getNumDofs()));
    BOOST_FOREACH (JointInfo &joint, joints_) {
      if (joint.pid && joint.cmd_buf) {
        u[joint.id_in_model] = joint.pid->computeCommand(
            *joint.cmd_buf->readFromRT() - joint.hw_state_handle.getPosition(), period);
      }
    }

    // compute required torque
    const Eigen::VectorXd t(model_->getMassMatrix() * u + model_->getCoriolisAndGravityForces());

    // set torque commands
    BOOST_FOREACH (JointInfo &joint, joints_) {
      if (joint.hw_cmd_handle) {
        joint.hw_cmd_handle->setCommand(t[joint.id_in_model]);
      }
    }
  }

  virtual void stopping(const ros::Time &time) {
    // nothing to do
    // (or set torque commands with zero control input??)
  }

private:
  static void commandCB(const std_msgs::Float64ConstPtr &msg,
                        const boost::shared_ptr< rt::RealtimeBuffer< double > > &buf) {
    buf->writeFromNonRT(msg->data);
  }

private:
  dd::SkeletonPtr model_;
  dd::FreeJoint *model_root_joint_;
  std::vector< JointInfo > joints_;
};
} // namespace computed_torque_controller

#endif