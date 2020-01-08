#include <computed_torque_controllers/position_joint_controller.hpp>
#include <computed_torque_controllers/velocity_joint_controller.hpp>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(computed_torque_controllers::PositionJointController,
                       controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(computed_torque_controllers::VelocityJointController,
                       controller_interface::ControllerBase);