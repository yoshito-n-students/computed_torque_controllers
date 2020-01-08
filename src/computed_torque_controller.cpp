#include <computed_torque_controller/position_joint_controller.hpp>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(computed_torque_controller::PositionJointController,
                       controller_interface::ControllerBase);