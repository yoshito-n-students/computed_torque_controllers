#include <computed_torque_controller/computed_torque_controller.hpp>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(computed_torque_controller::ComputedTorqueController,
                       controller_interface::ControllerBase);