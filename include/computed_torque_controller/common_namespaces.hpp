#ifndef COMPUTED_TORQUE_CONTROLLER_COMMON_NAMESPACES_HPP
#define COMPUTED_TORQUE_CONTROLLER_COMMON_NAMESPACES_HPP

namespace controller_interface {}

namespace control_toolbox {}

namespace dart {
namespace common {}
namespace dynamics {}
namespace utils {}
} // namespace dart

namespace hardware_interface {}

namespace joint_limits_interface {}

namespace realtime_tools {}

namespace computed_torque_controller {
namespace ci = controller_interface;
namespace ct = control_toolbox;
namespace dc = dart::common;
namespace dd = dart::dynamics;
namespace du = dart::utils;
namespace hi = hardware_interface;
namespace jli = joint_limits_interface;
namespace rt = realtime_tools;
} // namespace computed_torque_controller

#endif