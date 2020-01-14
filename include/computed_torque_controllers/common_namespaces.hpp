#ifndef COMPUTED_TORQUE_CONTROLLERS_COMMON_NAMESPACES_HPP
#define COMPUTED_TORQUE_CONTROLLERS_COMMON_NAMESPACES_HPP

namespace controller_interface {}

namespace control_toolbox {}

namespace dart {
namespace common {}
namespace dynamics {}
namespace math {}
namespace utils {}
} // namespace dart

namespace hardware_interface {}

namespace joint_limits_interface {}

namespace realtime_tools {}

namespace computed_torque_controllers {
namespace ci = controller_interface;
namespace ct = control_toolbox;
namespace dc = dart::common;
namespace dd = dart::dynamics;
namespace dm = dart::math;
namespace du = dart::utils;
namespace hi = hardware_interface;
namespace jli = joint_limits_interface;
namespace rt = realtime_tools;
} // namespace computed_torque_controllers

#endif