#ifndef COMPUTED_TORQUE_CONTROLLERS_UTILS_HPP
#define COMPUTED_TORQUE_CONTROLLERS_UTILS_HPP

#include <map>
#include <stdexcept>
#include <string>

#include <ros/assert.h>
#include <ros/console.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/param.h>

namespace computed_torque_controllers {

// ================================================================================
// utility functions to find a value corresponding the given key in the given map.
// returns the reference of value if found, or die.
template < typename Value >
static Value &findValue(std::map< std::string, Value > &val_map, const std::string &key) {
  const typename std::map< std::string, Value >::iterator it(val_map.find(key));
  ROS_ASSERT_MSG(it != val_map.end(), "findValue(): No value found for the key '%s'", key.c_str());
  return it->second;
}

template < typename Value >
static const Value &findValue(const std::map< std::string, Value > &val_map,
                              const std::string &key) {
  const typename std::map< std::string, Value >::const_iterator it(val_map.find(key));
  ROS_ASSERT_MSG(it != val_map.end(), "findValue(): No value found for the key '%s'", key.c_str());
  return it->second;
}

// ========================================================================
// get robot description (normally written in URDF) from ROS param server
static inline std::string getRobotDescription(const ros::NodeHandle &param_nh) {
  std::string robot_desc;
  // TODO: search cache ??
  // try loading from the private namespace
  if (param_nh.getParam("robot_description", robot_desc)) {
    return robot_desc;
  }
  // try loading from the public namespace
  if (ros::param::get("robot_description", robot_desc)) {
    return robot_desc;
  }
  // return an empty string if nothing loaded
  ROS_ERROR_STREAM("getRobotDescription(): Faild to get robot description from '"
                   << param_nh.resolveName("robot_description") << "' nor '"
                   << ros::names::resolve("robot_description") << "'");
  return std::string();
}

} // namespace computed_torque_controllers

#endif