#ifndef COMPUTED_TORQUE_CONTROLLER_ROS_PACKAGE_RESOURCE_RETRIEVER_HPP
#define COMPUTED_TORQUE_CONTROLLER_ROS_PACKAGE_RESOURCE_RETRIEVER_HPP

#include <string>

#include <computed_torque_controller/common_namespaces.hpp>
#include <ros/console.h>
#include <ros/package.h>

#include <dart/common/LocalResourceRetriever.hpp>

namespace computed_torque_controller {

// Resolver for URIs in URDF files. Currently supports file or package scheme URIs.
// (ex. "file:/path" or "package://my_robot_description/meshes/body.dae")
class ROSPackageResourceRetriever : public dc::LocalResourceRetriever {
public:
  ROSPackageResourceRetriever() {}

  virtual ~ROSPackageResourceRetriever() {}

  // required interfaces as a ResourceRetriever

  virtual bool exists(const dc::Uri &uri) {
    return dc::LocalResourceRetriever::exists(toFileUri(uri));
  }

  virtual dc::ResourcePtr retrieve(const dc::Uri &uri) {
    return dc::LocalResourceRetriever::retrieve(toFileUri(uri));
  }

  virtual std::string readAll(const dc::Uri &uri) {
    return dc::LocalResourceRetriever::readAll(toFileUri(uri));
  }

  virtual std::string getFilePath(const dc::Uri &uri) {
    return dc::LocalResourceRetriever::getFilePath(toFileUri(uri));
  }

  // helper function

  // converts the given URI to a file scheme URI.
  //   1. given a valid file URI, returns ifself
  //   2. given a valid package URI, convert it to a file URI by resolving the (ROS) package
  //   3. given any other URI, returns an empty URI
  static dc::Uri toFileUri(const dc::Uri &src_uri) {
    // handle the case 3
    if ( // invalid URI
        (!src_uri.mScheme || !src_uri.mPath)
        // package scheme URI with no package name
        || (src_uri.mScheme.get() == "package" && !src_uri.mAuthority)
        // unsupported scheme URI
        || (src_uri.mScheme.get() != "file" && src_uri.mScheme.get() != "package")) {
      ROS_ERROR_STREAM("Invalid or unsupported URI: " << src_uri.toString());
      return dc::Uri();
    }

    // handle the case 1
    if (src_uri.mScheme.get() == "file") {
      return src_uri;
    }

    // resolve the package path
    const std::string package_path(ros::package::getPath(src_uri.mAuthority.get()));
    // handle the case 3; unresolved package name
    if (package_path.empty()) {
      ROS_ERROR_STREAM("Unresolved package name in URI: " << src_uri.toString());
      return dc::Uri();
    }

    // handle the case 2
    dc::Uri dst_uri;
    dst_uri.mScheme = "file";
    dst_uri.mPath = package_path + src_uri.mPath.get();
    // ROS_INFO_STREAM(src_uri.toString() << " -> " << dst_uri.toString());
    return dst_uri;
  }
};

} // namespace computed_torque_controller

#endif