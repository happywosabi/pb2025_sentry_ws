#ifndef AUTO_AIM__PATH_UTILS_HPP
#define AUTO_AIM__PATH_UTILS_HPP

#include <string>

// Conditional compilation: only use ament_index_cpp if available
#if __has_include(<ament_index_cpp/get_package_share_directory.hpp>)
#include <ament_index_cpp/get_package_share_directory.hpp>
#define HAS_AMENT_INDEX 1
#else
#define HAS_AMENT_INDEX 0
#endif

namespace auto_aim
{

/**
 * @brief Resolve a relative path to absolute path within the sp_vision_25 package
 *
 * If the input path is already absolute (starts with '/'), returns it unchanged.
 * Otherwise, prepends the package share directory path (if ROS2 is available).
 *
 * @param relative_path  Path relative to package share directory (e.g., "assets/model.onnx")
 * @return Absolute path (e.g., "/install/sp_vision_25/share/sp_vision_25/assets/model.onnx")
 *         or original path if ROS2 is not available
 */
inline std::string resolve_package_path(const std::string & relative_path)
{
  // If already absolute, return as-is
  if (!relative_path.empty() && relative_path[0] == '/') {
    return relative_path;
  }

#if HAS_AMENT_INDEX
  // If building with ROS2, prepend package share directory
  try {
    std::string pkg_share = ament_index_cpp::get_package_share_directory("sp_vision_25");
    return pkg_share + "/" + relative_path;
  } catch (const std::exception & e) {
    // Fallback: return original path (might work if running from source directory)
    return relative_path;
  }
#else
  // If not building with ROS2, return original path
  // (assumes running from source directory where relative paths work)
  return relative_path;
#endif
}

}  // namespace auto_aim

#endif  // AUTO_AIM__PATH_UTILS_HPP
