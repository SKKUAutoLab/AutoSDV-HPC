// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ota_update_interfaces:msg/UpdateNotification.idl
// generated code does not contain a copyright notice

#ifndef OTA_UPDATE_INTERFACES__MSG__DETAIL__UPDATE_NOTIFICATION__TRAITS_HPP_
#define OTA_UPDATE_INTERFACES__MSG__DETAIL__UPDATE_NOTIFICATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ota_update_interfaces/msg/detail/update_notification__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ota_update_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const UpdateNotification & msg,
  std::ostream & out)
{
  out << "{";
  // member: target
  {
    out << "target: ";
    rosidl_generator_traits::value_to_yaml(msg.target, out);
    out << ", ";
  }

  // member: version
  {
    out << "version: ";
    rosidl_generator_traits::value_to_yaml(msg.version, out);
    out << ", ";
  }

  // member: file_path
  {
    out << "file_path: ";
    rosidl_generator_traits::value_to_yaml(msg.file_path, out);
    out << ", ";
  }

  // member: file_size
  {
    out << "file_size: ";
    rosidl_generator_traits::value_to_yaml(msg.file_size, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UpdateNotification & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: target
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target: ";
    rosidl_generator_traits::value_to_yaml(msg.target, out);
    out << "\n";
  }

  // member: version
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "version: ";
    rosidl_generator_traits::value_to_yaml(msg.version, out);
    out << "\n";
  }

  // member: file_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "file_path: ";
    rosidl_generator_traits::value_to_yaml(msg.file_path, out);
    out << "\n";
  }

  // member: file_size
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "file_size: ";
    rosidl_generator_traits::value_to_yaml(msg.file_size, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UpdateNotification & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace ota_update_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use ota_update_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ota_update_interfaces::msg::UpdateNotification & msg,
  std::ostream & out, size_t indentation = 0)
{
  ota_update_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ota_update_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const ota_update_interfaces::msg::UpdateNotification & msg)
{
  return ota_update_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ota_update_interfaces::msg::UpdateNotification>()
{
  return "ota_update_interfaces::msg::UpdateNotification";
}

template<>
inline const char * name<ota_update_interfaces::msg::UpdateNotification>()
{
  return "ota_update_interfaces/msg/UpdateNotification";
}

template<>
struct has_fixed_size<ota_update_interfaces::msg::UpdateNotification>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ota_update_interfaces::msg::UpdateNotification>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ota_update_interfaces::msg::UpdateNotification>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // OTA_UPDATE_INTERFACES__MSG__DETAIL__UPDATE_NOTIFICATION__TRAITS_HPP_
