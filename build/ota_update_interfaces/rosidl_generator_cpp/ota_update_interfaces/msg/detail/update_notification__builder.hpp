// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ota_update_interfaces:msg/UpdateNotification.idl
// generated code does not contain a copyright notice

#ifndef OTA_UPDATE_INTERFACES__MSG__DETAIL__UPDATE_NOTIFICATION__BUILDER_HPP_
#define OTA_UPDATE_INTERFACES__MSG__DETAIL__UPDATE_NOTIFICATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ota_update_interfaces/msg/detail/update_notification__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ota_update_interfaces
{

namespace msg
{

namespace builder
{

class Init_UpdateNotification_checksum
{
public:
  explicit Init_UpdateNotification_checksum(::ota_update_interfaces::msg::UpdateNotification & msg)
  : msg_(msg)
  {}
  ::ota_update_interfaces::msg::UpdateNotification checksum(::ota_update_interfaces::msg::UpdateNotification::_checksum_type arg)
  {
    msg_.checksum = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ota_update_interfaces::msg::UpdateNotification msg_;
};

class Init_UpdateNotification_file_size
{
public:
  explicit Init_UpdateNotification_file_size(::ota_update_interfaces::msg::UpdateNotification & msg)
  : msg_(msg)
  {}
  Init_UpdateNotification_checksum file_size(::ota_update_interfaces::msg::UpdateNotification::_file_size_type arg)
  {
    msg_.file_size = std::move(arg);
    return Init_UpdateNotification_checksum(msg_);
  }

private:
  ::ota_update_interfaces::msg::UpdateNotification msg_;
};

class Init_UpdateNotification_file_path
{
public:
  explicit Init_UpdateNotification_file_path(::ota_update_interfaces::msg::UpdateNotification & msg)
  : msg_(msg)
  {}
  Init_UpdateNotification_file_size file_path(::ota_update_interfaces::msg::UpdateNotification::_file_path_type arg)
  {
    msg_.file_path = std::move(arg);
    return Init_UpdateNotification_file_size(msg_);
  }

private:
  ::ota_update_interfaces::msg::UpdateNotification msg_;
};

class Init_UpdateNotification_version
{
public:
  explicit Init_UpdateNotification_version(::ota_update_interfaces::msg::UpdateNotification & msg)
  : msg_(msg)
  {}
  Init_UpdateNotification_file_path version(::ota_update_interfaces::msg::UpdateNotification::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_UpdateNotification_file_path(msg_);
  }

private:
  ::ota_update_interfaces::msg::UpdateNotification msg_;
};

class Init_UpdateNotification_target
{
public:
  Init_UpdateNotification_target()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UpdateNotification_version target(::ota_update_interfaces::msg::UpdateNotification::_target_type arg)
  {
    msg_.target = std::move(arg);
    return Init_UpdateNotification_version(msg_);
  }

private:
  ::ota_update_interfaces::msg::UpdateNotification msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ota_update_interfaces::msg::UpdateNotification>()
{
  return ota_update_interfaces::msg::builder::Init_UpdateNotification_target();
}

}  // namespace ota_update_interfaces

#endif  // OTA_UPDATE_INTERFACES__MSG__DETAIL__UPDATE_NOTIFICATION__BUILDER_HPP_
