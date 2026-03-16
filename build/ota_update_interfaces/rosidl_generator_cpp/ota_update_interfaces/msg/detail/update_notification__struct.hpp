// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ota_update_interfaces:msg/UpdateNotification.idl
// generated code does not contain a copyright notice

#ifndef OTA_UPDATE_INTERFACES__MSG__DETAIL__UPDATE_NOTIFICATION__STRUCT_HPP_
#define OTA_UPDATE_INTERFACES__MSG__DETAIL__UPDATE_NOTIFICATION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ota_update_interfaces__msg__UpdateNotification __attribute__((deprecated))
#else
# define DEPRECATED__ota_update_interfaces__msg__UpdateNotification __declspec(deprecated)
#endif

namespace ota_update_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UpdateNotification_
{
  using Type = UpdateNotification_<ContainerAllocator>;

  explicit UpdateNotification_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target = "";
      this->version = "";
      this->file_path = "";
      this->file_size = 0ull;
    }
  }

  explicit UpdateNotification_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target(_alloc),
    version(_alloc),
    file_path(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target = "";
      this->version = "";
      this->file_path = "";
      this->file_size = 0ull;
    }
  }

  // field types and members
  using _target_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _target_type target;
  using _version_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _version_type version;
  using _file_path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _file_path_type file_path;
  using _file_size_type =
    uint64_t;
  _file_size_type file_size;

  // setters for named parameter idiom
  Type & set__target(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->target = _arg;
    return *this;
  }
  Type & set__version(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->version = _arg;
    return *this;
  }
  Type & set__file_path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->file_path = _arg;
    return *this;
  }
  Type & set__file_size(
    const uint64_t & _arg)
  {
    this->file_size = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ota_update_interfaces::msg::UpdateNotification_<ContainerAllocator> *;
  using ConstRawPtr =
    const ota_update_interfaces::msg::UpdateNotification_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ota_update_interfaces::msg::UpdateNotification_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ota_update_interfaces::msg::UpdateNotification_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ota_update_interfaces::msg::UpdateNotification_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ota_update_interfaces::msg::UpdateNotification_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ota_update_interfaces::msg::UpdateNotification_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ota_update_interfaces::msg::UpdateNotification_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ota_update_interfaces::msg::UpdateNotification_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ota_update_interfaces::msg::UpdateNotification_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ota_update_interfaces__msg__UpdateNotification
    std::shared_ptr<ota_update_interfaces::msg::UpdateNotification_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ota_update_interfaces__msg__UpdateNotification
    std::shared_ptr<ota_update_interfaces::msg::UpdateNotification_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UpdateNotification_ & other) const
  {
    if (this->target != other.target) {
      return false;
    }
    if (this->version != other.version) {
      return false;
    }
    if (this->file_path != other.file_path) {
      return false;
    }
    if (this->file_size != other.file_size) {
      return false;
    }
    return true;
  }
  bool operator!=(const UpdateNotification_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UpdateNotification_

// alias to use template instance with default allocator
using UpdateNotification =
  ota_update_interfaces::msg::UpdateNotification_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ota_update_interfaces

#endif  // OTA_UPDATE_INTERFACES__MSG__DETAIL__UPDATE_NOTIFICATION__STRUCT_HPP_
