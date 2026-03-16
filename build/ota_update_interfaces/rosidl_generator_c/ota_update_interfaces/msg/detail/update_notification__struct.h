// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ota_update_interfaces:msg/UpdateNotification.idl
// generated code does not contain a copyright notice

#ifndef OTA_UPDATE_INTERFACES__MSG__DETAIL__UPDATE_NOTIFICATION__STRUCT_H_
#define OTA_UPDATE_INTERFACES__MSG__DETAIL__UPDATE_NOTIFICATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'target'
// Member 'version'
// Member 'file_path'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/UpdateNotification in the package ota_update_interfaces.
typedef struct ota_update_interfaces__msg__UpdateNotification
{
  rosidl_runtime_c__String target;
  rosidl_runtime_c__String version;
  rosidl_runtime_c__String file_path;
  uint64_t file_size;
} ota_update_interfaces__msg__UpdateNotification;

// Struct for a sequence of ota_update_interfaces__msg__UpdateNotification.
typedef struct ota_update_interfaces__msg__UpdateNotification__Sequence
{
  ota_update_interfaces__msg__UpdateNotification * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ota_update_interfaces__msg__UpdateNotification__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // OTA_UPDATE_INTERFACES__MSG__DETAIL__UPDATE_NOTIFICATION__STRUCT_H_
