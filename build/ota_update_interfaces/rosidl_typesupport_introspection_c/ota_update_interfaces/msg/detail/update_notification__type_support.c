// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ota_update_interfaces:msg/UpdateNotification.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ota_update_interfaces/msg/detail/update_notification__rosidl_typesupport_introspection_c.h"
#include "ota_update_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ota_update_interfaces/msg/detail/update_notification__functions.h"
#include "ota_update_interfaces/msg/detail/update_notification__struct.h"


// Include directives for member types
// Member `target`
// Member `version`
// Member `file_path`
// Member `checksum`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ota_update_interfaces__msg__UpdateNotification__rosidl_typesupport_introspection_c__UpdateNotification_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ota_update_interfaces__msg__UpdateNotification__init(message_memory);
}

void ota_update_interfaces__msg__UpdateNotification__rosidl_typesupport_introspection_c__UpdateNotification_fini_function(void * message_memory)
{
  ota_update_interfaces__msg__UpdateNotification__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ota_update_interfaces__msg__UpdateNotification__rosidl_typesupport_introspection_c__UpdateNotification_message_member_array[5] = {
  {
    "target",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ota_update_interfaces__msg__UpdateNotification, target),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "version",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ota_update_interfaces__msg__UpdateNotification, version),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "file_path",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ota_update_interfaces__msg__UpdateNotification, file_path),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "file_size",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ota_update_interfaces__msg__UpdateNotification, file_size),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "checksum",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ota_update_interfaces__msg__UpdateNotification, checksum),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ota_update_interfaces__msg__UpdateNotification__rosidl_typesupport_introspection_c__UpdateNotification_message_members = {
  "ota_update_interfaces__msg",  // message namespace
  "UpdateNotification",  // message name
  5,  // number of fields
  sizeof(ota_update_interfaces__msg__UpdateNotification),
  ota_update_interfaces__msg__UpdateNotification__rosidl_typesupport_introspection_c__UpdateNotification_message_member_array,  // message members
  ota_update_interfaces__msg__UpdateNotification__rosidl_typesupport_introspection_c__UpdateNotification_init_function,  // function to initialize message memory (memory has to be allocated)
  ota_update_interfaces__msg__UpdateNotification__rosidl_typesupport_introspection_c__UpdateNotification_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ota_update_interfaces__msg__UpdateNotification__rosidl_typesupport_introspection_c__UpdateNotification_message_type_support_handle = {
  0,
  &ota_update_interfaces__msg__UpdateNotification__rosidl_typesupport_introspection_c__UpdateNotification_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ota_update_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ota_update_interfaces, msg, UpdateNotification)() {
  if (!ota_update_interfaces__msg__UpdateNotification__rosidl_typesupport_introspection_c__UpdateNotification_message_type_support_handle.typesupport_identifier) {
    ota_update_interfaces__msg__UpdateNotification__rosidl_typesupport_introspection_c__UpdateNotification_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ota_update_interfaces__msg__UpdateNotification__rosidl_typesupport_introspection_c__UpdateNotification_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
