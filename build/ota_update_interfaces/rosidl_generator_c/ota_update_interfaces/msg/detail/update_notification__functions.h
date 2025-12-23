// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ota_update_interfaces:msg/UpdateNotification.idl
// generated code does not contain a copyright notice

#ifndef OTA_UPDATE_INTERFACES__MSG__DETAIL__UPDATE_NOTIFICATION__FUNCTIONS_H_
#define OTA_UPDATE_INTERFACES__MSG__DETAIL__UPDATE_NOTIFICATION__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ota_update_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "ota_update_interfaces/msg/detail/update_notification__struct.h"

/// Initialize msg/UpdateNotification message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ota_update_interfaces__msg__UpdateNotification
 * )) before or use
 * ota_update_interfaces__msg__UpdateNotification__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ota_update_interfaces
bool
ota_update_interfaces__msg__UpdateNotification__init(ota_update_interfaces__msg__UpdateNotification * msg);

/// Finalize msg/UpdateNotification message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ota_update_interfaces
void
ota_update_interfaces__msg__UpdateNotification__fini(ota_update_interfaces__msg__UpdateNotification * msg);

/// Create msg/UpdateNotification message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ota_update_interfaces__msg__UpdateNotification__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ota_update_interfaces
ota_update_interfaces__msg__UpdateNotification *
ota_update_interfaces__msg__UpdateNotification__create();

/// Destroy msg/UpdateNotification message.
/**
 * It calls
 * ota_update_interfaces__msg__UpdateNotification__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ota_update_interfaces
void
ota_update_interfaces__msg__UpdateNotification__destroy(ota_update_interfaces__msg__UpdateNotification * msg);

/// Check for msg/UpdateNotification message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ota_update_interfaces
bool
ota_update_interfaces__msg__UpdateNotification__are_equal(const ota_update_interfaces__msg__UpdateNotification * lhs, const ota_update_interfaces__msg__UpdateNotification * rhs);

/// Copy a msg/UpdateNotification message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ota_update_interfaces
bool
ota_update_interfaces__msg__UpdateNotification__copy(
  const ota_update_interfaces__msg__UpdateNotification * input,
  ota_update_interfaces__msg__UpdateNotification * output);

/// Initialize array of msg/UpdateNotification messages.
/**
 * It allocates the memory for the number of elements and calls
 * ota_update_interfaces__msg__UpdateNotification__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ota_update_interfaces
bool
ota_update_interfaces__msg__UpdateNotification__Sequence__init(ota_update_interfaces__msg__UpdateNotification__Sequence * array, size_t size);

/// Finalize array of msg/UpdateNotification messages.
/**
 * It calls
 * ota_update_interfaces__msg__UpdateNotification__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ota_update_interfaces
void
ota_update_interfaces__msg__UpdateNotification__Sequence__fini(ota_update_interfaces__msg__UpdateNotification__Sequence * array);

/// Create array of msg/UpdateNotification messages.
/**
 * It allocates the memory for the array and calls
 * ota_update_interfaces__msg__UpdateNotification__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ota_update_interfaces
ota_update_interfaces__msg__UpdateNotification__Sequence *
ota_update_interfaces__msg__UpdateNotification__Sequence__create(size_t size);

/// Destroy array of msg/UpdateNotification messages.
/**
 * It calls
 * ota_update_interfaces__msg__UpdateNotification__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ota_update_interfaces
void
ota_update_interfaces__msg__UpdateNotification__Sequence__destroy(ota_update_interfaces__msg__UpdateNotification__Sequence * array);

/// Check for msg/UpdateNotification message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ota_update_interfaces
bool
ota_update_interfaces__msg__UpdateNotification__Sequence__are_equal(const ota_update_interfaces__msg__UpdateNotification__Sequence * lhs, const ota_update_interfaces__msg__UpdateNotification__Sequence * rhs);

/// Copy an array of msg/UpdateNotification messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ota_update_interfaces
bool
ota_update_interfaces__msg__UpdateNotification__Sequence__copy(
  const ota_update_interfaces__msg__UpdateNotification__Sequence * input,
  ota_update_interfaces__msg__UpdateNotification__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // OTA_UPDATE_INTERFACES__MSG__DETAIL__UPDATE_NOTIFICATION__FUNCTIONS_H_
