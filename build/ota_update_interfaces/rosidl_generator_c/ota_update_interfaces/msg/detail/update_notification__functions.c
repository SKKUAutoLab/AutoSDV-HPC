// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ota_update_interfaces:msg/UpdateNotification.idl
// generated code does not contain a copyright notice
#include "ota_update_interfaces/msg/detail/update_notification__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `target`
// Member `version`
// Member `file_path`
#include "rosidl_runtime_c/string_functions.h"

bool
ota_update_interfaces__msg__UpdateNotification__init(ota_update_interfaces__msg__UpdateNotification * msg)
{
  if (!msg) {
    return false;
  }
  // target
  if (!rosidl_runtime_c__String__init(&msg->target)) {
    ota_update_interfaces__msg__UpdateNotification__fini(msg);
    return false;
  }
  // version
  if (!rosidl_runtime_c__String__init(&msg->version)) {
    ota_update_interfaces__msg__UpdateNotification__fini(msg);
    return false;
  }
  // file_path
  if (!rosidl_runtime_c__String__init(&msg->file_path)) {
    ota_update_interfaces__msg__UpdateNotification__fini(msg);
    return false;
  }
  // file_size
  return true;
}

void
ota_update_interfaces__msg__UpdateNotification__fini(ota_update_interfaces__msg__UpdateNotification * msg)
{
  if (!msg) {
    return;
  }
  // target
  rosidl_runtime_c__String__fini(&msg->target);
  // version
  rosidl_runtime_c__String__fini(&msg->version);
  // file_path
  rosidl_runtime_c__String__fini(&msg->file_path);
  // file_size
}

bool
ota_update_interfaces__msg__UpdateNotification__are_equal(const ota_update_interfaces__msg__UpdateNotification * lhs, const ota_update_interfaces__msg__UpdateNotification * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // target
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->target), &(rhs->target)))
  {
    return false;
  }
  // version
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->version), &(rhs->version)))
  {
    return false;
  }
  // file_path
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->file_path), &(rhs->file_path)))
  {
    return false;
  }
  // file_size
  if (lhs->file_size != rhs->file_size) {
    return false;
  }
  return true;
}

bool
ota_update_interfaces__msg__UpdateNotification__copy(
  const ota_update_interfaces__msg__UpdateNotification * input,
  ota_update_interfaces__msg__UpdateNotification * output)
{
  if (!input || !output) {
    return false;
  }
  // target
  if (!rosidl_runtime_c__String__copy(
      &(input->target), &(output->target)))
  {
    return false;
  }
  // version
  if (!rosidl_runtime_c__String__copy(
      &(input->version), &(output->version)))
  {
    return false;
  }
  // file_path
  if (!rosidl_runtime_c__String__copy(
      &(input->file_path), &(output->file_path)))
  {
    return false;
  }
  // file_size
  output->file_size = input->file_size;
  return true;
}

ota_update_interfaces__msg__UpdateNotification *
ota_update_interfaces__msg__UpdateNotification__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ota_update_interfaces__msg__UpdateNotification * msg = (ota_update_interfaces__msg__UpdateNotification *)allocator.allocate(sizeof(ota_update_interfaces__msg__UpdateNotification), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ota_update_interfaces__msg__UpdateNotification));
  bool success = ota_update_interfaces__msg__UpdateNotification__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ota_update_interfaces__msg__UpdateNotification__destroy(ota_update_interfaces__msg__UpdateNotification * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ota_update_interfaces__msg__UpdateNotification__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ota_update_interfaces__msg__UpdateNotification__Sequence__init(ota_update_interfaces__msg__UpdateNotification__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ota_update_interfaces__msg__UpdateNotification * data = NULL;

  if (size) {
    data = (ota_update_interfaces__msg__UpdateNotification *)allocator.zero_allocate(size, sizeof(ota_update_interfaces__msg__UpdateNotification), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ota_update_interfaces__msg__UpdateNotification__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ota_update_interfaces__msg__UpdateNotification__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ota_update_interfaces__msg__UpdateNotification__Sequence__fini(ota_update_interfaces__msg__UpdateNotification__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ota_update_interfaces__msg__UpdateNotification__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ota_update_interfaces__msg__UpdateNotification__Sequence *
ota_update_interfaces__msg__UpdateNotification__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ota_update_interfaces__msg__UpdateNotification__Sequence * array = (ota_update_interfaces__msg__UpdateNotification__Sequence *)allocator.allocate(sizeof(ota_update_interfaces__msg__UpdateNotification__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ota_update_interfaces__msg__UpdateNotification__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ota_update_interfaces__msg__UpdateNotification__Sequence__destroy(ota_update_interfaces__msg__UpdateNotification__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ota_update_interfaces__msg__UpdateNotification__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ota_update_interfaces__msg__UpdateNotification__Sequence__are_equal(const ota_update_interfaces__msg__UpdateNotification__Sequence * lhs, const ota_update_interfaces__msg__UpdateNotification__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ota_update_interfaces__msg__UpdateNotification__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ota_update_interfaces__msg__UpdateNotification__Sequence__copy(
  const ota_update_interfaces__msg__UpdateNotification__Sequence * input,
  ota_update_interfaces__msg__UpdateNotification__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ota_update_interfaces__msg__UpdateNotification);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ota_update_interfaces__msg__UpdateNotification * data =
      (ota_update_interfaces__msg__UpdateNotification *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ota_update_interfaces__msg__UpdateNotification__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ota_update_interfaces__msg__UpdateNotification__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ota_update_interfaces__msg__UpdateNotification__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
