// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from altair_interfaces:msg/CanStatus.idl
// generated code does not contain a copyright notice
#include "altair_interfaces/msg/detail/can_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `active_port`
// Member `error_message`
#include "rosidl_runtime_c/string_functions.h"

bool
altair_interfaces__msg__CanStatus__init(altair_interfaces__msg__CanStatus * msg)
{
  if (!msg) {
    return false;
  }
  // is_connected
  // active_port
  if (!rosidl_runtime_c__String__init(&msg->active_port)) {
    altair_interfaces__msg__CanStatus__fini(msg);
    return false;
  }
  // bitrate
  // error_message
  if (!rosidl_runtime_c__String__init(&msg->error_message)) {
    altair_interfaces__msg__CanStatus__fini(msg);
    return false;
  }
  return true;
}

void
altair_interfaces__msg__CanStatus__fini(altair_interfaces__msg__CanStatus * msg)
{
  if (!msg) {
    return;
  }
  // is_connected
  // active_port
  rosidl_runtime_c__String__fini(&msg->active_port);
  // bitrate
  // error_message
  rosidl_runtime_c__String__fini(&msg->error_message);
}

bool
altair_interfaces__msg__CanStatus__are_equal(const altair_interfaces__msg__CanStatus * lhs, const altair_interfaces__msg__CanStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // is_connected
  if (lhs->is_connected != rhs->is_connected) {
    return false;
  }
  // active_port
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->active_port), &(rhs->active_port)))
  {
    return false;
  }
  // bitrate
  if (lhs->bitrate != rhs->bitrate) {
    return false;
  }
  // error_message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->error_message), &(rhs->error_message)))
  {
    return false;
  }
  return true;
}

bool
altair_interfaces__msg__CanStatus__copy(
  const altair_interfaces__msg__CanStatus * input,
  altair_interfaces__msg__CanStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // is_connected
  output->is_connected = input->is_connected;
  // active_port
  if (!rosidl_runtime_c__String__copy(
      &(input->active_port), &(output->active_port)))
  {
    return false;
  }
  // bitrate
  output->bitrate = input->bitrate;
  // error_message
  if (!rosidl_runtime_c__String__copy(
      &(input->error_message), &(output->error_message)))
  {
    return false;
  }
  return true;
}

altair_interfaces__msg__CanStatus *
altair_interfaces__msg__CanStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  altair_interfaces__msg__CanStatus * msg = (altair_interfaces__msg__CanStatus *)allocator.allocate(sizeof(altair_interfaces__msg__CanStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(altair_interfaces__msg__CanStatus));
  bool success = altair_interfaces__msg__CanStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
altair_interfaces__msg__CanStatus__destroy(altair_interfaces__msg__CanStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    altair_interfaces__msg__CanStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
altair_interfaces__msg__CanStatus__Sequence__init(altair_interfaces__msg__CanStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  altair_interfaces__msg__CanStatus * data = NULL;

  if (size) {
    data = (altair_interfaces__msg__CanStatus *)allocator.zero_allocate(size, sizeof(altair_interfaces__msg__CanStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = altair_interfaces__msg__CanStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        altair_interfaces__msg__CanStatus__fini(&data[i - 1]);
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
altair_interfaces__msg__CanStatus__Sequence__fini(altair_interfaces__msg__CanStatus__Sequence * array)
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
      altair_interfaces__msg__CanStatus__fini(&array->data[i]);
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

altair_interfaces__msg__CanStatus__Sequence *
altair_interfaces__msg__CanStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  altair_interfaces__msg__CanStatus__Sequence * array = (altair_interfaces__msg__CanStatus__Sequence *)allocator.allocate(sizeof(altair_interfaces__msg__CanStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = altair_interfaces__msg__CanStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
altair_interfaces__msg__CanStatus__Sequence__destroy(altair_interfaces__msg__CanStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    altair_interfaces__msg__CanStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
altair_interfaces__msg__CanStatus__Sequence__are_equal(const altair_interfaces__msg__CanStatus__Sequence * lhs, const altair_interfaces__msg__CanStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!altair_interfaces__msg__CanStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
altair_interfaces__msg__CanStatus__Sequence__copy(
  const altair_interfaces__msg__CanStatus__Sequence * input,
  altair_interfaces__msg__CanStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(altair_interfaces__msg__CanStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    altair_interfaces__msg__CanStatus * data =
      (altair_interfaces__msg__CanStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!altair_interfaces__msg__CanStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          altair_interfaces__msg__CanStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!altair_interfaces__msg__CanStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
