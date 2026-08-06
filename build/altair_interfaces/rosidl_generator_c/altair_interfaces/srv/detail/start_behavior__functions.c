// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from altair_interfaces:srv/StartBehavior.idl
// generated code does not contain a copyright notice
#include "altair_interfaces/srv/detail/start_behavior__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `behavior_name`
#include "rosidl_runtime_c/string_functions.h"

bool
altair_interfaces__srv__StartBehavior_Request__init(altair_interfaces__srv__StartBehavior_Request * msg)
{
  if (!msg) {
    return false;
  }
  // behavior_name
  if (!rosidl_runtime_c__String__init(&msg->behavior_name)) {
    altair_interfaces__srv__StartBehavior_Request__fini(msg);
    return false;
  }
  return true;
}

void
altair_interfaces__srv__StartBehavior_Request__fini(altair_interfaces__srv__StartBehavior_Request * msg)
{
  if (!msg) {
    return;
  }
  // behavior_name
  rosidl_runtime_c__String__fini(&msg->behavior_name);
}

bool
altair_interfaces__srv__StartBehavior_Request__are_equal(const altair_interfaces__srv__StartBehavior_Request * lhs, const altair_interfaces__srv__StartBehavior_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // behavior_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->behavior_name), &(rhs->behavior_name)))
  {
    return false;
  }
  return true;
}

bool
altair_interfaces__srv__StartBehavior_Request__copy(
  const altair_interfaces__srv__StartBehavior_Request * input,
  altair_interfaces__srv__StartBehavior_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // behavior_name
  if (!rosidl_runtime_c__String__copy(
      &(input->behavior_name), &(output->behavior_name)))
  {
    return false;
  }
  return true;
}

altair_interfaces__srv__StartBehavior_Request *
altair_interfaces__srv__StartBehavior_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  altair_interfaces__srv__StartBehavior_Request * msg = (altair_interfaces__srv__StartBehavior_Request *)allocator.allocate(sizeof(altair_interfaces__srv__StartBehavior_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(altair_interfaces__srv__StartBehavior_Request));
  bool success = altair_interfaces__srv__StartBehavior_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
altair_interfaces__srv__StartBehavior_Request__destroy(altair_interfaces__srv__StartBehavior_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    altair_interfaces__srv__StartBehavior_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
altair_interfaces__srv__StartBehavior_Request__Sequence__init(altair_interfaces__srv__StartBehavior_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  altair_interfaces__srv__StartBehavior_Request * data = NULL;

  if (size) {
    data = (altair_interfaces__srv__StartBehavior_Request *)allocator.zero_allocate(size, sizeof(altair_interfaces__srv__StartBehavior_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = altair_interfaces__srv__StartBehavior_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        altair_interfaces__srv__StartBehavior_Request__fini(&data[i - 1]);
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
altair_interfaces__srv__StartBehavior_Request__Sequence__fini(altair_interfaces__srv__StartBehavior_Request__Sequence * array)
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
      altair_interfaces__srv__StartBehavior_Request__fini(&array->data[i]);
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

altair_interfaces__srv__StartBehavior_Request__Sequence *
altair_interfaces__srv__StartBehavior_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  altair_interfaces__srv__StartBehavior_Request__Sequence * array = (altair_interfaces__srv__StartBehavior_Request__Sequence *)allocator.allocate(sizeof(altair_interfaces__srv__StartBehavior_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = altair_interfaces__srv__StartBehavior_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
altair_interfaces__srv__StartBehavior_Request__Sequence__destroy(altair_interfaces__srv__StartBehavior_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    altair_interfaces__srv__StartBehavior_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
altair_interfaces__srv__StartBehavior_Request__Sequence__are_equal(const altair_interfaces__srv__StartBehavior_Request__Sequence * lhs, const altair_interfaces__srv__StartBehavior_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!altair_interfaces__srv__StartBehavior_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
altair_interfaces__srv__StartBehavior_Request__Sequence__copy(
  const altair_interfaces__srv__StartBehavior_Request__Sequence * input,
  altair_interfaces__srv__StartBehavior_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(altair_interfaces__srv__StartBehavior_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    altair_interfaces__srv__StartBehavior_Request * data =
      (altair_interfaces__srv__StartBehavior_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!altair_interfaces__srv__StartBehavior_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          altair_interfaces__srv__StartBehavior_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!altair_interfaces__srv__StartBehavior_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
altair_interfaces__srv__StartBehavior_Response__init(altair_interfaces__srv__StartBehavior_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    altair_interfaces__srv__StartBehavior_Response__fini(msg);
    return false;
  }
  return true;
}

void
altair_interfaces__srv__StartBehavior_Response__fini(altair_interfaces__srv__StartBehavior_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
altair_interfaces__srv__StartBehavior_Response__are_equal(const altair_interfaces__srv__StartBehavior_Response * lhs, const altair_interfaces__srv__StartBehavior_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
altair_interfaces__srv__StartBehavior_Response__copy(
  const altair_interfaces__srv__StartBehavior_Response * input,
  altair_interfaces__srv__StartBehavior_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

altair_interfaces__srv__StartBehavior_Response *
altair_interfaces__srv__StartBehavior_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  altair_interfaces__srv__StartBehavior_Response * msg = (altair_interfaces__srv__StartBehavior_Response *)allocator.allocate(sizeof(altair_interfaces__srv__StartBehavior_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(altair_interfaces__srv__StartBehavior_Response));
  bool success = altair_interfaces__srv__StartBehavior_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
altair_interfaces__srv__StartBehavior_Response__destroy(altair_interfaces__srv__StartBehavior_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    altair_interfaces__srv__StartBehavior_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
altair_interfaces__srv__StartBehavior_Response__Sequence__init(altair_interfaces__srv__StartBehavior_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  altair_interfaces__srv__StartBehavior_Response * data = NULL;

  if (size) {
    data = (altair_interfaces__srv__StartBehavior_Response *)allocator.zero_allocate(size, sizeof(altair_interfaces__srv__StartBehavior_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = altair_interfaces__srv__StartBehavior_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        altair_interfaces__srv__StartBehavior_Response__fini(&data[i - 1]);
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
altair_interfaces__srv__StartBehavior_Response__Sequence__fini(altair_interfaces__srv__StartBehavior_Response__Sequence * array)
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
      altair_interfaces__srv__StartBehavior_Response__fini(&array->data[i]);
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

altair_interfaces__srv__StartBehavior_Response__Sequence *
altair_interfaces__srv__StartBehavior_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  altair_interfaces__srv__StartBehavior_Response__Sequence * array = (altair_interfaces__srv__StartBehavior_Response__Sequence *)allocator.allocate(sizeof(altair_interfaces__srv__StartBehavior_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = altair_interfaces__srv__StartBehavior_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
altair_interfaces__srv__StartBehavior_Response__Sequence__destroy(altair_interfaces__srv__StartBehavior_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    altair_interfaces__srv__StartBehavior_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
altair_interfaces__srv__StartBehavior_Response__Sequence__are_equal(const altair_interfaces__srv__StartBehavior_Response__Sequence * lhs, const altair_interfaces__srv__StartBehavior_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!altair_interfaces__srv__StartBehavior_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
altair_interfaces__srv__StartBehavior_Response__Sequence__copy(
  const altair_interfaces__srv__StartBehavior_Response__Sequence * input,
  altair_interfaces__srv__StartBehavior_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(altair_interfaces__srv__StartBehavior_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    altair_interfaces__srv__StartBehavior_Response * data =
      (altair_interfaces__srv__StartBehavior_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!altair_interfaces__srv__StartBehavior_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          altair_interfaces__srv__StartBehavior_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!altair_interfaces__srv__StartBehavior_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
