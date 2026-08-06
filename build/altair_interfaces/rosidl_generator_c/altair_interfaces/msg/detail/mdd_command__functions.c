// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from altair_interfaces:msg/MddCommand.idl
// generated code does not contain a copyright notice
#include "altair_interfaces/msg/detail/mdd_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
altair_interfaces__msg__MddCommand__init(altair_interfaces__msg__MddCommand * msg)
{
  if (!msg) {
    return false;
  }
  // targets
  return true;
}

void
altair_interfaces__msg__MddCommand__fini(altair_interfaces__msg__MddCommand * msg)
{
  if (!msg) {
    return;
  }
  // targets
}

bool
altair_interfaces__msg__MddCommand__are_equal(const altair_interfaces__msg__MddCommand * lhs, const altair_interfaces__msg__MddCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // targets
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->targets[i] != rhs->targets[i]) {
      return false;
    }
  }
  return true;
}

bool
altair_interfaces__msg__MddCommand__copy(
  const altair_interfaces__msg__MddCommand * input,
  altair_interfaces__msg__MddCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // targets
  for (size_t i = 0; i < 4; ++i) {
    output->targets[i] = input->targets[i];
  }
  return true;
}

altair_interfaces__msg__MddCommand *
altair_interfaces__msg__MddCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  altair_interfaces__msg__MddCommand * msg = (altair_interfaces__msg__MddCommand *)allocator.allocate(sizeof(altair_interfaces__msg__MddCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(altair_interfaces__msg__MddCommand));
  bool success = altair_interfaces__msg__MddCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
altair_interfaces__msg__MddCommand__destroy(altair_interfaces__msg__MddCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    altair_interfaces__msg__MddCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
altair_interfaces__msg__MddCommand__Sequence__init(altair_interfaces__msg__MddCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  altair_interfaces__msg__MddCommand * data = NULL;

  if (size) {
    data = (altair_interfaces__msg__MddCommand *)allocator.zero_allocate(size, sizeof(altair_interfaces__msg__MddCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = altair_interfaces__msg__MddCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        altair_interfaces__msg__MddCommand__fini(&data[i - 1]);
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
altair_interfaces__msg__MddCommand__Sequence__fini(altair_interfaces__msg__MddCommand__Sequence * array)
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
      altair_interfaces__msg__MddCommand__fini(&array->data[i]);
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

altair_interfaces__msg__MddCommand__Sequence *
altair_interfaces__msg__MddCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  altair_interfaces__msg__MddCommand__Sequence * array = (altair_interfaces__msg__MddCommand__Sequence *)allocator.allocate(sizeof(altair_interfaces__msg__MddCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = altair_interfaces__msg__MddCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
altair_interfaces__msg__MddCommand__Sequence__destroy(altair_interfaces__msg__MddCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    altair_interfaces__msg__MddCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
altair_interfaces__msg__MddCommand__Sequence__are_equal(const altair_interfaces__msg__MddCommand__Sequence * lhs, const altair_interfaces__msg__MddCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!altair_interfaces__msg__MddCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
altair_interfaces__msg__MddCommand__Sequence__copy(
  const altair_interfaces__msg__MddCommand__Sequence * input,
  altair_interfaces__msg__MddCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(altair_interfaces__msg__MddCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    altair_interfaces__msg__MddCommand * data =
      (altair_interfaces__msg__MddCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!altair_interfaces__msg__MddCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          altair_interfaces__msg__MddCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!altair_interfaces__msg__MddCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
