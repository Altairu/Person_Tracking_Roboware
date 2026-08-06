// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from altair_interfaces:msg/MddFeedback.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__MSG__DETAIL__MDD_FEEDBACK__FUNCTIONS_H_
#define ALTAIR_INTERFACES__MSG__DETAIL__MDD_FEEDBACK__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "altair_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "altair_interfaces/msg/detail/mdd_feedback__struct.h"

/// Initialize msg/MddFeedback message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * altair_interfaces__msg__MddFeedback
 * )) before or use
 * altair_interfaces__msg__MddFeedback__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
bool
altair_interfaces__msg__MddFeedback__init(altair_interfaces__msg__MddFeedback * msg);

/// Finalize msg/MddFeedback message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
void
altair_interfaces__msg__MddFeedback__fini(altair_interfaces__msg__MddFeedback * msg);

/// Create msg/MddFeedback message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * altair_interfaces__msg__MddFeedback__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
altair_interfaces__msg__MddFeedback *
altair_interfaces__msg__MddFeedback__create();

/// Destroy msg/MddFeedback message.
/**
 * It calls
 * altair_interfaces__msg__MddFeedback__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
void
altair_interfaces__msg__MddFeedback__destroy(altair_interfaces__msg__MddFeedback * msg);

/// Check for msg/MddFeedback message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
bool
altair_interfaces__msg__MddFeedback__are_equal(const altair_interfaces__msg__MddFeedback * lhs, const altair_interfaces__msg__MddFeedback * rhs);

/// Copy a msg/MddFeedback message.
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
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
bool
altair_interfaces__msg__MddFeedback__copy(
  const altair_interfaces__msg__MddFeedback * input,
  altair_interfaces__msg__MddFeedback * output);

/// Initialize array of msg/MddFeedback messages.
/**
 * It allocates the memory for the number of elements and calls
 * altair_interfaces__msg__MddFeedback__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
bool
altair_interfaces__msg__MddFeedback__Sequence__init(altair_interfaces__msg__MddFeedback__Sequence * array, size_t size);

/// Finalize array of msg/MddFeedback messages.
/**
 * It calls
 * altair_interfaces__msg__MddFeedback__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
void
altair_interfaces__msg__MddFeedback__Sequence__fini(altair_interfaces__msg__MddFeedback__Sequence * array);

/// Create array of msg/MddFeedback messages.
/**
 * It allocates the memory for the array and calls
 * altair_interfaces__msg__MddFeedback__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
altair_interfaces__msg__MddFeedback__Sequence *
altair_interfaces__msg__MddFeedback__Sequence__create(size_t size);

/// Destroy array of msg/MddFeedback messages.
/**
 * It calls
 * altair_interfaces__msg__MddFeedback__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
void
altair_interfaces__msg__MddFeedback__Sequence__destroy(altair_interfaces__msg__MddFeedback__Sequence * array);

/// Check for msg/MddFeedback message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
bool
altair_interfaces__msg__MddFeedback__Sequence__are_equal(const altair_interfaces__msg__MddFeedback__Sequence * lhs, const altair_interfaces__msg__MddFeedback__Sequence * rhs);

/// Copy an array of msg/MddFeedback messages.
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
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
bool
altair_interfaces__msg__MddFeedback__Sequence__copy(
  const altair_interfaces__msg__MddFeedback__Sequence * input,
  altair_interfaces__msg__MddFeedback__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ALTAIR_INTERFACES__MSG__DETAIL__MDD_FEEDBACK__FUNCTIONS_H_
