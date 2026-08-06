// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from altair_interfaces:srv/ConnectCan.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__SRV__DETAIL__CONNECT_CAN__FUNCTIONS_H_
#define ALTAIR_INTERFACES__SRV__DETAIL__CONNECT_CAN__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "altair_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "altair_interfaces/srv/detail/connect_can__struct.h"

/// Initialize srv/ConnectCan message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * altair_interfaces__srv__ConnectCan_Request
 * )) before or use
 * altair_interfaces__srv__ConnectCan_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
bool
altair_interfaces__srv__ConnectCan_Request__init(altair_interfaces__srv__ConnectCan_Request * msg);

/// Finalize srv/ConnectCan message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
void
altair_interfaces__srv__ConnectCan_Request__fini(altair_interfaces__srv__ConnectCan_Request * msg);

/// Create srv/ConnectCan message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * altair_interfaces__srv__ConnectCan_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
altair_interfaces__srv__ConnectCan_Request *
altair_interfaces__srv__ConnectCan_Request__create();

/// Destroy srv/ConnectCan message.
/**
 * It calls
 * altair_interfaces__srv__ConnectCan_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
void
altair_interfaces__srv__ConnectCan_Request__destroy(altair_interfaces__srv__ConnectCan_Request * msg);

/// Check for srv/ConnectCan message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
bool
altair_interfaces__srv__ConnectCan_Request__are_equal(const altair_interfaces__srv__ConnectCan_Request * lhs, const altair_interfaces__srv__ConnectCan_Request * rhs);

/// Copy a srv/ConnectCan message.
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
altair_interfaces__srv__ConnectCan_Request__copy(
  const altair_interfaces__srv__ConnectCan_Request * input,
  altair_interfaces__srv__ConnectCan_Request * output);

/// Initialize array of srv/ConnectCan messages.
/**
 * It allocates the memory for the number of elements and calls
 * altair_interfaces__srv__ConnectCan_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
bool
altair_interfaces__srv__ConnectCan_Request__Sequence__init(altair_interfaces__srv__ConnectCan_Request__Sequence * array, size_t size);

/// Finalize array of srv/ConnectCan messages.
/**
 * It calls
 * altair_interfaces__srv__ConnectCan_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
void
altair_interfaces__srv__ConnectCan_Request__Sequence__fini(altair_interfaces__srv__ConnectCan_Request__Sequence * array);

/// Create array of srv/ConnectCan messages.
/**
 * It allocates the memory for the array and calls
 * altair_interfaces__srv__ConnectCan_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
altair_interfaces__srv__ConnectCan_Request__Sequence *
altair_interfaces__srv__ConnectCan_Request__Sequence__create(size_t size);

/// Destroy array of srv/ConnectCan messages.
/**
 * It calls
 * altair_interfaces__srv__ConnectCan_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
void
altair_interfaces__srv__ConnectCan_Request__Sequence__destroy(altair_interfaces__srv__ConnectCan_Request__Sequence * array);

/// Check for srv/ConnectCan message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
bool
altair_interfaces__srv__ConnectCan_Request__Sequence__are_equal(const altair_interfaces__srv__ConnectCan_Request__Sequence * lhs, const altair_interfaces__srv__ConnectCan_Request__Sequence * rhs);

/// Copy an array of srv/ConnectCan messages.
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
altair_interfaces__srv__ConnectCan_Request__Sequence__copy(
  const altair_interfaces__srv__ConnectCan_Request__Sequence * input,
  altair_interfaces__srv__ConnectCan_Request__Sequence * output);

/// Initialize srv/ConnectCan message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * altair_interfaces__srv__ConnectCan_Response
 * )) before or use
 * altair_interfaces__srv__ConnectCan_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
bool
altair_interfaces__srv__ConnectCan_Response__init(altair_interfaces__srv__ConnectCan_Response * msg);

/// Finalize srv/ConnectCan message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
void
altair_interfaces__srv__ConnectCan_Response__fini(altair_interfaces__srv__ConnectCan_Response * msg);

/// Create srv/ConnectCan message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * altair_interfaces__srv__ConnectCan_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
altair_interfaces__srv__ConnectCan_Response *
altair_interfaces__srv__ConnectCan_Response__create();

/// Destroy srv/ConnectCan message.
/**
 * It calls
 * altair_interfaces__srv__ConnectCan_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
void
altair_interfaces__srv__ConnectCan_Response__destroy(altair_interfaces__srv__ConnectCan_Response * msg);

/// Check for srv/ConnectCan message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
bool
altair_interfaces__srv__ConnectCan_Response__are_equal(const altair_interfaces__srv__ConnectCan_Response * lhs, const altair_interfaces__srv__ConnectCan_Response * rhs);

/// Copy a srv/ConnectCan message.
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
altair_interfaces__srv__ConnectCan_Response__copy(
  const altair_interfaces__srv__ConnectCan_Response * input,
  altair_interfaces__srv__ConnectCan_Response * output);

/// Initialize array of srv/ConnectCan messages.
/**
 * It allocates the memory for the number of elements and calls
 * altair_interfaces__srv__ConnectCan_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
bool
altair_interfaces__srv__ConnectCan_Response__Sequence__init(altair_interfaces__srv__ConnectCan_Response__Sequence * array, size_t size);

/// Finalize array of srv/ConnectCan messages.
/**
 * It calls
 * altair_interfaces__srv__ConnectCan_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
void
altair_interfaces__srv__ConnectCan_Response__Sequence__fini(altair_interfaces__srv__ConnectCan_Response__Sequence * array);

/// Create array of srv/ConnectCan messages.
/**
 * It allocates the memory for the array and calls
 * altair_interfaces__srv__ConnectCan_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
altair_interfaces__srv__ConnectCan_Response__Sequence *
altair_interfaces__srv__ConnectCan_Response__Sequence__create(size_t size);

/// Destroy array of srv/ConnectCan messages.
/**
 * It calls
 * altair_interfaces__srv__ConnectCan_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
void
altair_interfaces__srv__ConnectCan_Response__Sequence__destroy(altair_interfaces__srv__ConnectCan_Response__Sequence * array);

/// Check for srv/ConnectCan message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_altair_interfaces
bool
altair_interfaces__srv__ConnectCan_Response__Sequence__are_equal(const altair_interfaces__srv__ConnectCan_Response__Sequence * lhs, const altair_interfaces__srv__ConnectCan_Response__Sequence * rhs);

/// Copy an array of srv/ConnectCan messages.
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
altair_interfaces__srv__ConnectCan_Response__Sequence__copy(
  const altair_interfaces__srv__ConnectCan_Response__Sequence * input,
  altair_interfaces__srv__ConnectCan_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ALTAIR_INTERFACES__SRV__DETAIL__CONNECT_CAN__FUNCTIONS_H_
