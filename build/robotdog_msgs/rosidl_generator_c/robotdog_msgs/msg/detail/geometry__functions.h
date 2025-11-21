// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from robotdog_msgs:msg/Geometry.idl
// generated code does not contain a copyright notice

#ifndef ROBOTDOG_MSGS__MSG__DETAIL__GEOMETRY__FUNCTIONS_H_
#define ROBOTDOG_MSGS__MSG__DETAIL__GEOMETRY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "robotdog_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "robotdog_msgs/msg/detail/geometry__struct.h"

/// Initialize msg/Geometry message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robotdog_msgs__msg__Geometry
 * )) before or use
 * robotdog_msgs__msg__Geometry__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robotdog_msgs
bool
robotdog_msgs__msg__Geometry__init(robotdog_msgs__msg__Geometry * msg);

/// Finalize msg/Geometry message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotdog_msgs
void
robotdog_msgs__msg__Geometry__fini(robotdog_msgs__msg__Geometry * msg);

/// Create msg/Geometry message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robotdog_msgs__msg__Geometry__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robotdog_msgs
robotdog_msgs__msg__Geometry *
robotdog_msgs__msg__Geometry__create();

/// Destroy msg/Geometry message.
/**
 * It calls
 * robotdog_msgs__msg__Geometry__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotdog_msgs
void
robotdog_msgs__msg__Geometry__destroy(robotdog_msgs__msg__Geometry * msg);

/// Check for msg/Geometry message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotdog_msgs
bool
robotdog_msgs__msg__Geometry__are_equal(const robotdog_msgs__msg__Geometry * lhs, const robotdog_msgs__msg__Geometry * rhs);

/// Copy a msg/Geometry message.
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
ROSIDL_GENERATOR_C_PUBLIC_robotdog_msgs
bool
robotdog_msgs__msg__Geometry__copy(
  const robotdog_msgs__msg__Geometry * input,
  robotdog_msgs__msg__Geometry * output);

/// Initialize array of msg/Geometry messages.
/**
 * It allocates the memory for the number of elements and calls
 * robotdog_msgs__msg__Geometry__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotdog_msgs
bool
robotdog_msgs__msg__Geometry__Sequence__init(robotdog_msgs__msg__Geometry__Sequence * array, size_t size);

/// Finalize array of msg/Geometry messages.
/**
 * It calls
 * robotdog_msgs__msg__Geometry__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotdog_msgs
void
robotdog_msgs__msg__Geometry__Sequence__fini(robotdog_msgs__msg__Geometry__Sequence * array);

/// Create array of msg/Geometry messages.
/**
 * It allocates the memory for the array and calls
 * robotdog_msgs__msg__Geometry__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robotdog_msgs
robotdog_msgs__msg__Geometry__Sequence *
robotdog_msgs__msg__Geometry__Sequence__create(size_t size);

/// Destroy array of msg/Geometry messages.
/**
 * It calls
 * robotdog_msgs__msg__Geometry__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotdog_msgs
void
robotdog_msgs__msg__Geometry__Sequence__destroy(robotdog_msgs__msg__Geometry__Sequence * array);

/// Check for msg/Geometry message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotdog_msgs
bool
robotdog_msgs__msg__Geometry__Sequence__are_equal(const robotdog_msgs__msg__Geometry__Sequence * lhs, const robotdog_msgs__msg__Geometry__Sequence * rhs);

/// Copy an array of msg/Geometry messages.
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
ROSIDL_GENERATOR_C_PUBLIC_robotdog_msgs
bool
robotdog_msgs__msg__Geometry__Sequence__copy(
  const robotdog_msgs__msg__Geometry__Sequence * input,
  robotdog_msgs__msg__Geometry__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROBOTDOG_MSGS__MSG__DETAIL__GEOMETRY__FUNCTIONS_H_
