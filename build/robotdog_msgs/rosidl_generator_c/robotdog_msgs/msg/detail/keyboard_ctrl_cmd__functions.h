// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from robotdog_msgs:msg/KeyboardCtrlCmd.idl
// generated code does not contain a copyright notice

#ifndef ROBOTDOG_MSGS__MSG__DETAIL__KEYBOARD_CTRL_CMD__FUNCTIONS_H_
#define ROBOTDOG_MSGS__MSG__DETAIL__KEYBOARD_CTRL_CMD__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "robotdog_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "robotdog_msgs/msg/detail/keyboard_ctrl_cmd__struct.h"

/// Initialize msg/KeyboardCtrlCmd message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robotdog_msgs__msg__KeyboardCtrlCmd
 * )) before or use
 * robotdog_msgs__msg__KeyboardCtrlCmd__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robotdog_msgs
bool
robotdog_msgs__msg__KeyboardCtrlCmd__init(robotdog_msgs__msg__KeyboardCtrlCmd * msg);

/// Finalize msg/KeyboardCtrlCmd message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotdog_msgs
void
robotdog_msgs__msg__KeyboardCtrlCmd__fini(robotdog_msgs__msg__KeyboardCtrlCmd * msg);

/// Create msg/KeyboardCtrlCmd message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robotdog_msgs__msg__KeyboardCtrlCmd__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robotdog_msgs
robotdog_msgs__msg__KeyboardCtrlCmd *
robotdog_msgs__msg__KeyboardCtrlCmd__create();

/// Destroy msg/KeyboardCtrlCmd message.
/**
 * It calls
 * robotdog_msgs__msg__KeyboardCtrlCmd__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotdog_msgs
void
robotdog_msgs__msg__KeyboardCtrlCmd__destroy(robotdog_msgs__msg__KeyboardCtrlCmd * msg);

/// Check for msg/KeyboardCtrlCmd message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotdog_msgs
bool
robotdog_msgs__msg__KeyboardCtrlCmd__are_equal(const robotdog_msgs__msg__KeyboardCtrlCmd * lhs, const robotdog_msgs__msg__KeyboardCtrlCmd * rhs);

/// Copy a msg/KeyboardCtrlCmd message.
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
robotdog_msgs__msg__KeyboardCtrlCmd__copy(
  const robotdog_msgs__msg__KeyboardCtrlCmd * input,
  robotdog_msgs__msg__KeyboardCtrlCmd * output);

/// Initialize array of msg/KeyboardCtrlCmd messages.
/**
 * It allocates the memory for the number of elements and calls
 * robotdog_msgs__msg__KeyboardCtrlCmd__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotdog_msgs
bool
robotdog_msgs__msg__KeyboardCtrlCmd__Sequence__init(robotdog_msgs__msg__KeyboardCtrlCmd__Sequence * array, size_t size);

/// Finalize array of msg/KeyboardCtrlCmd messages.
/**
 * It calls
 * robotdog_msgs__msg__KeyboardCtrlCmd__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotdog_msgs
void
robotdog_msgs__msg__KeyboardCtrlCmd__Sequence__fini(robotdog_msgs__msg__KeyboardCtrlCmd__Sequence * array);

/// Create array of msg/KeyboardCtrlCmd messages.
/**
 * It allocates the memory for the array and calls
 * robotdog_msgs__msg__KeyboardCtrlCmd__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robotdog_msgs
robotdog_msgs__msg__KeyboardCtrlCmd__Sequence *
robotdog_msgs__msg__KeyboardCtrlCmd__Sequence__create(size_t size);

/// Destroy array of msg/KeyboardCtrlCmd messages.
/**
 * It calls
 * robotdog_msgs__msg__KeyboardCtrlCmd__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotdog_msgs
void
robotdog_msgs__msg__KeyboardCtrlCmd__Sequence__destroy(robotdog_msgs__msg__KeyboardCtrlCmd__Sequence * array);

/// Check for msg/KeyboardCtrlCmd message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotdog_msgs
bool
robotdog_msgs__msg__KeyboardCtrlCmd__Sequence__are_equal(const robotdog_msgs__msg__KeyboardCtrlCmd__Sequence * lhs, const robotdog_msgs__msg__KeyboardCtrlCmd__Sequence * rhs);

/// Copy an array of msg/KeyboardCtrlCmd messages.
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
robotdog_msgs__msg__KeyboardCtrlCmd__Sequence__copy(
  const robotdog_msgs__msg__KeyboardCtrlCmd__Sequence * input,
  robotdog_msgs__msg__KeyboardCtrlCmd__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROBOTDOG_MSGS__MSG__DETAIL__KEYBOARD_CTRL_CMD__FUNCTIONS_H_
