// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robotdog_msgs:msg/KeyboardCtrlCmd.idl
// generated code does not contain a copyright notice

#ifndef ROBOTDOG_MSGS__MSG__DETAIL__KEYBOARD_CTRL_CMD__STRUCT_H_
#define ROBOTDOG_MSGS__MSG__DETAIL__KEYBOARD_CTRL_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'gait_step'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/KeyboardCtrlCmd in the package robotdog_msgs.
/**
  * std_msgs/Header header
 */
typedef struct robotdog_msgs__msg__KeyboardCtrlCmd
{
  /// states represent the robot's states
  /// states[0] start
  /// states[1] walk
  /// states[2] side_move_mode also on
  bool states[3];
  /// This represent the type of selected gait: 0,1,2,3,....
  uint8_t gait_type;
  /// This represents the robot pose
  /// pose.position: slant-x, slant_y, height
  /// pose.orientation: roll, pitch, yaw
  geometry_msgs__msg__Pose pose;
  /// This represents the gait_step commands
  /// gait_step.x = steplen_x
  /// gait_step.y = steplen_y
  /// gait_step.z = swing_height
  geometry_msgs__msg__Vector3 gait_step;
} robotdog_msgs__msg__KeyboardCtrlCmd;

// Struct for a sequence of robotdog_msgs__msg__KeyboardCtrlCmd.
typedef struct robotdog_msgs__msg__KeyboardCtrlCmd__Sequence
{
  robotdog_msgs__msg__KeyboardCtrlCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotdog_msgs__msg__KeyboardCtrlCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOTDOG_MSGS__MSG__DETAIL__KEYBOARD_CTRL_CMD__STRUCT_H_
