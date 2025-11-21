// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robotdog_msgs:msg/Geometry.idl
// generated code does not contain a copyright notice

#ifndef ROBOTDOG_MSGS__MSG__DETAIL__GEOMETRY__STRUCT_H_
#define ROBOTDOG_MSGS__MSG__DETAIL__GEOMETRY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'fr'
// Member 'fl'
// Member 'br'
// Member 'bl'
#include "geometry_msgs/msg/detail/point32__struct.h"
// Member 'euler_ang'
#include "geometry_msgs/msg/detail/quaternion__struct.h"

/// Struct defined in msg/Geometry in the package robotdog_msgs.
/**
  * information of pose of the each leg and orientation of the body
 */
typedef struct robotdog_msgs__msg__Geometry
{
  geometry_msgs__msg__Point32 fr;
  geometry_msgs__msg__Point32 fl;
  geometry_msgs__msg__Point32 br;
  geometry_msgs__msg__Point32 bl;
  geometry_msgs__msg__Quaternion euler_ang;
} robotdog_msgs__msg__Geometry;

// Struct for a sequence of robotdog_msgs__msg__Geometry.
typedef struct robotdog_msgs__msg__Geometry__Sequence
{
  robotdog_msgs__msg__Geometry * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotdog_msgs__msg__Geometry__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOTDOG_MSGS__MSG__DETAIL__GEOMETRY__STRUCT_H_
