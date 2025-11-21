// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robotdog_msgs:msg/Geometry.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robotdog_msgs/msg/detail/geometry__rosidl_typesupport_introspection_c.h"
#include "robotdog_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robotdog_msgs/msg/detail/geometry__functions.h"
#include "robotdog_msgs/msg/detail/geometry__struct.h"


// Include directives for member types
// Member `fr`
// Member `fl`
// Member `br`
// Member `bl`
#include "geometry_msgs/msg/point32.h"
// Member `fr`
// Member `fl`
// Member `br`
// Member `bl`
#include "geometry_msgs/msg/detail/point32__rosidl_typesupport_introspection_c.h"
// Member `euler_ang`
#include "geometry_msgs/msg/quaternion.h"
// Member `euler_ang`
#include "geometry_msgs/msg/detail/quaternion__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robotdog_msgs__msg__Geometry__rosidl_typesupport_introspection_c__Geometry_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robotdog_msgs__msg__Geometry__init(message_memory);
}

void robotdog_msgs__msg__Geometry__rosidl_typesupport_introspection_c__Geometry_fini_function(void * message_memory)
{
  robotdog_msgs__msg__Geometry__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robotdog_msgs__msg__Geometry__rosidl_typesupport_introspection_c__Geometry_message_member_array[5] = {
  {
    "fr",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotdog_msgs__msg__Geometry, fr),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "fl",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotdog_msgs__msg__Geometry, fl),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "br",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotdog_msgs__msg__Geometry, br),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bl",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotdog_msgs__msg__Geometry, bl),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "euler_ang",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotdog_msgs__msg__Geometry, euler_ang),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robotdog_msgs__msg__Geometry__rosidl_typesupport_introspection_c__Geometry_message_members = {
  "robotdog_msgs__msg",  // message namespace
  "Geometry",  // message name
  5,  // number of fields
  sizeof(robotdog_msgs__msg__Geometry),
  robotdog_msgs__msg__Geometry__rosidl_typesupport_introspection_c__Geometry_message_member_array,  // message members
  robotdog_msgs__msg__Geometry__rosidl_typesupport_introspection_c__Geometry_init_function,  // function to initialize message memory (memory has to be allocated)
  robotdog_msgs__msg__Geometry__rosidl_typesupport_introspection_c__Geometry_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robotdog_msgs__msg__Geometry__rosidl_typesupport_introspection_c__Geometry_message_type_support_handle = {
  0,
  &robotdog_msgs__msg__Geometry__rosidl_typesupport_introspection_c__Geometry_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robotdog_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotdog_msgs, msg, Geometry)() {
  robotdog_msgs__msg__Geometry__rosidl_typesupport_introspection_c__Geometry_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point32)();
  robotdog_msgs__msg__Geometry__rosidl_typesupport_introspection_c__Geometry_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point32)();
  robotdog_msgs__msg__Geometry__rosidl_typesupport_introspection_c__Geometry_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point32)();
  robotdog_msgs__msg__Geometry__rosidl_typesupport_introspection_c__Geometry_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point32)();
  robotdog_msgs__msg__Geometry__rosidl_typesupport_introspection_c__Geometry_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Quaternion)();
  if (!robotdog_msgs__msg__Geometry__rosidl_typesupport_introspection_c__Geometry_message_type_support_handle.typesupport_identifier) {
    robotdog_msgs__msg__Geometry__rosidl_typesupport_introspection_c__Geometry_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robotdog_msgs__msg__Geometry__rosidl_typesupport_introspection_c__Geometry_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
