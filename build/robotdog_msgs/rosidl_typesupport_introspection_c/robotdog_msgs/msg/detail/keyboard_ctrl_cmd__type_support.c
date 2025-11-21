// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robotdog_msgs:msg/KeyboardCtrlCmd.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robotdog_msgs/msg/detail/keyboard_ctrl_cmd__rosidl_typesupport_introspection_c.h"
#include "robotdog_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robotdog_msgs/msg/detail/keyboard_ctrl_cmd__functions.h"
#include "robotdog_msgs/msg/detail/keyboard_ctrl_cmd__struct.h"


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/pose.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"
// Member `gait_step`
#include "geometry_msgs/msg/vector3.h"
// Member `gait_step`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robotdog_msgs__msg__KeyboardCtrlCmd__rosidl_typesupport_introspection_c__KeyboardCtrlCmd_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robotdog_msgs__msg__KeyboardCtrlCmd__init(message_memory);
}

void robotdog_msgs__msg__KeyboardCtrlCmd__rosidl_typesupport_introspection_c__KeyboardCtrlCmd_fini_function(void * message_memory)
{
  robotdog_msgs__msg__KeyboardCtrlCmd__fini(message_memory);
}

size_t robotdog_msgs__msg__KeyboardCtrlCmd__rosidl_typesupport_introspection_c__size_function__KeyboardCtrlCmd__states(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * robotdog_msgs__msg__KeyboardCtrlCmd__rosidl_typesupport_introspection_c__get_const_function__KeyboardCtrlCmd__states(
  const void * untyped_member, size_t index)
{
  const bool * member =
    (const bool *)(untyped_member);
  return &member[index];
}

void * robotdog_msgs__msg__KeyboardCtrlCmd__rosidl_typesupport_introspection_c__get_function__KeyboardCtrlCmd__states(
  void * untyped_member, size_t index)
{
  bool * member =
    (bool *)(untyped_member);
  return &member[index];
}

void robotdog_msgs__msg__KeyboardCtrlCmd__rosidl_typesupport_introspection_c__fetch_function__KeyboardCtrlCmd__states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    robotdog_msgs__msg__KeyboardCtrlCmd__rosidl_typesupport_introspection_c__get_const_function__KeyboardCtrlCmd__states(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void robotdog_msgs__msg__KeyboardCtrlCmd__rosidl_typesupport_introspection_c__assign_function__KeyboardCtrlCmd__states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    robotdog_msgs__msg__KeyboardCtrlCmd__rosidl_typesupport_introspection_c__get_function__KeyboardCtrlCmd__states(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember robotdog_msgs__msg__KeyboardCtrlCmd__rosidl_typesupport_introspection_c__KeyboardCtrlCmd_message_member_array[4] = {
  {
    "states",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(robotdog_msgs__msg__KeyboardCtrlCmd, states),  // bytes offset in struct
    NULL,  // default value
    robotdog_msgs__msg__KeyboardCtrlCmd__rosidl_typesupport_introspection_c__size_function__KeyboardCtrlCmd__states,  // size() function pointer
    robotdog_msgs__msg__KeyboardCtrlCmd__rosidl_typesupport_introspection_c__get_const_function__KeyboardCtrlCmd__states,  // get_const(index) function pointer
    robotdog_msgs__msg__KeyboardCtrlCmd__rosidl_typesupport_introspection_c__get_function__KeyboardCtrlCmd__states,  // get(index) function pointer
    robotdog_msgs__msg__KeyboardCtrlCmd__rosidl_typesupport_introspection_c__fetch_function__KeyboardCtrlCmd__states,  // fetch(index, &value) function pointer
    robotdog_msgs__msg__KeyboardCtrlCmd__rosidl_typesupport_introspection_c__assign_function__KeyboardCtrlCmd__states,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gait_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotdog_msgs__msg__KeyboardCtrlCmd, gait_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotdog_msgs__msg__KeyboardCtrlCmd, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gait_step",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotdog_msgs__msg__KeyboardCtrlCmd, gait_step),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robotdog_msgs__msg__KeyboardCtrlCmd__rosidl_typesupport_introspection_c__KeyboardCtrlCmd_message_members = {
  "robotdog_msgs__msg",  // message namespace
  "KeyboardCtrlCmd",  // message name
  4,  // number of fields
  sizeof(robotdog_msgs__msg__KeyboardCtrlCmd),
  robotdog_msgs__msg__KeyboardCtrlCmd__rosidl_typesupport_introspection_c__KeyboardCtrlCmd_message_member_array,  // message members
  robotdog_msgs__msg__KeyboardCtrlCmd__rosidl_typesupport_introspection_c__KeyboardCtrlCmd_init_function,  // function to initialize message memory (memory has to be allocated)
  robotdog_msgs__msg__KeyboardCtrlCmd__rosidl_typesupport_introspection_c__KeyboardCtrlCmd_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robotdog_msgs__msg__KeyboardCtrlCmd__rosidl_typesupport_introspection_c__KeyboardCtrlCmd_message_type_support_handle = {
  0,
  &robotdog_msgs__msg__KeyboardCtrlCmd__rosidl_typesupport_introspection_c__KeyboardCtrlCmd_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robotdog_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotdog_msgs, msg, KeyboardCtrlCmd)() {
  robotdog_msgs__msg__KeyboardCtrlCmd__rosidl_typesupport_introspection_c__KeyboardCtrlCmd_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  robotdog_msgs__msg__KeyboardCtrlCmd__rosidl_typesupport_introspection_c__KeyboardCtrlCmd_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  if (!robotdog_msgs__msg__KeyboardCtrlCmd__rosidl_typesupport_introspection_c__KeyboardCtrlCmd_message_type_support_handle.typesupport_identifier) {
    robotdog_msgs__msg__KeyboardCtrlCmd__rosidl_typesupport_introspection_c__KeyboardCtrlCmd_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robotdog_msgs__msg__KeyboardCtrlCmd__rosidl_typesupport_introspection_c__KeyboardCtrlCmd_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
