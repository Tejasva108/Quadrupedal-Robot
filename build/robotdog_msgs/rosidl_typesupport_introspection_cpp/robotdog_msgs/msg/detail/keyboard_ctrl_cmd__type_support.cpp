// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from robotdog_msgs:msg/KeyboardCtrlCmd.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "robotdog_msgs/msg/detail/keyboard_ctrl_cmd__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace robotdog_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void KeyboardCtrlCmd_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) robotdog_msgs::msg::KeyboardCtrlCmd(_init);
}

void KeyboardCtrlCmd_fini_function(void * message_memory)
{
  auto typed_message = static_cast<robotdog_msgs::msg::KeyboardCtrlCmd *>(message_memory);
  typed_message->~KeyboardCtrlCmd();
}

size_t size_function__KeyboardCtrlCmd__states(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__KeyboardCtrlCmd__states(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<bool, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__KeyboardCtrlCmd__states(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<bool, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__KeyboardCtrlCmd__states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const bool *>(
    get_const_function__KeyboardCtrlCmd__states(untyped_member, index));
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = item;
}

void assign_function__KeyboardCtrlCmd__states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<bool *>(
    get_function__KeyboardCtrlCmd__states(untyped_member, index));
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember KeyboardCtrlCmd_message_member_array[4] = {
  {
    "states",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(robotdog_msgs::msg::KeyboardCtrlCmd, states),  // bytes offset in struct
    nullptr,  // default value
    size_function__KeyboardCtrlCmd__states,  // size() function pointer
    get_const_function__KeyboardCtrlCmd__states,  // get_const(index) function pointer
    get_function__KeyboardCtrlCmd__states,  // get(index) function pointer
    fetch_function__KeyboardCtrlCmd__states,  // fetch(index, &value) function pointer
    assign_function__KeyboardCtrlCmd__states,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "gait_type",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotdog_msgs::msg::KeyboardCtrlCmd, gait_type),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "pose",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Pose>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotdog_msgs::msg::KeyboardCtrlCmd, pose),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "gait_step",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Vector3>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotdog_msgs::msg::KeyboardCtrlCmd, gait_step),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers KeyboardCtrlCmd_message_members = {
  "robotdog_msgs::msg",  // message namespace
  "KeyboardCtrlCmd",  // message name
  4,  // number of fields
  sizeof(robotdog_msgs::msg::KeyboardCtrlCmd),
  KeyboardCtrlCmd_message_member_array,  // message members
  KeyboardCtrlCmd_init_function,  // function to initialize message memory (memory has to be allocated)
  KeyboardCtrlCmd_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t KeyboardCtrlCmd_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &KeyboardCtrlCmd_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace robotdog_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robotdog_msgs::msg::KeyboardCtrlCmd>()
{
  return &::robotdog_msgs::msg::rosidl_typesupport_introspection_cpp::KeyboardCtrlCmd_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robotdog_msgs, msg, KeyboardCtrlCmd)() {
  return &::robotdog_msgs::msg::rosidl_typesupport_introspection_cpp::KeyboardCtrlCmd_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
