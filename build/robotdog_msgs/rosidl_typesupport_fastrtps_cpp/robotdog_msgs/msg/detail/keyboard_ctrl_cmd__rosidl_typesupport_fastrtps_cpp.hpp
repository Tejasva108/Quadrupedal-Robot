// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from robotdog_msgs:msg/KeyboardCtrlCmd.idl
// generated code does not contain a copyright notice

#ifndef ROBOTDOG_MSGS__MSG__DETAIL__KEYBOARD_CTRL_CMD__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define ROBOTDOG_MSGS__MSG__DETAIL__KEYBOARD_CTRL_CMD__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "robotdog_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "robotdog_msgs/msg/detail/keyboard_ctrl_cmd__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace robotdog_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robotdog_msgs
cdr_serialize(
  const robotdog_msgs::msg::KeyboardCtrlCmd & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robotdog_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  robotdog_msgs::msg::KeyboardCtrlCmd & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robotdog_msgs
get_serialized_size(
  const robotdog_msgs::msg::KeyboardCtrlCmd & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robotdog_msgs
max_serialized_size_KeyboardCtrlCmd(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace robotdog_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robotdog_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robotdog_msgs, msg, KeyboardCtrlCmd)();

#ifdef __cplusplus
}
#endif

#endif  // ROBOTDOG_MSGS__MSG__DETAIL__KEYBOARD_CTRL_CMD__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
