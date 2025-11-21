// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotdog_msgs:msg/KeyboardCtrlCmd.idl
// generated code does not contain a copyright notice

#ifndef ROBOTDOG_MSGS__MSG__DETAIL__KEYBOARD_CTRL_CMD__BUILDER_HPP_
#define ROBOTDOG_MSGS__MSG__DETAIL__KEYBOARD_CTRL_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robotdog_msgs/msg/detail/keyboard_ctrl_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robotdog_msgs
{

namespace msg
{

namespace builder
{

class Init_KeyboardCtrlCmd_gait_step
{
public:
  explicit Init_KeyboardCtrlCmd_gait_step(::robotdog_msgs::msg::KeyboardCtrlCmd & msg)
  : msg_(msg)
  {}
  ::robotdog_msgs::msg::KeyboardCtrlCmd gait_step(::robotdog_msgs::msg::KeyboardCtrlCmd::_gait_step_type arg)
  {
    msg_.gait_step = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotdog_msgs::msg::KeyboardCtrlCmd msg_;
};

class Init_KeyboardCtrlCmd_pose
{
public:
  explicit Init_KeyboardCtrlCmd_pose(::robotdog_msgs::msg::KeyboardCtrlCmd & msg)
  : msg_(msg)
  {}
  Init_KeyboardCtrlCmd_gait_step pose(::robotdog_msgs::msg::KeyboardCtrlCmd::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_KeyboardCtrlCmd_gait_step(msg_);
  }

private:
  ::robotdog_msgs::msg::KeyboardCtrlCmd msg_;
};

class Init_KeyboardCtrlCmd_gait_type
{
public:
  explicit Init_KeyboardCtrlCmd_gait_type(::robotdog_msgs::msg::KeyboardCtrlCmd & msg)
  : msg_(msg)
  {}
  Init_KeyboardCtrlCmd_pose gait_type(::robotdog_msgs::msg::KeyboardCtrlCmd::_gait_type_type arg)
  {
    msg_.gait_type = std::move(arg);
    return Init_KeyboardCtrlCmd_pose(msg_);
  }

private:
  ::robotdog_msgs::msg::KeyboardCtrlCmd msg_;
};

class Init_KeyboardCtrlCmd_states
{
public:
  Init_KeyboardCtrlCmd_states()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_KeyboardCtrlCmd_gait_type states(::robotdog_msgs::msg::KeyboardCtrlCmd::_states_type arg)
  {
    msg_.states = std::move(arg);
    return Init_KeyboardCtrlCmd_gait_type(msg_);
  }

private:
  ::robotdog_msgs::msg::KeyboardCtrlCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotdog_msgs::msg::KeyboardCtrlCmd>()
{
  return robotdog_msgs::msg::builder::Init_KeyboardCtrlCmd_states();
}

}  // namespace robotdog_msgs

#endif  // ROBOTDOG_MSGS__MSG__DETAIL__KEYBOARD_CTRL_CMD__BUILDER_HPP_
