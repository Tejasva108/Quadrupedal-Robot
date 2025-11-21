// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robotdog_msgs:msg/KeyboardCtrlCmd.idl
// generated code does not contain a copyright notice
#include "robotdog_msgs/msg/detail/keyboard_ctrl_cmd__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `gait_step`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
robotdog_msgs__msg__KeyboardCtrlCmd__init(robotdog_msgs__msg__KeyboardCtrlCmd * msg)
{
  if (!msg) {
    return false;
  }
  // states
  // gait_type
  msg->gait_type = 0;
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    robotdog_msgs__msg__KeyboardCtrlCmd__fini(msg);
    return false;
  }
  // gait_step
  if (!geometry_msgs__msg__Vector3__init(&msg->gait_step)) {
    robotdog_msgs__msg__KeyboardCtrlCmd__fini(msg);
    return false;
  }
  return true;
}

void
robotdog_msgs__msg__KeyboardCtrlCmd__fini(robotdog_msgs__msg__KeyboardCtrlCmd * msg)
{
  if (!msg) {
    return;
  }
  // states
  // gait_type
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
  // gait_step
  geometry_msgs__msg__Vector3__fini(&msg->gait_step);
}

bool
robotdog_msgs__msg__KeyboardCtrlCmd__are_equal(const robotdog_msgs__msg__KeyboardCtrlCmd * lhs, const robotdog_msgs__msg__KeyboardCtrlCmd * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // states
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->states[i] != rhs->states[i]) {
      return false;
    }
  }
  // gait_type
  if (lhs->gait_type != rhs->gait_type) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // gait_step
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->gait_step), &(rhs->gait_step)))
  {
    return false;
  }
  return true;
}

bool
robotdog_msgs__msg__KeyboardCtrlCmd__copy(
  const robotdog_msgs__msg__KeyboardCtrlCmd * input,
  robotdog_msgs__msg__KeyboardCtrlCmd * output)
{
  if (!input || !output) {
    return false;
  }
  // states
  for (size_t i = 0; i < 3; ++i) {
    output->states[i] = input->states[i];
  }
  // gait_type
  output->gait_type = input->gait_type;
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // gait_step
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->gait_step), &(output->gait_step)))
  {
    return false;
  }
  return true;
}

robotdog_msgs__msg__KeyboardCtrlCmd *
robotdog_msgs__msg__KeyboardCtrlCmd__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotdog_msgs__msg__KeyboardCtrlCmd * msg = (robotdog_msgs__msg__KeyboardCtrlCmd *)allocator.allocate(sizeof(robotdog_msgs__msg__KeyboardCtrlCmd), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robotdog_msgs__msg__KeyboardCtrlCmd));
  bool success = robotdog_msgs__msg__KeyboardCtrlCmd__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robotdog_msgs__msg__KeyboardCtrlCmd__destroy(robotdog_msgs__msg__KeyboardCtrlCmd * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robotdog_msgs__msg__KeyboardCtrlCmd__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robotdog_msgs__msg__KeyboardCtrlCmd__Sequence__init(robotdog_msgs__msg__KeyboardCtrlCmd__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotdog_msgs__msg__KeyboardCtrlCmd * data = NULL;

  if (size) {
    data = (robotdog_msgs__msg__KeyboardCtrlCmd *)allocator.zero_allocate(size, sizeof(robotdog_msgs__msg__KeyboardCtrlCmd), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robotdog_msgs__msg__KeyboardCtrlCmd__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robotdog_msgs__msg__KeyboardCtrlCmd__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
robotdog_msgs__msg__KeyboardCtrlCmd__Sequence__fini(robotdog_msgs__msg__KeyboardCtrlCmd__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robotdog_msgs__msg__KeyboardCtrlCmd__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

robotdog_msgs__msg__KeyboardCtrlCmd__Sequence *
robotdog_msgs__msg__KeyboardCtrlCmd__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotdog_msgs__msg__KeyboardCtrlCmd__Sequence * array = (robotdog_msgs__msg__KeyboardCtrlCmd__Sequence *)allocator.allocate(sizeof(robotdog_msgs__msg__KeyboardCtrlCmd__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robotdog_msgs__msg__KeyboardCtrlCmd__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robotdog_msgs__msg__KeyboardCtrlCmd__Sequence__destroy(robotdog_msgs__msg__KeyboardCtrlCmd__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robotdog_msgs__msg__KeyboardCtrlCmd__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robotdog_msgs__msg__KeyboardCtrlCmd__Sequence__are_equal(const robotdog_msgs__msg__KeyboardCtrlCmd__Sequence * lhs, const robotdog_msgs__msg__KeyboardCtrlCmd__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robotdog_msgs__msg__KeyboardCtrlCmd__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robotdog_msgs__msg__KeyboardCtrlCmd__Sequence__copy(
  const robotdog_msgs__msg__KeyboardCtrlCmd__Sequence * input,
  robotdog_msgs__msg__KeyboardCtrlCmd__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robotdog_msgs__msg__KeyboardCtrlCmd);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robotdog_msgs__msg__KeyboardCtrlCmd * data =
      (robotdog_msgs__msg__KeyboardCtrlCmd *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robotdog_msgs__msg__KeyboardCtrlCmd__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robotdog_msgs__msg__KeyboardCtrlCmd__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robotdog_msgs__msg__KeyboardCtrlCmd__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
