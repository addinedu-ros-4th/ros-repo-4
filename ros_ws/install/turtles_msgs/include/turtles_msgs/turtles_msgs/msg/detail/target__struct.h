// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from turtles_msgs:msg/Target.idl
// generated code does not contain a copyright notice

#ifndef TURTLES_MSGS__MSG__DETAIL__TARGET__STRUCT_H_
#define TURTLES_MSGS__MSG__DETAIL__TARGET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'target'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Target in the package turtles_msgs.
typedef struct turtles_msgs__msg__Target
{
  rosidl_runtime_c__String target;
} turtles_msgs__msg__Target;

// Struct for a sequence of turtles_msgs__msg__Target.
typedef struct turtles_msgs__msg__Target__Sequence
{
  turtles_msgs__msg__Target * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtles_msgs__msg__Target__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TURTLES_MSGS__MSG__DETAIL__TARGET__STRUCT_H_
