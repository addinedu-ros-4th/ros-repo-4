// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from turtles_msgs:msg/Target.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "turtles_msgs/msg/detail/target__rosidl_typesupport_introspection_c.h"
#include "turtles_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "turtles_msgs/msg/detail/target__functions.h"
#include "turtles_msgs/msg/detail/target__struct.h"


// Include directives for member types
// Member `target`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void turtles_msgs__msg__Target__rosidl_typesupport_introspection_c__Target_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  turtles_msgs__msg__Target__init(message_memory);
}

void turtles_msgs__msg__Target__rosidl_typesupport_introspection_c__Target_fini_function(void * message_memory)
{
  turtles_msgs__msg__Target__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember turtles_msgs__msg__Target__rosidl_typesupport_introspection_c__Target_message_member_array[1] = {
  {
    "target",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtles_msgs__msg__Target, target),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers turtles_msgs__msg__Target__rosidl_typesupport_introspection_c__Target_message_members = {
  "turtles_msgs__msg",  // message namespace
  "Target",  // message name
  1,  // number of fields
  sizeof(turtles_msgs__msg__Target),
  turtles_msgs__msg__Target__rosidl_typesupport_introspection_c__Target_message_member_array,  // message members
  turtles_msgs__msg__Target__rosidl_typesupport_introspection_c__Target_init_function,  // function to initialize message memory (memory has to be allocated)
  turtles_msgs__msg__Target__rosidl_typesupport_introspection_c__Target_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t turtles_msgs__msg__Target__rosidl_typesupport_introspection_c__Target_message_type_support_handle = {
  0,
  &turtles_msgs__msg__Target__rosidl_typesupport_introspection_c__Target_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_turtles_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, turtles_msgs, msg, Target)() {
  if (!turtles_msgs__msg__Target__rosidl_typesupport_introspection_c__Target_message_type_support_handle.typesupport_identifier) {
    turtles_msgs__msg__Target__rosidl_typesupport_introspection_c__Target_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &turtles_msgs__msg__Target__rosidl_typesupport_introspection_c__Target_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
