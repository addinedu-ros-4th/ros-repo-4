// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from turtles_msgs:srv/Target.idl
// generated code does not contain a copyright notice

#ifndef TURTLES_MSGS__SRV__DETAIL__TARGET__TRAITS_HPP_
#define TURTLES_MSGS__SRV__DETAIL__TARGET__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "turtles_msgs/srv/detail/target__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace turtles_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const Target_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: target
  {
    out << "target: ";
    rosidl_generator_traits::value_to_yaml(msg.target, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Target_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: target
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target: ";
    rosidl_generator_traits::value_to_yaml(msg.target, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Target_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace turtles_msgs

namespace rosidl_generator_traits
{

[[deprecated("use turtles_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtles_msgs::srv::Target_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtles_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtles_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const turtles_msgs::srv::Target_Request & msg)
{
  return turtles_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<turtles_msgs::srv::Target_Request>()
{
  return "turtles_msgs::srv::Target_Request";
}

template<>
inline const char * name<turtles_msgs::srv::Target_Request>()
{
  return "turtles_msgs/srv/Target_Request";
}

template<>
struct has_fixed_size<turtles_msgs::srv::Target_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<turtles_msgs::srv::Target_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<turtles_msgs::srv::Target_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace turtles_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const Target_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: result
  {
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Target_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Target_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace turtles_msgs

namespace rosidl_generator_traits
{

[[deprecated("use turtles_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtles_msgs::srv::Target_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtles_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtles_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const turtles_msgs::srv::Target_Response & msg)
{
  return turtles_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<turtles_msgs::srv::Target_Response>()
{
  return "turtles_msgs::srv::Target_Response";
}

template<>
inline const char * name<turtles_msgs::srv::Target_Response>()
{
  return "turtles_msgs/srv/Target_Response";
}

template<>
struct has_fixed_size<turtles_msgs::srv::Target_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<turtles_msgs::srv::Target_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<turtles_msgs::srv::Target_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<turtles_msgs::srv::Target>()
{
  return "turtles_msgs::srv::Target";
}

template<>
inline const char * name<turtles_msgs::srv::Target>()
{
  return "turtles_msgs/srv/Target";
}

template<>
struct has_fixed_size<turtles_msgs::srv::Target>
  : std::integral_constant<
    bool,
    has_fixed_size<turtles_msgs::srv::Target_Request>::value &&
    has_fixed_size<turtles_msgs::srv::Target_Response>::value
  >
{
};

template<>
struct has_bounded_size<turtles_msgs::srv::Target>
  : std::integral_constant<
    bool,
    has_bounded_size<turtles_msgs::srv::Target_Request>::value &&
    has_bounded_size<turtles_msgs::srv::Target_Response>::value
  >
{
};

template<>
struct is_service<turtles_msgs::srv::Target>
  : std::true_type
{
};

template<>
struct is_service_request<turtles_msgs::srv::Target_Request>
  : std::true_type
{
};

template<>
struct is_service_response<turtles_msgs::srv::Target_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TURTLES_MSGS__SRV__DETAIL__TARGET__TRAITS_HPP_
