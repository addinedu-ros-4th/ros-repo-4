// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from turtles_msgs:msg/Target.idl
// generated code does not contain a copyright notice

#ifndef TURTLES_MSGS__MSG__DETAIL__TARGET__TRAITS_HPP_
#define TURTLES_MSGS__MSG__DETAIL__TARGET__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "turtles_msgs/msg/detail/target__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace turtles_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Target & msg,
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
  const Target & msg,
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

inline std::string to_yaml(const Target & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace turtles_msgs

namespace rosidl_generator_traits
{

[[deprecated("use turtles_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtles_msgs::msg::Target & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtles_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtles_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const turtles_msgs::msg::Target & msg)
{
  return turtles_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<turtles_msgs::msg::Target>()
{
  return "turtles_msgs::msg::Target";
}

template<>
inline const char * name<turtles_msgs::msg::Target>()
{
  return "turtles_msgs/msg/Target";
}

template<>
struct has_fixed_size<turtles_msgs::msg::Target>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<turtles_msgs::msg::Target>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<turtles_msgs::msg::Target>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TURTLES_MSGS__MSG__DETAIL__TARGET__TRAITS_HPP_
