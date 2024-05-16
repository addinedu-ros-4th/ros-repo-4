// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from turtles_msgs:msg/Target.idl
// generated code does not contain a copyright notice

#ifndef TURTLES_MSGS__MSG__DETAIL__TARGET__BUILDER_HPP_
#define TURTLES_MSGS__MSG__DETAIL__TARGET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "turtles_msgs/msg/detail/target__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace turtles_msgs
{

namespace msg
{

namespace builder
{

class Init_Target_target
{
public:
  Init_Target_target()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::turtles_msgs::msg::Target target(::turtles_msgs::msg::Target::_target_type arg)
  {
    msg_.target = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtles_msgs::msg::Target msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtles_msgs::msg::Target>()
{
  return turtles_msgs::msg::builder::Init_Target_target();
}

}  // namespace turtles_msgs

#endif  // TURTLES_MSGS__MSG__DETAIL__TARGET__BUILDER_HPP_
