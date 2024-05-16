// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from turtles_msgs:srv/Target.idl
// generated code does not contain a copyright notice

#ifndef TURTLES_MSGS__SRV__DETAIL__TARGET__BUILDER_HPP_
#define TURTLES_MSGS__SRV__DETAIL__TARGET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "turtles_msgs/srv/detail/target__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace turtles_msgs
{

namespace srv
{

namespace builder
{

class Init_Target_Request_target
{
public:
  Init_Target_Request_target()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::turtles_msgs::srv::Target_Request target(::turtles_msgs::srv::Target_Request::_target_type arg)
  {
    msg_.target = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtles_msgs::srv::Target_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtles_msgs::srv::Target_Request>()
{
  return turtles_msgs::srv::builder::Init_Target_Request_target();
}

}  // namespace turtles_msgs


namespace turtles_msgs
{

namespace srv
{

namespace builder
{

class Init_Target_Response_result
{
public:
  Init_Target_Response_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::turtles_msgs::srv::Target_Response result(::turtles_msgs::srv::Target_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtles_msgs::srv::Target_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtles_msgs::srv::Target_Response>()
{
  return turtles_msgs::srv::builder::Init_Target_Response_result();
}

}  // namespace turtles_msgs

#endif  // TURTLES_MSGS__SRV__DETAIL__TARGET__BUILDER_HPP_
