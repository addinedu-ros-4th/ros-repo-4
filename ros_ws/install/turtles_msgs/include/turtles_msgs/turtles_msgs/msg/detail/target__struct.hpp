// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from turtles_msgs:msg/Target.idl
// generated code does not contain a copyright notice

#ifndef TURTLES_MSGS__MSG__DETAIL__TARGET__STRUCT_HPP_
#define TURTLES_MSGS__MSG__DETAIL__TARGET__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__turtles_msgs__msg__Target __attribute__((deprecated))
#else
# define DEPRECATED__turtles_msgs__msg__Target __declspec(deprecated)
#endif

namespace turtles_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Target_
{
  using Type = Target_<ContainerAllocator>;

  explicit Target_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target = "";
    }
  }

  explicit Target_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target = "";
    }
  }

  // field types and members
  using _target_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _target_type target;

  // setters for named parameter idiom
  Type & set__target(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->target = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtles_msgs::msg::Target_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtles_msgs::msg::Target_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtles_msgs::msg::Target_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtles_msgs::msg::Target_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtles_msgs::msg::Target_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtles_msgs::msg::Target_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtles_msgs::msg::Target_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtles_msgs::msg::Target_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtles_msgs::msg::Target_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtles_msgs::msg::Target_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtles_msgs__msg__Target
    std::shared_ptr<turtles_msgs::msg::Target_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtles_msgs__msg__Target
    std::shared_ptr<turtles_msgs::msg::Target_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Target_ & other) const
  {
    if (this->target != other.target) {
      return false;
    }
    return true;
  }
  bool operator!=(const Target_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Target_

// alias to use template instance with default allocator
using Target =
  turtles_msgs::msg::Target_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace turtles_msgs

#endif  // TURTLES_MSGS__MSG__DETAIL__TARGET__STRUCT_HPP_
