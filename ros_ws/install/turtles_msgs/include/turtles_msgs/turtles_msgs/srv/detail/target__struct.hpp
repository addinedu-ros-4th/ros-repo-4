// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from turtles_msgs:srv/Target.idl
// generated code does not contain a copyright notice

#ifndef TURTLES_MSGS__SRV__DETAIL__TARGET__STRUCT_HPP_
#define TURTLES_MSGS__SRV__DETAIL__TARGET__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__turtles_msgs__srv__Target_Request __attribute__((deprecated))
#else
# define DEPRECATED__turtles_msgs__srv__Target_Request __declspec(deprecated)
#endif

namespace turtles_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Target_Request_
{
  using Type = Target_Request_<ContainerAllocator>;

  explicit Target_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target = "";
    }
  }

  explicit Target_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    turtles_msgs::srv::Target_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtles_msgs::srv::Target_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtles_msgs::srv::Target_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtles_msgs::srv::Target_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtles_msgs::srv::Target_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtles_msgs::srv::Target_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtles_msgs::srv::Target_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtles_msgs::srv::Target_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtles_msgs::srv::Target_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtles_msgs::srv::Target_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtles_msgs__srv__Target_Request
    std::shared_ptr<turtles_msgs::srv::Target_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtles_msgs__srv__Target_Request
    std::shared_ptr<turtles_msgs::srv::Target_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Target_Request_ & other) const
  {
    if (this->target != other.target) {
      return false;
    }
    return true;
  }
  bool operator!=(const Target_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Target_Request_

// alias to use template instance with default allocator
using Target_Request =
  turtles_msgs::srv::Target_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace turtles_msgs


#ifndef _WIN32
# define DEPRECATED__turtles_msgs__srv__Target_Response __attribute__((deprecated))
#else
# define DEPRECATED__turtles_msgs__srv__Target_Response __declspec(deprecated)
#endif

namespace turtles_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Target_Response_
{
  using Type = Target_Response_<ContainerAllocator>;

  explicit Target_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = "";
    }
  }

  explicit Target_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = "";
    }
  }

  // field types and members
  using _result_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__result(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtles_msgs::srv::Target_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtles_msgs::srv::Target_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtles_msgs::srv::Target_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtles_msgs::srv::Target_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtles_msgs::srv::Target_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtles_msgs::srv::Target_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtles_msgs::srv::Target_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtles_msgs::srv::Target_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtles_msgs::srv::Target_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtles_msgs::srv::Target_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtles_msgs__srv__Target_Response
    std::shared_ptr<turtles_msgs::srv::Target_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtles_msgs__srv__Target_Response
    std::shared_ptr<turtles_msgs::srv::Target_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Target_Response_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const Target_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Target_Response_

// alias to use template instance with default allocator
using Target_Response =
  turtles_msgs::srv::Target_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace turtles_msgs

namespace turtles_msgs
{

namespace srv
{

struct Target
{
  using Request = turtles_msgs::srv::Target_Request;
  using Response = turtles_msgs::srv::Target_Response;
};

}  // namespace srv

}  // namespace turtles_msgs

#endif  // TURTLES_MSGS__SRV__DETAIL__TARGET__STRUCT_HPP_
