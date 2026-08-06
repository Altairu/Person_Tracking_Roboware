// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from altair_interfaces:srv/StartBehavior.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__SRV__DETAIL__START_BEHAVIOR__STRUCT_HPP_
#define ALTAIR_INTERFACES__SRV__DETAIL__START_BEHAVIOR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__altair_interfaces__srv__StartBehavior_Request __attribute__((deprecated))
#else
# define DEPRECATED__altair_interfaces__srv__StartBehavior_Request __declspec(deprecated)
#endif

namespace altair_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct StartBehavior_Request_
{
  using Type = StartBehavior_Request_<ContainerAllocator>;

  explicit StartBehavior_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->behavior_name = "";
    }
  }

  explicit StartBehavior_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : behavior_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->behavior_name = "";
    }
  }

  // field types and members
  using _behavior_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _behavior_name_type behavior_name;

  // setters for named parameter idiom
  Type & set__behavior_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->behavior_name = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    altair_interfaces::srv::StartBehavior_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const altair_interfaces::srv::StartBehavior_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<altair_interfaces::srv::StartBehavior_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<altair_interfaces::srv::StartBehavior_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      altair_interfaces::srv::StartBehavior_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<altair_interfaces::srv::StartBehavior_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      altair_interfaces::srv::StartBehavior_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<altair_interfaces::srv::StartBehavior_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<altair_interfaces::srv::StartBehavior_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<altair_interfaces::srv::StartBehavior_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__altair_interfaces__srv__StartBehavior_Request
    std::shared_ptr<altair_interfaces::srv::StartBehavior_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__altair_interfaces__srv__StartBehavior_Request
    std::shared_ptr<altair_interfaces::srv::StartBehavior_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StartBehavior_Request_ & other) const
  {
    if (this->behavior_name != other.behavior_name) {
      return false;
    }
    return true;
  }
  bool operator!=(const StartBehavior_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StartBehavior_Request_

// alias to use template instance with default allocator
using StartBehavior_Request =
  altair_interfaces::srv::StartBehavior_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace altair_interfaces


#ifndef _WIN32
# define DEPRECATED__altair_interfaces__srv__StartBehavior_Response __attribute__((deprecated))
#else
# define DEPRECATED__altair_interfaces__srv__StartBehavior_Response __declspec(deprecated)
#endif

namespace altair_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct StartBehavior_Response_
{
  using Type = StartBehavior_Response_<ContainerAllocator>;

  explicit StartBehavior_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit StartBehavior_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    altair_interfaces::srv::StartBehavior_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const altair_interfaces::srv::StartBehavior_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<altair_interfaces::srv::StartBehavior_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<altair_interfaces::srv::StartBehavior_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      altair_interfaces::srv::StartBehavior_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<altair_interfaces::srv::StartBehavior_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      altair_interfaces::srv::StartBehavior_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<altair_interfaces::srv::StartBehavior_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<altair_interfaces::srv::StartBehavior_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<altair_interfaces::srv::StartBehavior_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__altair_interfaces__srv__StartBehavior_Response
    std::shared_ptr<altair_interfaces::srv::StartBehavior_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__altair_interfaces__srv__StartBehavior_Response
    std::shared_ptr<altair_interfaces::srv::StartBehavior_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StartBehavior_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const StartBehavior_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StartBehavior_Response_

// alias to use template instance with default allocator
using StartBehavior_Response =
  altair_interfaces::srv::StartBehavior_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace altair_interfaces

namespace altair_interfaces
{

namespace srv
{

struct StartBehavior
{
  using Request = altair_interfaces::srv::StartBehavior_Request;
  using Response = altair_interfaces::srv::StartBehavior_Response;
};

}  // namespace srv

}  // namespace altair_interfaces

#endif  // ALTAIR_INTERFACES__SRV__DETAIL__START_BEHAVIOR__STRUCT_HPP_
