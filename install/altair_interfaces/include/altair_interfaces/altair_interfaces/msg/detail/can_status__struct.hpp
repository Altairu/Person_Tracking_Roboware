// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from altair_interfaces:msg/CanStatus.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__MSG__DETAIL__CAN_STATUS__STRUCT_HPP_
#define ALTAIR_INTERFACES__MSG__DETAIL__CAN_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__altair_interfaces__msg__CanStatus __attribute__((deprecated))
#else
# define DEPRECATED__altair_interfaces__msg__CanStatus __declspec(deprecated)
#endif

namespace altair_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CanStatus_
{
  using Type = CanStatus_<ContainerAllocator>;

  explicit CanStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_connected = false;
      this->active_port = "";
      this->bitrate = 0ul;
      this->error_message = "";
    }
  }

  explicit CanStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : active_port(_alloc),
    error_message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_connected = false;
      this->active_port = "";
      this->bitrate = 0ul;
      this->error_message = "";
    }
  }

  // field types and members
  using _is_connected_type =
    bool;
  _is_connected_type is_connected;
  using _active_port_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _active_port_type active_port;
  using _bitrate_type =
    uint32_t;
  _bitrate_type bitrate;
  using _error_message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _error_message_type error_message;

  // setters for named parameter idiom
  Type & set__is_connected(
    const bool & _arg)
  {
    this->is_connected = _arg;
    return *this;
  }
  Type & set__active_port(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->active_port = _arg;
    return *this;
  }
  Type & set__bitrate(
    const uint32_t & _arg)
  {
    this->bitrate = _arg;
    return *this;
  }
  Type & set__error_message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->error_message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    altair_interfaces::msg::CanStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const altair_interfaces::msg::CanStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<altair_interfaces::msg::CanStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<altair_interfaces::msg::CanStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      altair_interfaces::msg::CanStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<altair_interfaces::msg::CanStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      altair_interfaces::msg::CanStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<altair_interfaces::msg::CanStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<altair_interfaces::msg::CanStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<altair_interfaces::msg::CanStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__altair_interfaces__msg__CanStatus
    std::shared_ptr<altair_interfaces::msg::CanStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__altair_interfaces__msg__CanStatus
    std::shared_ptr<altair_interfaces::msg::CanStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CanStatus_ & other) const
  {
    if (this->is_connected != other.is_connected) {
      return false;
    }
    if (this->active_port != other.active_port) {
      return false;
    }
    if (this->bitrate != other.bitrate) {
      return false;
    }
    if (this->error_message != other.error_message) {
      return false;
    }
    return true;
  }
  bool operator!=(const CanStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CanStatus_

// alias to use template instance with default allocator
using CanStatus =
  altair_interfaces::msg::CanStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace altair_interfaces

#endif  // ALTAIR_INTERFACES__MSG__DETAIL__CAN_STATUS__STRUCT_HPP_
