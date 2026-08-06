// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from altair_interfaces:msg/ServoCommand.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__MSG__DETAIL__SERVO_COMMAND__STRUCT_HPP_
#define ALTAIR_INTERFACES__MSG__DETAIL__SERVO_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__altair_interfaces__msg__ServoCommand __attribute__((deprecated))
#else
# define DEPRECATED__altair_interfaces__msg__ServoCommand __declspec(deprecated)
#endif

namespace altair_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ServoCommand_
{
  using Type = ServoCommand_<ContainerAllocator>;

  explicit ServoCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<uint8_t, 6>::iterator, uint8_t>(this->angles.begin(), this->angles.end(), 0);
    }
  }

  explicit ServoCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : angles(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<uint8_t, 6>::iterator, uint8_t>(this->angles.begin(), this->angles.end(), 0);
    }
  }

  // field types and members
  using _angles_type =
    std::array<uint8_t, 6>;
  _angles_type angles;

  // setters for named parameter idiom
  Type & set__angles(
    const std::array<uint8_t, 6> & _arg)
  {
    this->angles = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    altair_interfaces::msg::ServoCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const altair_interfaces::msg::ServoCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<altair_interfaces::msg::ServoCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<altair_interfaces::msg::ServoCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      altair_interfaces::msg::ServoCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<altair_interfaces::msg::ServoCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      altair_interfaces::msg::ServoCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<altair_interfaces::msg::ServoCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<altair_interfaces::msg::ServoCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<altair_interfaces::msg::ServoCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__altair_interfaces__msg__ServoCommand
    std::shared_ptr<altair_interfaces::msg::ServoCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__altair_interfaces__msg__ServoCommand
    std::shared_ptr<altair_interfaces::msg::ServoCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ServoCommand_ & other) const
  {
    if (this->angles != other.angles) {
      return false;
    }
    return true;
  }
  bool operator!=(const ServoCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ServoCommand_

// alias to use template instance with default allocator
using ServoCommand =
  altair_interfaces::msg::ServoCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace altair_interfaces

#endif  // ALTAIR_INTERFACES__MSG__DETAIL__SERVO_COMMAND__STRUCT_HPP_
