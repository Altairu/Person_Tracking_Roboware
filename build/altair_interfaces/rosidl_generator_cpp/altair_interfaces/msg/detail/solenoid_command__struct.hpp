// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from altair_interfaces:msg/SolenoidCommand.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__MSG__DETAIL__SOLENOID_COMMAND__STRUCT_HPP_
#define ALTAIR_INTERFACES__MSG__DETAIL__SOLENOID_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__altair_interfaces__msg__SolenoidCommand __attribute__((deprecated))
#else
# define DEPRECATED__altair_interfaces__msg__SolenoidCommand __declspec(deprecated)
#endif

namespace altair_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SolenoidCommand_
{
  using Type = SolenoidCommand_<ContainerAllocator>;

  explicit SolenoidCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<bool, 12>::iterator, bool>(this->valves.begin(), this->valves.end(), false);
    }
  }

  explicit SolenoidCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : valves(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<bool, 12>::iterator, bool>(this->valves.begin(), this->valves.end(), false);
    }
  }

  // field types and members
  using _valves_type =
    std::array<bool, 12>;
  _valves_type valves;

  // setters for named parameter idiom
  Type & set__valves(
    const std::array<bool, 12> & _arg)
  {
    this->valves = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    altair_interfaces::msg::SolenoidCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const altair_interfaces::msg::SolenoidCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<altair_interfaces::msg::SolenoidCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<altair_interfaces::msg::SolenoidCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      altair_interfaces::msg::SolenoidCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<altair_interfaces::msg::SolenoidCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      altair_interfaces::msg::SolenoidCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<altair_interfaces::msg::SolenoidCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<altair_interfaces::msg::SolenoidCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<altair_interfaces::msg::SolenoidCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__altair_interfaces__msg__SolenoidCommand
    std::shared_ptr<altair_interfaces::msg::SolenoidCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__altair_interfaces__msg__SolenoidCommand
    std::shared_ptr<altair_interfaces::msg::SolenoidCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SolenoidCommand_ & other) const
  {
    if (this->valves != other.valves) {
      return false;
    }
    return true;
  }
  bool operator!=(const SolenoidCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SolenoidCommand_

// alias to use template instance with default allocator
using SolenoidCommand =
  altair_interfaces::msg::SolenoidCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace altair_interfaces

#endif  // ALTAIR_INTERFACES__MSG__DETAIL__SOLENOID_COMMAND__STRUCT_HPP_
