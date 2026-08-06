// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from altair_interfaces:msg/MddCommand.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__MSG__DETAIL__MDD_COMMAND__STRUCT_HPP_
#define ALTAIR_INTERFACES__MSG__DETAIL__MDD_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__altair_interfaces__msg__MddCommand __attribute__((deprecated))
#else
# define DEPRECATED__altair_interfaces__msg__MddCommand __declspec(deprecated)
#endif

namespace altair_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MddCommand_
{
  using Type = MddCommand_<ContainerAllocator>;

  explicit MddCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 4>::iterator, float>(this->targets.begin(), this->targets.end(), 0.0f);
    }
  }

  explicit MddCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : targets(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 4>::iterator, float>(this->targets.begin(), this->targets.end(), 0.0f);
    }
  }

  // field types and members
  using _targets_type =
    std::array<float, 4>;
  _targets_type targets;

  // setters for named parameter idiom
  Type & set__targets(
    const std::array<float, 4> & _arg)
  {
    this->targets = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    altair_interfaces::msg::MddCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const altair_interfaces::msg::MddCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<altair_interfaces::msg::MddCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<altair_interfaces::msg::MddCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      altair_interfaces::msg::MddCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<altair_interfaces::msg::MddCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      altair_interfaces::msg::MddCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<altair_interfaces::msg::MddCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<altair_interfaces::msg::MddCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<altair_interfaces::msg::MddCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__altair_interfaces__msg__MddCommand
    std::shared_ptr<altair_interfaces::msg::MddCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__altair_interfaces__msg__MddCommand
    std::shared_ptr<altair_interfaces::msg::MddCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MddCommand_ & other) const
  {
    if (this->targets != other.targets) {
      return false;
    }
    return true;
  }
  bool operator!=(const MddCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MddCommand_

// alias to use template instance with default allocator
using MddCommand =
  altair_interfaces::msg::MddCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace altair_interfaces

#endif  // ALTAIR_INTERFACES__MSG__DETAIL__MDD_COMMAND__STRUCT_HPP_
