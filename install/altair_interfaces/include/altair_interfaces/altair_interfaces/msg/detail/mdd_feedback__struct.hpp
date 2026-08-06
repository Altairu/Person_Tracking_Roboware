// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from altair_interfaces:msg/MddFeedback.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__MSG__DETAIL__MDD_FEEDBACK__STRUCT_HPP_
#define ALTAIR_INTERFACES__MSG__DETAIL__MDD_FEEDBACK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__altair_interfaces__msg__MddFeedback __attribute__((deprecated))
#else
# define DEPRECATED__altair_interfaces__msg__MddFeedback __declspec(deprecated)
#endif

namespace altair_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MddFeedback_
{
  using Type = MddFeedback_<ContainerAllocator>;

  explicit MddFeedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<bool, 4>::iterator, bool>(this->limit_switches.begin(), this->limit_switches.end(), false);
      this->error_code = 0;
      this->system_status = 0;
      std::fill<typename std::array<float, 4>::iterator, float>(this->angles.begin(), this->angles.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->speeds.begin(), this->speeds.end(), 0.0f);
    }
  }

  explicit MddFeedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : limit_switches(_alloc),
    angles(_alloc),
    speeds(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<bool, 4>::iterator, bool>(this->limit_switches.begin(), this->limit_switches.end(), false);
      this->error_code = 0;
      this->system_status = 0;
      std::fill<typename std::array<float, 4>::iterator, float>(this->angles.begin(), this->angles.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->speeds.begin(), this->speeds.end(), 0.0f);
    }
  }

  // field types and members
  using _limit_switches_type =
    std::array<bool, 4>;
  _limit_switches_type limit_switches;
  using _error_code_type =
    uint8_t;
  _error_code_type error_code;
  using _system_status_type =
    uint8_t;
  _system_status_type system_status;
  using _angles_type =
    std::array<float, 4>;
  _angles_type angles;
  using _speeds_type =
    std::array<float, 4>;
  _speeds_type speeds;

  // setters for named parameter idiom
  Type & set__limit_switches(
    const std::array<bool, 4> & _arg)
  {
    this->limit_switches = _arg;
    return *this;
  }
  Type & set__error_code(
    const uint8_t & _arg)
  {
    this->error_code = _arg;
    return *this;
  }
  Type & set__system_status(
    const uint8_t & _arg)
  {
    this->system_status = _arg;
    return *this;
  }
  Type & set__angles(
    const std::array<float, 4> & _arg)
  {
    this->angles = _arg;
    return *this;
  }
  Type & set__speeds(
    const std::array<float, 4> & _arg)
  {
    this->speeds = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    altair_interfaces::msg::MddFeedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const altair_interfaces::msg::MddFeedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<altair_interfaces::msg::MddFeedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<altair_interfaces::msg::MddFeedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      altair_interfaces::msg::MddFeedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<altair_interfaces::msg::MddFeedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      altair_interfaces::msg::MddFeedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<altair_interfaces::msg::MddFeedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<altair_interfaces::msg::MddFeedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<altair_interfaces::msg::MddFeedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__altair_interfaces__msg__MddFeedback
    std::shared_ptr<altair_interfaces::msg::MddFeedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__altair_interfaces__msg__MddFeedback
    std::shared_ptr<altair_interfaces::msg::MddFeedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MddFeedback_ & other) const
  {
    if (this->limit_switches != other.limit_switches) {
      return false;
    }
    if (this->error_code != other.error_code) {
      return false;
    }
    if (this->system_status != other.system_status) {
      return false;
    }
    if (this->angles != other.angles) {
      return false;
    }
    if (this->speeds != other.speeds) {
      return false;
    }
    return true;
  }
  bool operator!=(const MddFeedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MddFeedback_

// alias to use template instance with default allocator
using MddFeedback =
  altair_interfaces::msg::MddFeedback_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace altair_interfaces

#endif  // ALTAIR_INTERFACES__MSG__DETAIL__MDD_FEEDBACK__STRUCT_HPP_
