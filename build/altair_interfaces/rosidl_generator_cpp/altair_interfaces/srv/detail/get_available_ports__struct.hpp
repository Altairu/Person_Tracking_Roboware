// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from altair_interfaces:srv/GetAvailablePorts.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__SRV__DETAIL__GET_AVAILABLE_PORTS__STRUCT_HPP_
#define ALTAIR_INTERFACES__SRV__DETAIL__GET_AVAILABLE_PORTS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__altair_interfaces__srv__GetAvailablePorts_Request __attribute__((deprecated))
#else
# define DEPRECATED__altair_interfaces__srv__GetAvailablePorts_Request __declspec(deprecated)
#endif

namespace altair_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetAvailablePorts_Request_
{
  using Type = GetAvailablePorts_Request_<ContainerAllocator>;

  explicit GetAvailablePorts_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit GetAvailablePorts_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    altair_interfaces::srv::GetAvailablePorts_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const altair_interfaces::srv::GetAvailablePorts_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<altair_interfaces::srv::GetAvailablePorts_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<altair_interfaces::srv::GetAvailablePorts_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      altair_interfaces::srv::GetAvailablePorts_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<altair_interfaces::srv::GetAvailablePorts_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      altair_interfaces::srv::GetAvailablePorts_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<altair_interfaces::srv::GetAvailablePorts_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<altair_interfaces::srv::GetAvailablePorts_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<altair_interfaces::srv::GetAvailablePorts_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__altair_interfaces__srv__GetAvailablePorts_Request
    std::shared_ptr<altair_interfaces::srv::GetAvailablePorts_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__altair_interfaces__srv__GetAvailablePorts_Request
    std::shared_ptr<altair_interfaces::srv::GetAvailablePorts_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetAvailablePorts_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetAvailablePorts_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetAvailablePorts_Request_

// alias to use template instance with default allocator
using GetAvailablePorts_Request =
  altair_interfaces::srv::GetAvailablePorts_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace altair_interfaces


#ifndef _WIN32
# define DEPRECATED__altair_interfaces__srv__GetAvailablePorts_Response __attribute__((deprecated))
#else
# define DEPRECATED__altair_interfaces__srv__GetAvailablePorts_Response __declspec(deprecated)
#endif

namespace altair_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetAvailablePorts_Response_
{
  using Type = GetAvailablePorts_Response_<ContainerAllocator>;

  explicit GetAvailablePorts_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit GetAvailablePorts_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _ports_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _ports_type ports;

  // setters for named parameter idiom
  Type & set__ports(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->ports = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    altair_interfaces::srv::GetAvailablePorts_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const altair_interfaces::srv::GetAvailablePorts_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<altair_interfaces::srv::GetAvailablePorts_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<altair_interfaces::srv::GetAvailablePorts_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      altair_interfaces::srv::GetAvailablePorts_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<altair_interfaces::srv::GetAvailablePorts_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      altair_interfaces::srv::GetAvailablePorts_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<altair_interfaces::srv::GetAvailablePorts_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<altair_interfaces::srv::GetAvailablePorts_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<altair_interfaces::srv::GetAvailablePorts_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__altair_interfaces__srv__GetAvailablePorts_Response
    std::shared_ptr<altair_interfaces::srv::GetAvailablePorts_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__altair_interfaces__srv__GetAvailablePorts_Response
    std::shared_ptr<altair_interfaces::srv::GetAvailablePorts_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetAvailablePorts_Response_ & other) const
  {
    if (this->ports != other.ports) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetAvailablePorts_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetAvailablePorts_Response_

// alias to use template instance with default allocator
using GetAvailablePorts_Response =
  altair_interfaces::srv::GetAvailablePorts_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace altair_interfaces

namespace altair_interfaces
{

namespace srv
{

struct GetAvailablePorts
{
  using Request = altair_interfaces::srv::GetAvailablePorts_Request;
  using Response = altair_interfaces::srv::GetAvailablePorts_Response;
};

}  // namespace srv

}  // namespace altair_interfaces

#endif  // ALTAIR_INTERFACES__SRV__DETAIL__GET_AVAILABLE_PORTS__STRUCT_HPP_
