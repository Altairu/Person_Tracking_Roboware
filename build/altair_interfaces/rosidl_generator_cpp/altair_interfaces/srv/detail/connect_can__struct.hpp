// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from altair_interfaces:srv/ConnectCan.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__SRV__DETAIL__CONNECT_CAN__STRUCT_HPP_
#define ALTAIR_INTERFACES__SRV__DETAIL__CONNECT_CAN__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__altair_interfaces__srv__ConnectCan_Request __attribute__((deprecated))
#else
# define DEPRECATED__altair_interfaces__srv__ConnectCan_Request __declspec(deprecated)
#endif

namespace altair_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ConnectCan_Request_
{
  using Type = ConnectCan_Request_<ContainerAllocator>;

  explicit ConnectCan_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->port = "";
      this->auto_connect = false;
    }
  }

  explicit ConnectCan_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : port(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->port = "";
      this->auto_connect = false;
    }
  }

  // field types and members
  using _port_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _port_type port;
  using _auto_connect_type =
    bool;
  _auto_connect_type auto_connect;

  // setters for named parameter idiom
  Type & set__port(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->port = _arg;
    return *this;
  }
  Type & set__auto_connect(
    const bool & _arg)
  {
    this->auto_connect = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    altair_interfaces::srv::ConnectCan_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const altair_interfaces::srv::ConnectCan_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<altair_interfaces::srv::ConnectCan_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<altair_interfaces::srv::ConnectCan_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      altair_interfaces::srv::ConnectCan_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<altair_interfaces::srv::ConnectCan_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      altair_interfaces::srv::ConnectCan_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<altair_interfaces::srv::ConnectCan_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<altair_interfaces::srv::ConnectCan_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<altair_interfaces::srv::ConnectCan_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__altair_interfaces__srv__ConnectCan_Request
    std::shared_ptr<altair_interfaces::srv::ConnectCan_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__altair_interfaces__srv__ConnectCan_Request
    std::shared_ptr<altair_interfaces::srv::ConnectCan_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ConnectCan_Request_ & other) const
  {
    if (this->port != other.port) {
      return false;
    }
    if (this->auto_connect != other.auto_connect) {
      return false;
    }
    return true;
  }
  bool operator!=(const ConnectCan_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ConnectCan_Request_

// alias to use template instance with default allocator
using ConnectCan_Request =
  altair_interfaces::srv::ConnectCan_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace altair_interfaces


#ifndef _WIN32
# define DEPRECATED__altair_interfaces__srv__ConnectCan_Response __attribute__((deprecated))
#else
# define DEPRECATED__altair_interfaces__srv__ConnectCan_Response __declspec(deprecated)
#endif

namespace altair_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ConnectCan_Response_
{
  using Type = ConnectCan_Response_<ContainerAllocator>;

  explicit ConnectCan_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit ConnectCan_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    altair_interfaces::srv::ConnectCan_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const altair_interfaces::srv::ConnectCan_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<altair_interfaces::srv::ConnectCan_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<altair_interfaces::srv::ConnectCan_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      altair_interfaces::srv::ConnectCan_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<altair_interfaces::srv::ConnectCan_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      altair_interfaces::srv::ConnectCan_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<altair_interfaces::srv::ConnectCan_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<altair_interfaces::srv::ConnectCan_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<altair_interfaces::srv::ConnectCan_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__altair_interfaces__srv__ConnectCan_Response
    std::shared_ptr<altair_interfaces::srv::ConnectCan_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__altair_interfaces__srv__ConnectCan_Response
    std::shared_ptr<altair_interfaces::srv::ConnectCan_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ConnectCan_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const ConnectCan_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ConnectCan_Response_

// alias to use template instance with default allocator
using ConnectCan_Response =
  altair_interfaces::srv::ConnectCan_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace altair_interfaces

namespace altair_interfaces
{

namespace srv
{

struct ConnectCan
{
  using Request = altair_interfaces::srv::ConnectCan_Request;
  using Response = altair_interfaces::srv::ConnectCan_Response;
};

}  // namespace srv

}  // namespace altair_interfaces

#endif  // ALTAIR_INTERFACES__SRV__DETAIL__CONNECT_CAN__STRUCT_HPP_
