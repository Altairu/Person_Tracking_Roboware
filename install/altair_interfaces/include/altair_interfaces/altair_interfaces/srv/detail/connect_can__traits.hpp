// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from altair_interfaces:srv/ConnectCan.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__SRV__DETAIL__CONNECT_CAN__TRAITS_HPP_
#define ALTAIR_INTERFACES__SRV__DETAIL__CONNECT_CAN__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "altair_interfaces/srv/detail/connect_can__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace altair_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ConnectCan_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: port
  {
    out << "port: ";
    rosidl_generator_traits::value_to_yaml(msg.port, out);
    out << ", ";
  }

  // member: auto_connect
  {
    out << "auto_connect: ";
    rosidl_generator_traits::value_to_yaml(msg.auto_connect, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ConnectCan_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: port
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "port: ";
    rosidl_generator_traits::value_to_yaml(msg.port, out);
    out << "\n";
  }

  // member: auto_connect
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "auto_connect: ";
    rosidl_generator_traits::value_to_yaml(msg.auto_connect, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ConnectCan_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace altair_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use altair_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const altair_interfaces::srv::ConnectCan_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  altair_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use altair_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const altair_interfaces::srv::ConnectCan_Request & msg)
{
  return altair_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<altair_interfaces::srv::ConnectCan_Request>()
{
  return "altair_interfaces::srv::ConnectCan_Request";
}

template<>
inline const char * name<altair_interfaces::srv::ConnectCan_Request>()
{
  return "altair_interfaces/srv/ConnectCan_Request";
}

template<>
struct has_fixed_size<altair_interfaces::srv::ConnectCan_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<altair_interfaces::srv::ConnectCan_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<altair_interfaces::srv::ConnectCan_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace altair_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ConnectCan_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ConnectCan_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ConnectCan_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace altair_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use altair_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const altair_interfaces::srv::ConnectCan_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  altair_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use altair_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const altair_interfaces::srv::ConnectCan_Response & msg)
{
  return altair_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<altair_interfaces::srv::ConnectCan_Response>()
{
  return "altair_interfaces::srv::ConnectCan_Response";
}

template<>
inline const char * name<altair_interfaces::srv::ConnectCan_Response>()
{
  return "altair_interfaces/srv/ConnectCan_Response";
}

template<>
struct has_fixed_size<altair_interfaces::srv::ConnectCan_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<altair_interfaces::srv::ConnectCan_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<altair_interfaces::srv::ConnectCan_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<altair_interfaces::srv::ConnectCan>()
{
  return "altair_interfaces::srv::ConnectCan";
}

template<>
inline const char * name<altair_interfaces::srv::ConnectCan>()
{
  return "altair_interfaces/srv/ConnectCan";
}

template<>
struct has_fixed_size<altair_interfaces::srv::ConnectCan>
  : std::integral_constant<
    bool,
    has_fixed_size<altair_interfaces::srv::ConnectCan_Request>::value &&
    has_fixed_size<altair_interfaces::srv::ConnectCan_Response>::value
  >
{
};

template<>
struct has_bounded_size<altair_interfaces::srv::ConnectCan>
  : std::integral_constant<
    bool,
    has_bounded_size<altair_interfaces::srv::ConnectCan_Request>::value &&
    has_bounded_size<altair_interfaces::srv::ConnectCan_Response>::value
  >
{
};

template<>
struct is_service<altair_interfaces::srv::ConnectCan>
  : std::true_type
{
};

template<>
struct is_service_request<altair_interfaces::srv::ConnectCan_Request>
  : std::true_type
{
};

template<>
struct is_service_response<altair_interfaces::srv::ConnectCan_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ALTAIR_INTERFACES__SRV__DETAIL__CONNECT_CAN__TRAITS_HPP_
