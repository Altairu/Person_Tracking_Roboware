// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from altair_interfaces:srv/GetAvailablePorts.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__SRV__DETAIL__GET_AVAILABLE_PORTS__TRAITS_HPP_
#define ALTAIR_INTERFACES__SRV__DETAIL__GET_AVAILABLE_PORTS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "altair_interfaces/srv/detail/get_available_ports__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace altair_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetAvailablePorts_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetAvailablePorts_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetAvailablePorts_Request & msg, bool use_flow_style = false)
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
  const altair_interfaces::srv::GetAvailablePorts_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  altair_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use altair_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const altair_interfaces::srv::GetAvailablePorts_Request & msg)
{
  return altair_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<altair_interfaces::srv::GetAvailablePorts_Request>()
{
  return "altair_interfaces::srv::GetAvailablePorts_Request";
}

template<>
inline const char * name<altair_interfaces::srv::GetAvailablePorts_Request>()
{
  return "altair_interfaces/srv/GetAvailablePorts_Request";
}

template<>
struct has_fixed_size<altair_interfaces::srv::GetAvailablePorts_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<altair_interfaces::srv::GetAvailablePorts_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<altair_interfaces::srv::GetAvailablePorts_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace altair_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetAvailablePorts_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: ports
  {
    if (msg.ports.size() == 0) {
      out << "ports: []";
    } else {
      out << "ports: [";
      size_t pending_items = msg.ports.size();
      for (auto item : msg.ports) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetAvailablePorts_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ports
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ports.size() == 0) {
      out << "ports: []\n";
    } else {
      out << "ports:\n";
      for (auto item : msg.ports) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetAvailablePorts_Response & msg, bool use_flow_style = false)
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
  const altair_interfaces::srv::GetAvailablePorts_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  altair_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use altair_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const altair_interfaces::srv::GetAvailablePorts_Response & msg)
{
  return altair_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<altair_interfaces::srv::GetAvailablePorts_Response>()
{
  return "altair_interfaces::srv::GetAvailablePorts_Response";
}

template<>
inline const char * name<altair_interfaces::srv::GetAvailablePorts_Response>()
{
  return "altair_interfaces/srv/GetAvailablePorts_Response";
}

template<>
struct has_fixed_size<altair_interfaces::srv::GetAvailablePorts_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<altair_interfaces::srv::GetAvailablePorts_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<altair_interfaces::srv::GetAvailablePorts_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<altair_interfaces::srv::GetAvailablePorts>()
{
  return "altair_interfaces::srv::GetAvailablePorts";
}

template<>
inline const char * name<altair_interfaces::srv::GetAvailablePorts>()
{
  return "altair_interfaces/srv/GetAvailablePorts";
}

template<>
struct has_fixed_size<altair_interfaces::srv::GetAvailablePorts>
  : std::integral_constant<
    bool,
    has_fixed_size<altair_interfaces::srv::GetAvailablePorts_Request>::value &&
    has_fixed_size<altair_interfaces::srv::GetAvailablePorts_Response>::value
  >
{
};

template<>
struct has_bounded_size<altair_interfaces::srv::GetAvailablePorts>
  : std::integral_constant<
    bool,
    has_bounded_size<altair_interfaces::srv::GetAvailablePorts_Request>::value &&
    has_bounded_size<altair_interfaces::srv::GetAvailablePorts_Response>::value
  >
{
};

template<>
struct is_service<altair_interfaces::srv::GetAvailablePorts>
  : std::true_type
{
};

template<>
struct is_service_request<altair_interfaces::srv::GetAvailablePorts_Request>
  : std::true_type
{
};

template<>
struct is_service_response<altair_interfaces::srv::GetAvailablePorts_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ALTAIR_INTERFACES__SRV__DETAIL__GET_AVAILABLE_PORTS__TRAITS_HPP_
