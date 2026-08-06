// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from altair_interfaces:msg/CanStatus.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__MSG__DETAIL__CAN_STATUS__TRAITS_HPP_
#define ALTAIR_INTERFACES__MSG__DETAIL__CAN_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "altair_interfaces/msg/detail/can_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace altair_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const CanStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: is_connected
  {
    out << "is_connected: ";
    rosidl_generator_traits::value_to_yaml(msg.is_connected, out);
    out << ", ";
  }

  // member: active_port
  {
    out << "active_port: ";
    rosidl_generator_traits::value_to_yaml(msg.active_port, out);
    out << ", ";
  }

  // member: bitrate
  {
    out << "bitrate: ";
    rosidl_generator_traits::value_to_yaml(msg.bitrate, out);
    out << ", ";
  }

  // member: error_message
  {
    out << "error_message: ";
    rosidl_generator_traits::value_to_yaml(msg.error_message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CanStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: is_connected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_connected: ";
    rosidl_generator_traits::value_to_yaml(msg.is_connected, out);
    out << "\n";
  }

  // member: active_port
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "active_port: ";
    rosidl_generator_traits::value_to_yaml(msg.active_port, out);
    out << "\n";
  }

  // member: bitrate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bitrate: ";
    rosidl_generator_traits::value_to_yaml(msg.bitrate, out);
    out << "\n";
  }

  // member: error_message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_message: ";
    rosidl_generator_traits::value_to_yaml(msg.error_message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CanStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace altair_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use altair_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const altair_interfaces::msg::CanStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  altair_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use altair_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const altair_interfaces::msg::CanStatus & msg)
{
  return altair_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<altair_interfaces::msg::CanStatus>()
{
  return "altair_interfaces::msg::CanStatus";
}

template<>
inline const char * name<altair_interfaces::msg::CanStatus>()
{
  return "altair_interfaces/msg/CanStatus";
}

template<>
struct has_fixed_size<altair_interfaces::msg::CanStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<altair_interfaces::msg::CanStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<altair_interfaces::msg::CanStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ALTAIR_INTERFACES__MSG__DETAIL__CAN_STATUS__TRAITS_HPP_
