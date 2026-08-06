// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from altair_interfaces:msg/ServoCommand.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__MSG__DETAIL__SERVO_COMMAND__TRAITS_HPP_
#define ALTAIR_INTERFACES__MSG__DETAIL__SERVO_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "altair_interfaces/msg/detail/servo_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace altair_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ServoCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: angles
  {
    if (msg.angles.size() == 0) {
      out << "angles: []";
    } else {
      out << "angles: [";
      size_t pending_items = msg.angles.size();
      for (auto item : msg.angles) {
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
  const ServoCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: angles
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.angles.size() == 0) {
      out << "angles: []\n";
    } else {
      out << "angles:\n";
      for (auto item : msg.angles) {
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

inline std::string to_yaml(const ServoCommand & msg, bool use_flow_style = false)
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
  const altair_interfaces::msg::ServoCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  altair_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use altair_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const altair_interfaces::msg::ServoCommand & msg)
{
  return altair_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<altair_interfaces::msg::ServoCommand>()
{
  return "altair_interfaces::msg::ServoCommand";
}

template<>
inline const char * name<altair_interfaces::msg::ServoCommand>()
{
  return "altair_interfaces/msg/ServoCommand";
}

template<>
struct has_fixed_size<altair_interfaces::msg::ServoCommand>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<altair_interfaces::msg::ServoCommand>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<altair_interfaces::msg::ServoCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ALTAIR_INTERFACES__MSG__DETAIL__SERVO_COMMAND__TRAITS_HPP_
