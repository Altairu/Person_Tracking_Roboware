// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from altair_interfaces:msg/MddFeedback.idl
// generated code does not contain a copyright notice

#ifndef ALTAIR_INTERFACES__MSG__DETAIL__MDD_FEEDBACK__TRAITS_HPP_
#define ALTAIR_INTERFACES__MSG__DETAIL__MDD_FEEDBACK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "altair_interfaces/msg/detail/mdd_feedback__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace altair_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const MddFeedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: limit_switches
  {
    if (msg.limit_switches.size() == 0) {
      out << "limit_switches: []";
    } else {
      out << "limit_switches: [";
      size_t pending_items = msg.limit_switches.size();
      for (auto item : msg.limit_switches) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: error_code
  {
    out << "error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_code, out);
    out << ", ";
  }

  // member: system_status
  {
    out << "system_status: ";
    rosidl_generator_traits::value_to_yaml(msg.system_status, out);
    out << ", ";
  }

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
    out << ", ";
  }

  // member: speeds
  {
    if (msg.speeds.size() == 0) {
      out << "speeds: []";
    } else {
      out << "speeds: [";
      size_t pending_items = msg.speeds.size();
      for (auto item : msg.speeds) {
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
  const MddFeedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: limit_switches
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.limit_switches.size() == 0) {
      out << "limit_switches: []\n";
    } else {
      out << "limit_switches:\n";
      for (auto item : msg.limit_switches) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: error_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_code, out);
    out << "\n";
  }

  // member: system_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "system_status: ";
    rosidl_generator_traits::value_to_yaml(msg.system_status, out);
    out << "\n";
  }

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

  // member: speeds
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.speeds.size() == 0) {
      out << "speeds: []\n";
    } else {
      out << "speeds:\n";
      for (auto item : msg.speeds) {
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

inline std::string to_yaml(const MddFeedback & msg, bool use_flow_style = false)
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
  const altair_interfaces::msg::MddFeedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  altair_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use altair_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const altair_interfaces::msg::MddFeedback & msg)
{
  return altair_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<altair_interfaces::msg::MddFeedback>()
{
  return "altair_interfaces::msg::MddFeedback";
}

template<>
inline const char * name<altair_interfaces::msg::MddFeedback>()
{
  return "altair_interfaces/msg/MddFeedback";
}

template<>
struct has_fixed_size<altair_interfaces::msg::MddFeedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<altair_interfaces::msg::MddFeedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<altair_interfaces::msg::MddFeedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ALTAIR_INTERFACES__MSG__DETAIL__MDD_FEEDBACK__TRAITS_HPP_
