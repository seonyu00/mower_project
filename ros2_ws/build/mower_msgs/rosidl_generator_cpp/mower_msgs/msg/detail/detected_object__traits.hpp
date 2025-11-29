// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mower_msgs:msg/DetectedObject.idl
// generated code does not contain a copyright notice

#ifndef MOWER_MSGS__MSG__DETAIL__DETECTED_OBJECT__TRAITS_HPP_
#define MOWER_MSGS__MSG__DETAIL__DETECTED_OBJECT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mower_msgs/msg/detail/detected_object__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mower_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DetectedObject & msg,
  std::ostream & out)
{
  out << "{";
  // member: label
  {
    out << "label: ";
    rosidl_generator_traits::value_to_yaml(msg.label, out);
    out << ", ";
  }

  // member: detected
  {
    out << "detected: ";
    rosidl_generator_traits::value_to_yaml(msg.detected, out);
    out << ", ";
  }

  // member: distance
  {
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << ", ";
  }

  // member: angle
  {
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DetectedObject & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: label
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "label: ";
    rosidl_generator_traits::value_to_yaml(msg.label, out);
    out << "\n";
  }

  // member: detected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "detected: ";
    rosidl_generator_traits::value_to_yaml(msg.detected, out);
    out << "\n";
  }

  // member: distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << "\n";
  }

  // member: angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DetectedObject & msg, bool use_flow_style = false)
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

}  // namespace mower_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mower_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mower_msgs::msg::DetectedObject & msg,
  std::ostream & out, size_t indentation = 0)
{
  mower_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mower_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mower_msgs::msg::DetectedObject & msg)
{
  return mower_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mower_msgs::msg::DetectedObject>()
{
  return "mower_msgs::msg::DetectedObject";
}

template<>
inline const char * name<mower_msgs::msg::DetectedObject>()
{
  return "mower_msgs/msg/DetectedObject";
}

template<>
struct has_fixed_size<mower_msgs::msg::DetectedObject>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mower_msgs::msg::DetectedObject>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mower_msgs::msg::DetectedObject>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MOWER_MSGS__MSG__DETAIL__DETECTED_OBJECT__TRAITS_HPP_
