// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mower_msgs:msg/DetectedObjectList.idl
// generated code does not contain a copyright notice

#ifndef MOWER_MSGS__MSG__DETAIL__DETECTED_OBJECT_LIST__TRAITS_HPP_
#define MOWER_MSGS__MSG__DETAIL__DETECTED_OBJECT_LIST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mower_msgs/msg/detail/detected_object_list__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'objects'
#include "mower_msgs/msg/detail/detected_object__traits.hpp"

namespace mower_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DetectedObjectList & msg,
  std::ostream & out)
{
  out << "{";
  // member: objects
  {
    if (msg.objects.size() == 0) {
      out << "objects: []";
    } else {
      out << "objects: [";
      size_t pending_items = msg.objects.size();
      for (auto item : msg.objects) {
        to_flow_style_yaml(item, out);
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
  const DetectedObjectList & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: objects
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.objects.size() == 0) {
      out << "objects: []\n";
    } else {
      out << "objects:\n";
      for (auto item : msg.objects) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DetectedObjectList & msg, bool use_flow_style = false)
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
  const mower_msgs::msg::DetectedObjectList & msg,
  std::ostream & out, size_t indentation = 0)
{
  mower_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mower_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mower_msgs::msg::DetectedObjectList & msg)
{
  return mower_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mower_msgs::msg::DetectedObjectList>()
{
  return "mower_msgs::msg::DetectedObjectList";
}

template<>
inline const char * name<mower_msgs::msg::DetectedObjectList>()
{
  return "mower_msgs/msg/DetectedObjectList";
}

template<>
struct has_fixed_size<mower_msgs::msg::DetectedObjectList>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mower_msgs::msg::DetectedObjectList>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mower_msgs::msg::DetectedObjectList>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MOWER_MSGS__MSG__DETAIL__DETECTED_OBJECT_LIST__TRAITS_HPP_
