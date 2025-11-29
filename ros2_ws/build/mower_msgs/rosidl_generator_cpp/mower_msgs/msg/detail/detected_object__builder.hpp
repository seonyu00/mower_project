// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mower_msgs:msg/DetectedObject.idl
// generated code does not contain a copyright notice

#ifndef MOWER_MSGS__MSG__DETAIL__DETECTED_OBJECT__BUILDER_HPP_
#define MOWER_MSGS__MSG__DETAIL__DETECTED_OBJECT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mower_msgs/msg/detail/detected_object__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mower_msgs
{

namespace msg
{

namespace builder
{

class Init_DetectedObject_angle
{
public:
  explicit Init_DetectedObject_angle(::mower_msgs::msg::DetectedObject & msg)
  : msg_(msg)
  {}
  ::mower_msgs::msg::DetectedObject angle(::mower_msgs::msg::DetectedObject::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mower_msgs::msg::DetectedObject msg_;
};

class Init_DetectedObject_distance
{
public:
  explicit Init_DetectedObject_distance(::mower_msgs::msg::DetectedObject & msg)
  : msg_(msg)
  {}
  Init_DetectedObject_angle distance(::mower_msgs::msg::DetectedObject::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return Init_DetectedObject_angle(msg_);
  }

private:
  ::mower_msgs::msg::DetectedObject msg_;
};

class Init_DetectedObject_detected
{
public:
  explicit Init_DetectedObject_detected(::mower_msgs::msg::DetectedObject & msg)
  : msg_(msg)
  {}
  Init_DetectedObject_distance detected(::mower_msgs::msg::DetectedObject::_detected_type arg)
  {
    msg_.detected = std::move(arg);
    return Init_DetectedObject_distance(msg_);
  }

private:
  ::mower_msgs::msg::DetectedObject msg_;
};

class Init_DetectedObject_label
{
public:
  Init_DetectedObject_label()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectedObject_detected label(::mower_msgs::msg::DetectedObject::_label_type arg)
  {
    msg_.label = std::move(arg);
    return Init_DetectedObject_detected(msg_);
  }

private:
  ::mower_msgs::msg::DetectedObject msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mower_msgs::msg::DetectedObject>()
{
  return mower_msgs::msg::builder::Init_DetectedObject_label();
}

}  // namespace mower_msgs

#endif  // MOWER_MSGS__MSG__DETAIL__DETECTED_OBJECT__BUILDER_HPP_
