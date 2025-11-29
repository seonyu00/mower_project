// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mower_msgs:msg/DetectedObjectList.idl
// generated code does not contain a copyright notice

#ifndef MOWER_MSGS__MSG__DETAIL__DETECTED_OBJECT_LIST__BUILDER_HPP_
#define MOWER_MSGS__MSG__DETAIL__DETECTED_OBJECT_LIST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mower_msgs/msg/detail/detected_object_list__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mower_msgs
{

namespace msg
{

namespace builder
{

class Init_DetectedObjectList_objects
{
public:
  Init_DetectedObjectList_objects()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mower_msgs::msg::DetectedObjectList objects(::mower_msgs::msg::DetectedObjectList::_objects_type arg)
  {
    msg_.objects = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mower_msgs::msg::DetectedObjectList msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mower_msgs::msg::DetectedObjectList>()
{
  return mower_msgs::msg::builder::Init_DetectedObjectList_objects();
}

}  // namespace mower_msgs

#endif  // MOWER_MSGS__MSG__DETAIL__DETECTED_OBJECT_LIST__BUILDER_HPP_
