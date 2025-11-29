// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mower_msgs:msg/DetectedObjectList.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mower_msgs/msg/detail/detected_object_list__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace mower_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void DetectedObjectList_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mower_msgs::msg::DetectedObjectList(_init);
}

void DetectedObjectList_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mower_msgs::msg::DetectedObjectList *>(message_memory);
  typed_message->~DetectedObjectList();
}

size_t size_function__DetectedObjectList__objects(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<mower_msgs::msg::DetectedObject> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DetectedObjectList__objects(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<mower_msgs::msg::DetectedObject> *>(untyped_member);
  return &member[index];
}

void * get_function__DetectedObjectList__objects(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<mower_msgs::msg::DetectedObject> *>(untyped_member);
  return &member[index];
}

void fetch_function__DetectedObjectList__objects(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const mower_msgs::msg::DetectedObject *>(
    get_const_function__DetectedObjectList__objects(untyped_member, index));
  auto & value = *reinterpret_cast<mower_msgs::msg::DetectedObject *>(untyped_value);
  value = item;
}

void assign_function__DetectedObjectList__objects(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<mower_msgs::msg::DetectedObject *>(
    get_function__DetectedObjectList__objects(untyped_member, index));
  const auto & value = *reinterpret_cast<const mower_msgs::msg::DetectedObject *>(untyped_value);
  item = value;
}

void resize_function__DetectedObjectList__objects(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<mower_msgs::msg::DetectedObject> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember DetectedObjectList_message_member_array[1] = {
  {
    "objects",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<mower_msgs::msg::DetectedObject>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mower_msgs::msg::DetectedObjectList, objects),  // bytes offset in struct
    nullptr,  // default value
    size_function__DetectedObjectList__objects,  // size() function pointer
    get_const_function__DetectedObjectList__objects,  // get_const(index) function pointer
    get_function__DetectedObjectList__objects,  // get(index) function pointer
    fetch_function__DetectedObjectList__objects,  // fetch(index, &value) function pointer
    assign_function__DetectedObjectList__objects,  // assign(index, value) function pointer
    resize_function__DetectedObjectList__objects  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers DetectedObjectList_message_members = {
  "mower_msgs::msg",  // message namespace
  "DetectedObjectList",  // message name
  1,  // number of fields
  sizeof(mower_msgs::msg::DetectedObjectList),
  DetectedObjectList_message_member_array,  // message members
  DetectedObjectList_init_function,  // function to initialize message memory (memory has to be allocated)
  DetectedObjectList_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t DetectedObjectList_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &DetectedObjectList_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace mower_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mower_msgs::msg::DetectedObjectList>()
{
  return &::mower_msgs::msg::rosidl_typesupport_introspection_cpp::DetectedObjectList_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mower_msgs, msg, DetectedObjectList)() {
  return &::mower_msgs::msg::rosidl_typesupport_introspection_cpp::DetectedObjectList_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
