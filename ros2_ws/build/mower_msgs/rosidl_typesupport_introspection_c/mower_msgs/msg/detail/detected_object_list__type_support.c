// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mower_msgs:msg/DetectedObjectList.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mower_msgs/msg/detail/detected_object_list__rosidl_typesupport_introspection_c.h"
#include "mower_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mower_msgs/msg/detail/detected_object_list__functions.h"
#include "mower_msgs/msg/detail/detected_object_list__struct.h"


// Include directives for member types
// Member `objects`
#include "mower_msgs/msg/detected_object.h"
// Member `objects`
#include "mower_msgs/msg/detail/detected_object__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mower_msgs__msg__DetectedObjectList__rosidl_typesupport_introspection_c__DetectedObjectList_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mower_msgs__msg__DetectedObjectList__init(message_memory);
}

void mower_msgs__msg__DetectedObjectList__rosidl_typesupport_introspection_c__DetectedObjectList_fini_function(void * message_memory)
{
  mower_msgs__msg__DetectedObjectList__fini(message_memory);
}

size_t mower_msgs__msg__DetectedObjectList__rosidl_typesupport_introspection_c__size_function__DetectedObjectList__objects(
  const void * untyped_member)
{
  const mower_msgs__msg__DetectedObject__Sequence * member =
    (const mower_msgs__msg__DetectedObject__Sequence *)(untyped_member);
  return member->size;
}

const void * mower_msgs__msg__DetectedObjectList__rosidl_typesupport_introspection_c__get_const_function__DetectedObjectList__objects(
  const void * untyped_member, size_t index)
{
  const mower_msgs__msg__DetectedObject__Sequence * member =
    (const mower_msgs__msg__DetectedObject__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mower_msgs__msg__DetectedObjectList__rosidl_typesupport_introspection_c__get_function__DetectedObjectList__objects(
  void * untyped_member, size_t index)
{
  mower_msgs__msg__DetectedObject__Sequence * member =
    (mower_msgs__msg__DetectedObject__Sequence *)(untyped_member);
  return &member->data[index];
}

void mower_msgs__msg__DetectedObjectList__rosidl_typesupport_introspection_c__fetch_function__DetectedObjectList__objects(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const mower_msgs__msg__DetectedObject * item =
    ((const mower_msgs__msg__DetectedObject *)
    mower_msgs__msg__DetectedObjectList__rosidl_typesupport_introspection_c__get_const_function__DetectedObjectList__objects(untyped_member, index));
  mower_msgs__msg__DetectedObject * value =
    (mower_msgs__msg__DetectedObject *)(untyped_value);
  *value = *item;
}

void mower_msgs__msg__DetectedObjectList__rosidl_typesupport_introspection_c__assign_function__DetectedObjectList__objects(
  void * untyped_member, size_t index, const void * untyped_value)
{
  mower_msgs__msg__DetectedObject * item =
    ((mower_msgs__msg__DetectedObject *)
    mower_msgs__msg__DetectedObjectList__rosidl_typesupport_introspection_c__get_function__DetectedObjectList__objects(untyped_member, index));
  const mower_msgs__msg__DetectedObject * value =
    (const mower_msgs__msg__DetectedObject *)(untyped_value);
  *item = *value;
}

bool mower_msgs__msg__DetectedObjectList__rosidl_typesupport_introspection_c__resize_function__DetectedObjectList__objects(
  void * untyped_member, size_t size)
{
  mower_msgs__msg__DetectedObject__Sequence * member =
    (mower_msgs__msg__DetectedObject__Sequence *)(untyped_member);
  mower_msgs__msg__DetectedObject__Sequence__fini(member);
  return mower_msgs__msg__DetectedObject__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember mower_msgs__msg__DetectedObjectList__rosidl_typesupport_introspection_c__DetectedObjectList_message_member_array[1] = {
  {
    "objects",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mower_msgs__msg__DetectedObjectList, objects),  // bytes offset in struct
    NULL,  // default value
    mower_msgs__msg__DetectedObjectList__rosidl_typesupport_introspection_c__size_function__DetectedObjectList__objects,  // size() function pointer
    mower_msgs__msg__DetectedObjectList__rosidl_typesupport_introspection_c__get_const_function__DetectedObjectList__objects,  // get_const(index) function pointer
    mower_msgs__msg__DetectedObjectList__rosidl_typesupport_introspection_c__get_function__DetectedObjectList__objects,  // get(index) function pointer
    mower_msgs__msg__DetectedObjectList__rosidl_typesupport_introspection_c__fetch_function__DetectedObjectList__objects,  // fetch(index, &value) function pointer
    mower_msgs__msg__DetectedObjectList__rosidl_typesupport_introspection_c__assign_function__DetectedObjectList__objects,  // assign(index, value) function pointer
    mower_msgs__msg__DetectedObjectList__rosidl_typesupport_introspection_c__resize_function__DetectedObjectList__objects  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mower_msgs__msg__DetectedObjectList__rosidl_typesupport_introspection_c__DetectedObjectList_message_members = {
  "mower_msgs__msg",  // message namespace
  "DetectedObjectList",  // message name
  1,  // number of fields
  sizeof(mower_msgs__msg__DetectedObjectList),
  mower_msgs__msg__DetectedObjectList__rosidl_typesupport_introspection_c__DetectedObjectList_message_member_array,  // message members
  mower_msgs__msg__DetectedObjectList__rosidl_typesupport_introspection_c__DetectedObjectList_init_function,  // function to initialize message memory (memory has to be allocated)
  mower_msgs__msg__DetectedObjectList__rosidl_typesupport_introspection_c__DetectedObjectList_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mower_msgs__msg__DetectedObjectList__rosidl_typesupport_introspection_c__DetectedObjectList_message_type_support_handle = {
  0,
  &mower_msgs__msg__DetectedObjectList__rosidl_typesupport_introspection_c__DetectedObjectList_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mower_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mower_msgs, msg, DetectedObjectList)() {
  mower_msgs__msg__DetectedObjectList__rosidl_typesupport_introspection_c__DetectedObjectList_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mower_msgs, msg, DetectedObject)();
  if (!mower_msgs__msg__DetectedObjectList__rosidl_typesupport_introspection_c__DetectedObjectList_message_type_support_handle.typesupport_identifier) {
    mower_msgs__msg__DetectedObjectList__rosidl_typesupport_introspection_c__DetectedObjectList_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mower_msgs__msg__DetectedObjectList__rosidl_typesupport_introspection_c__DetectedObjectList_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
