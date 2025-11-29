// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mower_msgs:msg/DetectedObjectList.idl
// generated code does not contain a copyright notice

#ifndef MOWER_MSGS__MSG__DETAIL__DETECTED_OBJECT_LIST__STRUCT_H_
#define MOWER_MSGS__MSG__DETAIL__DETECTED_OBJECT_LIST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'objects'
#include "mower_msgs/msg/detail/detected_object__struct.h"

/// Struct defined in msg/DetectedObjectList in the package mower_msgs.
/**
  * DetectedObject 메시지의 배열을 담는 메시지
 */
typedef struct mower_msgs__msg__DetectedObjectList
{
  mower_msgs__msg__DetectedObject__Sequence objects;
} mower_msgs__msg__DetectedObjectList;

// Struct for a sequence of mower_msgs__msg__DetectedObjectList.
typedef struct mower_msgs__msg__DetectedObjectList__Sequence
{
  mower_msgs__msg__DetectedObjectList * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mower_msgs__msg__DetectedObjectList__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOWER_MSGS__MSG__DETAIL__DETECTED_OBJECT_LIST__STRUCT_H_
