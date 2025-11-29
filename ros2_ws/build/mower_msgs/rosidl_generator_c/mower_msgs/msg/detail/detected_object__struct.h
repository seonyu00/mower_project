// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mower_msgs:msg/DetectedObject.idl
// generated code does not contain a copyright notice

#ifndef MOWER_MSGS__MSG__DETAIL__DETECTED_OBJECT__STRUCT_H_
#define MOWER_MSGS__MSG__DETAIL__DETECTED_OBJECT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'label'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/DetectedObject in the package mower_msgs.
/**
  * 이 메시지는 단일 탐지 객체에 대한 정보를 담습니다.
 */
typedef struct mower_msgs__msg__DetectedObject
{
  rosidl_runtime_c__String label;
  bool detected;
  float distance;
  float angle;
} mower_msgs__msg__DetectedObject;

// Struct for a sequence of mower_msgs__msg__DetectedObject.
typedef struct mower_msgs__msg__DetectedObject__Sequence
{
  mower_msgs__msg__DetectedObject * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mower_msgs__msg__DetectedObject__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOWER_MSGS__MSG__DETAIL__DETECTED_OBJECT__STRUCT_H_
