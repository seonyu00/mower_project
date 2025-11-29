// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from mower_msgs:msg/DetectedObjectList.idl
// generated code does not contain a copyright notice

#ifndef MOWER_MSGS__MSG__DETAIL__DETECTED_OBJECT_LIST__FUNCTIONS_H_
#define MOWER_MSGS__MSG__DETAIL__DETECTED_OBJECT_LIST__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "mower_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "mower_msgs/msg/detail/detected_object_list__struct.h"

/// Initialize msg/DetectedObjectList message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * mower_msgs__msg__DetectedObjectList
 * )) before or use
 * mower_msgs__msg__DetectedObjectList__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_mower_msgs
bool
mower_msgs__msg__DetectedObjectList__init(mower_msgs__msg__DetectedObjectList * msg);

/// Finalize msg/DetectedObjectList message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mower_msgs
void
mower_msgs__msg__DetectedObjectList__fini(mower_msgs__msg__DetectedObjectList * msg);

/// Create msg/DetectedObjectList message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * mower_msgs__msg__DetectedObjectList__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mower_msgs
mower_msgs__msg__DetectedObjectList *
mower_msgs__msg__DetectedObjectList__create();

/// Destroy msg/DetectedObjectList message.
/**
 * It calls
 * mower_msgs__msg__DetectedObjectList__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mower_msgs
void
mower_msgs__msg__DetectedObjectList__destroy(mower_msgs__msg__DetectedObjectList * msg);

/// Check for msg/DetectedObjectList message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mower_msgs
bool
mower_msgs__msg__DetectedObjectList__are_equal(const mower_msgs__msg__DetectedObjectList * lhs, const mower_msgs__msg__DetectedObjectList * rhs);

/// Copy a msg/DetectedObjectList message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mower_msgs
bool
mower_msgs__msg__DetectedObjectList__copy(
  const mower_msgs__msg__DetectedObjectList * input,
  mower_msgs__msg__DetectedObjectList * output);

/// Initialize array of msg/DetectedObjectList messages.
/**
 * It allocates the memory for the number of elements and calls
 * mower_msgs__msg__DetectedObjectList__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_mower_msgs
bool
mower_msgs__msg__DetectedObjectList__Sequence__init(mower_msgs__msg__DetectedObjectList__Sequence * array, size_t size);

/// Finalize array of msg/DetectedObjectList messages.
/**
 * It calls
 * mower_msgs__msg__DetectedObjectList__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mower_msgs
void
mower_msgs__msg__DetectedObjectList__Sequence__fini(mower_msgs__msg__DetectedObjectList__Sequence * array);

/// Create array of msg/DetectedObjectList messages.
/**
 * It allocates the memory for the array and calls
 * mower_msgs__msg__DetectedObjectList__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mower_msgs
mower_msgs__msg__DetectedObjectList__Sequence *
mower_msgs__msg__DetectedObjectList__Sequence__create(size_t size);

/// Destroy array of msg/DetectedObjectList messages.
/**
 * It calls
 * mower_msgs__msg__DetectedObjectList__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mower_msgs
void
mower_msgs__msg__DetectedObjectList__Sequence__destroy(mower_msgs__msg__DetectedObjectList__Sequence * array);

/// Check for msg/DetectedObjectList message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mower_msgs
bool
mower_msgs__msg__DetectedObjectList__Sequence__are_equal(const mower_msgs__msg__DetectedObjectList__Sequence * lhs, const mower_msgs__msg__DetectedObjectList__Sequence * rhs);

/// Copy an array of msg/DetectedObjectList messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mower_msgs
bool
mower_msgs__msg__DetectedObjectList__Sequence__copy(
  const mower_msgs__msg__DetectedObjectList__Sequence * input,
  mower_msgs__msg__DetectedObjectList__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MOWER_MSGS__MSG__DETAIL__DETECTED_OBJECT_LIST__FUNCTIONS_H_
