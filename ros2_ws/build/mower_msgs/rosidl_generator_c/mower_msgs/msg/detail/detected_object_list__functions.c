// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mower_msgs:msg/DetectedObjectList.idl
// generated code does not contain a copyright notice
#include "mower_msgs/msg/detail/detected_object_list__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `objects`
#include "mower_msgs/msg/detail/detected_object__functions.h"

bool
mower_msgs__msg__DetectedObjectList__init(mower_msgs__msg__DetectedObjectList * msg)
{
  if (!msg) {
    return false;
  }
  // objects
  if (!mower_msgs__msg__DetectedObject__Sequence__init(&msg->objects, 0)) {
    mower_msgs__msg__DetectedObjectList__fini(msg);
    return false;
  }
  return true;
}

void
mower_msgs__msg__DetectedObjectList__fini(mower_msgs__msg__DetectedObjectList * msg)
{
  if (!msg) {
    return;
  }
  // objects
  mower_msgs__msg__DetectedObject__Sequence__fini(&msg->objects);
}

bool
mower_msgs__msg__DetectedObjectList__are_equal(const mower_msgs__msg__DetectedObjectList * lhs, const mower_msgs__msg__DetectedObjectList * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // objects
  if (!mower_msgs__msg__DetectedObject__Sequence__are_equal(
      &(lhs->objects), &(rhs->objects)))
  {
    return false;
  }
  return true;
}

bool
mower_msgs__msg__DetectedObjectList__copy(
  const mower_msgs__msg__DetectedObjectList * input,
  mower_msgs__msg__DetectedObjectList * output)
{
  if (!input || !output) {
    return false;
  }
  // objects
  if (!mower_msgs__msg__DetectedObject__Sequence__copy(
      &(input->objects), &(output->objects)))
  {
    return false;
  }
  return true;
}

mower_msgs__msg__DetectedObjectList *
mower_msgs__msg__DetectedObjectList__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mower_msgs__msg__DetectedObjectList * msg = (mower_msgs__msg__DetectedObjectList *)allocator.allocate(sizeof(mower_msgs__msg__DetectedObjectList), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mower_msgs__msg__DetectedObjectList));
  bool success = mower_msgs__msg__DetectedObjectList__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mower_msgs__msg__DetectedObjectList__destroy(mower_msgs__msg__DetectedObjectList * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mower_msgs__msg__DetectedObjectList__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mower_msgs__msg__DetectedObjectList__Sequence__init(mower_msgs__msg__DetectedObjectList__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mower_msgs__msg__DetectedObjectList * data = NULL;

  if (size) {
    data = (mower_msgs__msg__DetectedObjectList *)allocator.zero_allocate(size, sizeof(mower_msgs__msg__DetectedObjectList), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mower_msgs__msg__DetectedObjectList__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mower_msgs__msg__DetectedObjectList__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mower_msgs__msg__DetectedObjectList__Sequence__fini(mower_msgs__msg__DetectedObjectList__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mower_msgs__msg__DetectedObjectList__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mower_msgs__msg__DetectedObjectList__Sequence *
mower_msgs__msg__DetectedObjectList__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mower_msgs__msg__DetectedObjectList__Sequence * array = (mower_msgs__msg__DetectedObjectList__Sequence *)allocator.allocate(sizeof(mower_msgs__msg__DetectedObjectList__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mower_msgs__msg__DetectedObjectList__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mower_msgs__msg__DetectedObjectList__Sequence__destroy(mower_msgs__msg__DetectedObjectList__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mower_msgs__msg__DetectedObjectList__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mower_msgs__msg__DetectedObjectList__Sequence__are_equal(const mower_msgs__msg__DetectedObjectList__Sequence * lhs, const mower_msgs__msg__DetectedObjectList__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mower_msgs__msg__DetectedObjectList__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mower_msgs__msg__DetectedObjectList__Sequence__copy(
  const mower_msgs__msg__DetectedObjectList__Sequence * input,
  mower_msgs__msg__DetectedObjectList__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mower_msgs__msg__DetectedObjectList);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mower_msgs__msg__DetectedObjectList * data =
      (mower_msgs__msg__DetectedObjectList *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mower_msgs__msg__DetectedObjectList__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mower_msgs__msg__DetectedObjectList__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mower_msgs__msg__DetectedObjectList__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
