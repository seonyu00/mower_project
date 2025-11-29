// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mower_msgs:msg/DetectedObjectList.idl
// generated code does not contain a copyright notice

#ifndef MOWER_MSGS__MSG__DETAIL__DETECTED_OBJECT_LIST__STRUCT_HPP_
#define MOWER_MSGS__MSG__DETAIL__DETECTED_OBJECT_LIST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'objects'
#include "mower_msgs/msg/detail/detected_object__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mower_msgs__msg__DetectedObjectList __attribute__((deprecated))
#else
# define DEPRECATED__mower_msgs__msg__DetectedObjectList __declspec(deprecated)
#endif

namespace mower_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DetectedObjectList_
{
  using Type = DetectedObjectList_<ContainerAllocator>;

  explicit DetectedObjectList_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit DetectedObjectList_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _objects_type =
    std::vector<mower_msgs::msg::DetectedObject_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mower_msgs::msg::DetectedObject_<ContainerAllocator>>>;
  _objects_type objects;

  // setters for named parameter idiom
  Type & set__objects(
    const std::vector<mower_msgs::msg::DetectedObject_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mower_msgs::msg::DetectedObject_<ContainerAllocator>>> & _arg)
  {
    this->objects = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mower_msgs::msg::DetectedObjectList_<ContainerAllocator> *;
  using ConstRawPtr =
    const mower_msgs::msg::DetectedObjectList_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mower_msgs::msg::DetectedObjectList_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mower_msgs::msg::DetectedObjectList_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mower_msgs::msg::DetectedObjectList_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mower_msgs::msg::DetectedObjectList_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mower_msgs::msg::DetectedObjectList_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mower_msgs::msg::DetectedObjectList_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mower_msgs::msg::DetectedObjectList_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mower_msgs::msg::DetectedObjectList_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mower_msgs__msg__DetectedObjectList
    std::shared_ptr<mower_msgs::msg::DetectedObjectList_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mower_msgs__msg__DetectedObjectList
    std::shared_ptr<mower_msgs::msg::DetectedObjectList_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DetectedObjectList_ & other) const
  {
    if (this->objects != other.objects) {
      return false;
    }
    return true;
  }
  bool operator!=(const DetectedObjectList_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DetectedObjectList_

// alias to use template instance with default allocator
using DetectedObjectList =
  mower_msgs::msg::DetectedObjectList_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mower_msgs

#endif  // MOWER_MSGS__MSG__DETAIL__DETECTED_OBJECT_LIST__STRUCT_HPP_
