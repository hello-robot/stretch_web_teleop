// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from stretch_web_teleop:msg/TextToSpeech.idl
// generated code does not contain a copyright notice

#ifndef STRETCH_WEB_TELEOP__MSG__DETAIL__TEXT_TO_SPEECH__STRUCT_HPP_
#define STRETCH_WEB_TELEOP__MSG__DETAIL__TEXT_TO_SPEECH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__stretch_web_teleop__msg__TextToSpeech __attribute__((deprecated))
#else
# define DEPRECATED__stretch_web_teleop__msg__TextToSpeech __declspec(deprecated)
#endif

namespace stretch_web_teleop
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TextToSpeech_
{
  using Type = TextToSpeech_<ContainerAllocator>;

  explicit TextToSpeech_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->text = "";
      this->voice = "";
      this->is_slow = false;
      this->override_behavior = 0;
    }
  }

  explicit TextToSpeech_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : text(_alloc),
    voice(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->text = "";
      this->voice = "";
      this->is_slow = false;
      this->override_behavior = 0;
    }
  }

  // field types and members
  using _text_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _text_type text;
  using _voice_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _voice_type voice;
  using _is_slow_type =
    bool;
  _is_slow_type is_slow;
  using _override_behavior_type =
    uint8_t;
  _override_behavior_type override_behavior;

  // setters for named parameter idiom
  Type & set__text(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->text = _arg;
    return *this;
  }
  Type & set__voice(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->voice = _arg;
    return *this;
  }
  Type & set__is_slow(
    const bool & _arg)
  {
    this->is_slow = _arg;
    return *this;
  }
  Type & set__override_behavior(
    const uint8_t & _arg)
  {
    this->override_behavior = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t OVERRIDE_BEHAVIOR_QUEUE =
    0u;
  static constexpr uint8_t OVERRIDE_BEHAVIOR_INTERRUPT =
    1u;

  // pointer types
  using RawPtr =
    stretch_web_teleop::msg::TextToSpeech_<ContainerAllocator> *;
  using ConstRawPtr =
    const stretch_web_teleop::msg::TextToSpeech_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<stretch_web_teleop::msg::TextToSpeech_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<stretch_web_teleop::msg::TextToSpeech_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      stretch_web_teleop::msg::TextToSpeech_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<stretch_web_teleop::msg::TextToSpeech_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      stretch_web_teleop::msg::TextToSpeech_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<stretch_web_teleop::msg::TextToSpeech_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<stretch_web_teleop::msg::TextToSpeech_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<stretch_web_teleop::msg::TextToSpeech_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__stretch_web_teleop__msg__TextToSpeech
    std::shared_ptr<stretch_web_teleop::msg::TextToSpeech_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__stretch_web_teleop__msg__TextToSpeech
    std::shared_ptr<stretch_web_teleop::msg::TextToSpeech_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TextToSpeech_ & other) const
  {
    if (this->text != other.text) {
      return false;
    }
    if (this->voice != other.voice) {
      return false;
    }
    if (this->is_slow != other.is_slow) {
      return false;
    }
    if (this->override_behavior != other.override_behavior) {
      return false;
    }
    return true;
  }
  bool operator!=(const TextToSpeech_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TextToSpeech_

// alias to use template instance with default allocator
using TextToSpeech =
  stretch_web_teleop::msg::TextToSpeech_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TextToSpeech_<ContainerAllocator>::OVERRIDE_BEHAVIOR_QUEUE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TextToSpeech_<ContainerAllocator>::OVERRIDE_BEHAVIOR_INTERRUPT;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace stretch_web_teleop

#endif  // STRETCH_WEB_TELEOP__MSG__DETAIL__TEXT_TO_SPEECH__STRUCT_HPP_
