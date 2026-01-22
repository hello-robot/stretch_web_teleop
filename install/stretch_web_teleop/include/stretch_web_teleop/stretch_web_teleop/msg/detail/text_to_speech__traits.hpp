// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from stretch_web_teleop:msg/TextToSpeech.idl
// generated code does not contain a copyright notice

#ifndef STRETCH_WEB_TELEOP__MSG__DETAIL__TEXT_TO_SPEECH__TRAITS_HPP_
#define STRETCH_WEB_TELEOP__MSG__DETAIL__TEXT_TO_SPEECH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "stretch_web_teleop/msg/detail/text_to_speech__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace stretch_web_teleop
{

namespace msg
{

inline void to_flow_style_yaml(
  const TextToSpeech & msg,
  std::ostream & out)
{
  out << "{";
  // member: text
  {
    out << "text: ";
    rosidl_generator_traits::value_to_yaml(msg.text, out);
    out << ", ";
  }

  // member: voice
  {
    out << "voice: ";
    rosidl_generator_traits::value_to_yaml(msg.voice, out);
    out << ", ";
  }

  // member: is_slow
  {
    out << "is_slow: ";
    rosidl_generator_traits::value_to_yaml(msg.is_slow, out);
    out << ", ";
  }

  // member: override_behavior
  {
    out << "override_behavior: ";
    rosidl_generator_traits::value_to_yaml(msg.override_behavior, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TextToSpeech & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: text
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "text: ";
    rosidl_generator_traits::value_to_yaml(msg.text, out);
    out << "\n";
  }

  // member: voice
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "voice: ";
    rosidl_generator_traits::value_to_yaml(msg.voice, out);
    out << "\n";
  }

  // member: is_slow
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_slow: ";
    rosidl_generator_traits::value_to_yaml(msg.is_slow, out);
    out << "\n";
  }

  // member: override_behavior
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "override_behavior: ";
    rosidl_generator_traits::value_to_yaml(msg.override_behavior, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TextToSpeech & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace stretch_web_teleop

namespace rosidl_generator_traits
{

[[deprecated("use stretch_web_teleop::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const stretch_web_teleop::msg::TextToSpeech & msg,
  std::ostream & out, size_t indentation = 0)
{
  stretch_web_teleop::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stretch_web_teleop::msg::to_yaml() instead")]]
inline std::string to_yaml(const stretch_web_teleop::msg::TextToSpeech & msg)
{
  return stretch_web_teleop::msg::to_yaml(msg);
}

template<>
inline const char * data_type<stretch_web_teleop::msg::TextToSpeech>()
{
  return "stretch_web_teleop::msg::TextToSpeech";
}

template<>
inline const char * name<stretch_web_teleop::msg::TextToSpeech>()
{
  return "stretch_web_teleop/msg/TextToSpeech";
}

template<>
struct has_fixed_size<stretch_web_teleop::msg::TextToSpeech>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<stretch_web_teleop::msg::TextToSpeech>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<stretch_web_teleop::msg::TextToSpeech>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // STRETCH_WEB_TELEOP__MSG__DETAIL__TEXT_TO_SPEECH__TRAITS_HPP_
