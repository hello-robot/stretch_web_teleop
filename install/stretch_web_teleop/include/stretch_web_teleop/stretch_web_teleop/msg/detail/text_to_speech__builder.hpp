// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from stretch_web_teleop:msg/TextToSpeech.idl
// generated code does not contain a copyright notice

#ifndef STRETCH_WEB_TELEOP__MSG__DETAIL__TEXT_TO_SPEECH__BUILDER_HPP_
#define STRETCH_WEB_TELEOP__MSG__DETAIL__TEXT_TO_SPEECH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "stretch_web_teleop/msg/detail/text_to_speech__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace stretch_web_teleop
{

namespace msg
{

namespace builder
{

class Init_TextToSpeech_override_behavior
{
public:
  explicit Init_TextToSpeech_override_behavior(::stretch_web_teleop::msg::TextToSpeech & msg)
  : msg_(msg)
  {}
  ::stretch_web_teleop::msg::TextToSpeech override_behavior(::stretch_web_teleop::msg::TextToSpeech::_override_behavior_type arg)
  {
    msg_.override_behavior = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stretch_web_teleop::msg::TextToSpeech msg_;
};

class Init_TextToSpeech_is_slow
{
public:
  explicit Init_TextToSpeech_is_slow(::stretch_web_teleop::msg::TextToSpeech & msg)
  : msg_(msg)
  {}
  Init_TextToSpeech_override_behavior is_slow(::stretch_web_teleop::msg::TextToSpeech::_is_slow_type arg)
  {
    msg_.is_slow = std::move(arg);
    return Init_TextToSpeech_override_behavior(msg_);
  }

private:
  ::stretch_web_teleop::msg::TextToSpeech msg_;
};

class Init_TextToSpeech_voice
{
public:
  explicit Init_TextToSpeech_voice(::stretch_web_teleop::msg::TextToSpeech & msg)
  : msg_(msg)
  {}
  Init_TextToSpeech_is_slow voice(::stretch_web_teleop::msg::TextToSpeech::_voice_type arg)
  {
    msg_.voice = std::move(arg);
    return Init_TextToSpeech_is_slow(msg_);
  }

private:
  ::stretch_web_teleop::msg::TextToSpeech msg_;
};

class Init_TextToSpeech_text
{
public:
  Init_TextToSpeech_text()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TextToSpeech_voice text(::stretch_web_teleop::msg::TextToSpeech::_text_type arg)
  {
    msg_.text = std::move(arg);
    return Init_TextToSpeech_voice(msg_);
  }

private:
  ::stretch_web_teleop::msg::TextToSpeech msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::stretch_web_teleop::msg::TextToSpeech>()
{
  return stretch_web_teleop::msg::builder::Init_TextToSpeech_text();
}

}  // namespace stretch_web_teleop

#endif  // STRETCH_WEB_TELEOP__MSG__DETAIL__TEXT_TO_SPEECH__BUILDER_HPP_
