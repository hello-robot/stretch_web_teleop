// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from stretch_web_teleop:msg/TextToSpeech.idl
// generated code does not contain a copyright notice

#ifndef STRETCH_WEB_TELEOP__MSG__DETAIL__TEXT_TO_SPEECH__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define STRETCH_WEB_TELEOP__MSG__DETAIL__TEXT_TO_SPEECH__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "stretch_web_teleop/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "stretch_web_teleop/msg/detail/text_to_speech__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace stretch_web_teleop
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_stretch_web_teleop
cdr_serialize(
  const stretch_web_teleop::msg::TextToSpeech & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_stretch_web_teleop
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  stretch_web_teleop::msg::TextToSpeech & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_stretch_web_teleop
get_serialized_size(
  const stretch_web_teleop::msg::TextToSpeech & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_stretch_web_teleop
max_serialized_size_TextToSpeech(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace stretch_web_teleop

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_stretch_web_teleop
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, stretch_web_teleop, msg, TextToSpeech)();

#ifdef __cplusplus
}
#endif

#endif  // STRETCH_WEB_TELEOP__MSG__DETAIL__TEXT_TO_SPEECH__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
