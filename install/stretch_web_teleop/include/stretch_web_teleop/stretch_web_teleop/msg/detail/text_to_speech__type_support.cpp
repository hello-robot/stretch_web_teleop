// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from stretch_web_teleop:msg/TextToSpeech.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "stretch_web_teleop/msg/detail/text_to_speech__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace stretch_web_teleop
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void TextToSpeech_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) stretch_web_teleop::msg::TextToSpeech(_init);
}

void TextToSpeech_fini_function(void * message_memory)
{
  auto typed_message = static_cast<stretch_web_teleop::msg::TextToSpeech *>(message_memory);
  typed_message->~TextToSpeech();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TextToSpeech_message_member_array[4] = {
  {
    "text",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(stretch_web_teleop::msg::TextToSpeech, text),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "voice",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(stretch_web_teleop::msg::TextToSpeech, voice),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "is_slow",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(stretch_web_teleop::msg::TextToSpeech, is_slow),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "override_behavior",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(stretch_web_teleop::msg::TextToSpeech, override_behavior),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TextToSpeech_message_members = {
  "stretch_web_teleop::msg",  // message namespace
  "TextToSpeech",  // message name
  4,  // number of fields
  sizeof(stretch_web_teleop::msg::TextToSpeech),
  TextToSpeech_message_member_array,  // message members
  TextToSpeech_init_function,  // function to initialize message memory (memory has to be allocated)
  TextToSpeech_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TextToSpeech_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TextToSpeech_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace stretch_web_teleop


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<stretch_web_teleop::msg::TextToSpeech>()
{
  return &::stretch_web_teleop::msg::rosidl_typesupport_introspection_cpp::TextToSpeech_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, stretch_web_teleop, msg, TextToSpeech)() {
  return &::stretch_web_teleop::msg::rosidl_typesupport_introspection_cpp::TextToSpeech_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
