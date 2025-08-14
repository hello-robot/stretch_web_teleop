// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from stretch_web_teleop:msg/TextToSpeech.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "stretch_web_teleop/msg/detail/text_to_speech__rosidl_typesupport_introspection_c.h"
#include "stretch_web_teleop/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "stretch_web_teleop/msg/detail/text_to_speech__functions.h"
#include "stretch_web_teleop/msg/detail/text_to_speech__struct.h"


// Include directives for member types
// Member `text`
// Member `voice`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void stretch_web_teleop__msg__TextToSpeech__rosidl_typesupport_introspection_c__TextToSpeech_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  stretch_web_teleop__msg__TextToSpeech__init(message_memory);
}

void stretch_web_teleop__msg__TextToSpeech__rosidl_typesupport_introspection_c__TextToSpeech_fini_function(void * message_memory)
{
  stretch_web_teleop__msg__TextToSpeech__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember stretch_web_teleop__msg__TextToSpeech__rosidl_typesupport_introspection_c__TextToSpeech_message_member_array[4] = {
  {
    "text",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(stretch_web_teleop__msg__TextToSpeech, text),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "voice",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(stretch_web_teleop__msg__TextToSpeech, voice),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_slow",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(stretch_web_teleop__msg__TextToSpeech, is_slow),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "override_behavior",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(stretch_web_teleop__msg__TextToSpeech, override_behavior),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers stretch_web_teleop__msg__TextToSpeech__rosidl_typesupport_introspection_c__TextToSpeech_message_members = {
  "stretch_web_teleop__msg",  // message namespace
  "TextToSpeech",  // message name
  4,  // number of fields
  sizeof(stretch_web_teleop__msg__TextToSpeech),
  stretch_web_teleop__msg__TextToSpeech__rosidl_typesupport_introspection_c__TextToSpeech_message_member_array,  // message members
  stretch_web_teleop__msg__TextToSpeech__rosidl_typesupport_introspection_c__TextToSpeech_init_function,  // function to initialize message memory (memory has to be allocated)
  stretch_web_teleop__msg__TextToSpeech__rosidl_typesupport_introspection_c__TextToSpeech_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t stretch_web_teleop__msg__TextToSpeech__rosidl_typesupport_introspection_c__TextToSpeech_message_type_support_handle = {
  0,
  &stretch_web_teleop__msg__TextToSpeech__rosidl_typesupport_introspection_c__TextToSpeech_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_stretch_web_teleop
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, stretch_web_teleop, msg, TextToSpeech)() {
  if (!stretch_web_teleop__msg__TextToSpeech__rosidl_typesupport_introspection_c__TextToSpeech_message_type_support_handle.typesupport_identifier) {
    stretch_web_teleop__msg__TextToSpeech__rosidl_typesupport_introspection_c__TextToSpeech_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &stretch_web_teleop__msg__TextToSpeech__rosidl_typesupport_introspection_c__TextToSpeech_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
