// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stretch_web_teleop:msg/TextToSpeech.idl
// generated code does not contain a copyright notice

#ifndef STRETCH_WEB_TELEOP__MSG__DETAIL__TEXT_TO_SPEECH__STRUCT_H_
#define STRETCH_WEB_TELEOP__MSG__DETAIL__TEXT_TO_SPEECH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'OVERRIDE_BEHAVIOR_QUEUE'.
/**
  * If a message is already being spoken, this flag controls what to do with this message:
  * add it to a queue to be executed sequentially (Default), or interrupt the
  * current message and queue to speak this message (in this case, the old queue gets
  * discarded).
 */
enum
{
  stretch_web_teleop__msg__TextToSpeech__OVERRIDE_BEHAVIOR_QUEUE = 0
};

/// Constant 'OVERRIDE_BEHAVIOR_INTERRUPT'.
enum
{
  stretch_web_teleop__msg__TextToSpeech__OVERRIDE_BEHAVIOR_INTERRUPT = 1
};

// Include directives for member types
// Member 'text'
// Member 'voice'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/TextToSpeech in the package stretch_web_teleop.
/**
  * The text to say
 */
typedef struct stretch_web_teleop__msg__TextToSpeech
{
  rosidl_runtime_c__String text;
  /// The voice to use. Valid options for this depend on the engine.
  rosidl_runtime_c__String voice;
  /// Whether to speak slow or not.
  bool is_slow;
  uint8_t override_behavior;
} stretch_web_teleop__msg__TextToSpeech;

// Struct for a sequence of stretch_web_teleop__msg__TextToSpeech.
typedef struct stretch_web_teleop__msg__TextToSpeech__Sequence
{
  stretch_web_teleop__msg__TextToSpeech * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stretch_web_teleop__msg__TextToSpeech__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STRETCH_WEB_TELEOP__MSG__DETAIL__TEXT_TO_SPEECH__STRUCT_H_
