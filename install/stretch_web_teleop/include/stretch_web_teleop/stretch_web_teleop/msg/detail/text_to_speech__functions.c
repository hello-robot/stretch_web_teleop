// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from stretch_web_teleop:msg/TextToSpeech.idl
// generated code does not contain a copyright notice
#include "stretch_web_teleop/msg/detail/text_to_speech__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `text`
// Member `voice`
#include "rosidl_runtime_c/string_functions.h"

bool
stretch_web_teleop__msg__TextToSpeech__init(stretch_web_teleop__msg__TextToSpeech * msg)
{
  if (!msg) {
    return false;
  }
  // text
  if (!rosidl_runtime_c__String__init(&msg->text)) {
    stretch_web_teleop__msg__TextToSpeech__fini(msg);
    return false;
  }
  // voice
  if (!rosidl_runtime_c__String__init(&msg->voice)) {
    stretch_web_teleop__msg__TextToSpeech__fini(msg);
    return false;
  }
  // is_slow
  // override_behavior
  return true;
}

void
stretch_web_teleop__msg__TextToSpeech__fini(stretch_web_teleop__msg__TextToSpeech * msg)
{
  if (!msg) {
    return;
  }
  // text
  rosidl_runtime_c__String__fini(&msg->text);
  // voice
  rosidl_runtime_c__String__fini(&msg->voice);
  // is_slow
  // override_behavior
}

bool
stretch_web_teleop__msg__TextToSpeech__are_equal(const stretch_web_teleop__msg__TextToSpeech * lhs, const stretch_web_teleop__msg__TextToSpeech * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // text
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->text), &(rhs->text)))
  {
    return false;
  }
  // voice
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->voice), &(rhs->voice)))
  {
    return false;
  }
  // is_slow
  if (lhs->is_slow != rhs->is_slow) {
    return false;
  }
  // override_behavior
  if (lhs->override_behavior != rhs->override_behavior) {
    return false;
  }
  return true;
}

bool
stretch_web_teleop__msg__TextToSpeech__copy(
  const stretch_web_teleop__msg__TextToSpeech * input,
  stretch_web_teleop__msg__TextToSpeech * output)
{
  if (!input || !output) {
    return false;
  }
  // text
  if (!rosidl_runtime_c__String__copy(
      &(input->text), &(output->text)))
  {
    return false;
  }
  // voice
  if (!rosidl_runtime_c__String__copy(
      &(input->voice), &(output->voice)))
  {
    return false;
  }
  // is_slow
  output->is_slow = input->is_slow;
  // override_behavior
  output->override_behavior = input->override_behavior;
  return true;
}

stretch_web_teleop__msg__TextToSpeech *
stretch_web_teleop__msg__TextToSpeech__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stretch_web_teleop__msg__TextToSpeech * msg = (stretch_web_teleop__msg__TextToSpeech *)allocator.allocate(sizeof(stretch_web_teleop__msg__TextToSpeech), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(stretch_web_teleop__msg__TextToSpeech));
  bool success = stretch_web_teleop__msg__TextToSpeech__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
stretch_web_teleop__msg__TextToSpeech__destroy(stretch_web_teleop__msg__TextToSpeech * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    stretch_web_teleop__msg__TextToSpeech__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
stretch_web_teleop__msg__TextToSpeech__Sequence__init(stretch_web_teleop__msg__TextToSpeech__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stretch_web_teleop__msg__TextToSpeech * data = NULL;

  if (size) {
    data = (stretch_web_teleop__msg__TextToSpeech *)allocator.zero_allocate(size, sizeof(stretch_web_teleop__msg__TextToSpeech), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = stretch_web_teleop__msg__TextToSpeech__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        stretch_web_teleop__msg__TextToSpeech__fini(&data[i - 1]);
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
stretch_web_teleop__msg__TextToSpeech__Sequence__fini(stretch_web_teleop__msg__TextToSpeech__Sequence * array)
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
      stretch_web_teleop__msg__TextToSpeech__fini(&array->data[i]);
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

stretch_web_teleop__msg__TextToSpeech__Sequence *
stretch_web_teleop__msg__TextToSpeech__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stretch_web_teleop__msg__TextToSpeech__Sequence * array = (stretch_web_teleop__msg__TextToSpeech__Sequence *)allocator.allocate(sizeof(stretch_web_teleop__msg__TextToSpeech__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = stretch_web_teleop__msg__TextToSpeech__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
stretch_web_teleop__msg__TextToSpeech__Sequence__destroy(stretch_web_teleop__msg__TextToSpeech__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    stretch_web_teleop__msg__TextToSpeech__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
stretch_web_teleop__msg__TextToSpeech__Sequence__are_equal(const stretch_web_teleop__msg__TextToSpeech__Sequence * lhs, const stretch_web_teleop__msg__TextToSpeech__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!stretch_web_teleop__msg__TextToSpeech__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
stretch_web_teleop__msg__TextToSpeech__Sequence__copy(
  const stretch_web_teleop__msg__TextToSpeech__Sequence * input,
  stretch_web_teleop__msg__TextToSpeech__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(stretch_web_teleop__msg__TextToSpeech);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    stretch_web_teleop__msg__TextToSpeech * data =
      (stretch_web_teleop__msg__TextToSpeech *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!stretch_web_teleop__msg__TextToSpeech__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          stretch_web_teleop__msg__TextToSpeech__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!stretch_web_teleop__msg__TextToSpeech__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
