// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from stretch_web_teleop:action/MoveToPregrasp.idl
// generated code does not contain a copyright notice
#include "stretch_web_teleop/action/detail/move_to_pregrasp__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
stretch_web_teleop__action__MoveToPregrasp_Goal__init(stretch_web_teleop__action__MoveToPregrasp_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // scaled_u
  // scaled_v
  // pregrasp_direction
  return true;
}

void
stretch_web_teleop__action__MoveToPregrasp_Goal__fini(stretch_web_teleop__action__MoveToPregrasp_Goal * msg)
{
  if (!msg) {
    return;
  }
  // scaled_u
  // scaled_v
  // pregrasp_direction
}

bool
stretch_web_teleop__action__MoveToPregrasp_Goal__are_equal(const stretch_web_teleop__action__MoveToPregrasp_Goal * lhs, const stretch_web_teleop__action__MoveToPregrasp_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // scaled_u
  if (lhs->scaled_u != rhs->scaled_u) {
    return false;
  }
  // scaled_v
  if (lhs->scaled_v != rhs->scaled_v) {
    return false;
  }
  // pregrasp_direction
  if (lhs->pregrasp_direction != rhs->pregrasp_direction) {
    return false;
  }
  return true;
}

bool
stretch_web_teleop__action__MoveToPregrasp_Goal__copy(
  const stretch_web_teleop__action__MoveToPregrasp_Goal * input,
  stretch_web_teleop__action__MoveToPregrasp_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // scaled_u
  output->scaled_u = input->scaled_u;
  // scaled_v
  output->scaled_v = input->scaled_v;
  // pregrasp_direction
  output->pregrasp_direction = input->pregrasp_direction;
  return true;
}

stretch_web_teleop__action__MoveToPregrasp_Goal *
stretch_web_teleop__action__MoveToPregrasp_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stretch_web_teleop__action__MoveToPregrasp_Goal * msg = (stretch_web_teleop__action__MoveToPregrasp_Goal *)allocator.allocate(sizeof(stretch_web_teleop__action__MoveToPregrasp_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(stretch_web_teleop__action__MoveToPregrasp_Goal));
  bool success = stretch_web_teleop__action__MoveToPregrasp_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
stretch_web_teleop__action__MoveToPregrasp_Goal__destroy(stretch_web_teleop__action__MoveToPregrasp_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    stretch_web_teleop__action__MoveToPregrasp_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence__init(stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stretch_web_teleop__action__MoveToPregrasp_Goal * data = NULL;

  if (size) {
    data = (stretch_web_teleop__action__MoveToPregrasp_Goal *)allocator.zero_allocate(size, sizeof(stretch_web_teleop__action__MoveToPregrasp_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = stretch_web_teleop__action__MoveToPregrasp_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        stretch_web_teleop__action__MoveToPregrasp_Goal__fini(&data[i - 1]);
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
stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence__fini(stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence * array)
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
      stretch_web_teleop__action__MoveToPregrasp_Goal__fini(&array->data[i]);
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

stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence *
stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence * array = (stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence *)allocator.allocate(sizeof(stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence__destroy(stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence__are_equal(const stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence * lhs, const stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!stretch_web_teleop__action__MoveToPregrasp_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence__copy(
  const stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence * input,
  stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(stretch_web_teleop__action__MoveToPregrasp_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    stretch_web_teleop__action__MoveToPregrasp_Goal * data =
      (stretch_web_teleop__action__MoveToPregrasp_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!stretch_web_teleop__action__MoveToPregrasp_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          stretch_web_teleop__action__MoveToPregrasp_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!stretch_web_teleop__action__MoveToPregrasp_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
stretch_web_teleop__action__MoveToPregrasp_Result__init(stretch_web_teleop__action__MoveToPregrasp_Result * msg)
{
  if (!msg) {
    return false;
  }
  // status
  return true;
}

void
stretch_web_teleop__action__MoveToPregrasp_Result__fini(stretch_web_teleop__action__MoveToPregrasp_Result * msg)
{
  if (!msg) {
    return;
  }
  // status
}

bool
stretch_web_teleop__action__MoveToPregrasp_Result__are_equal(const stretch_web_teleop__action__MoveToPregrasp_Result * lhs, const stretch_web_teleop__action__MoveToPregrasp_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  return true;
}

bool
stretch_web_teleop__action__MoveToPregrasp_Result__copy(
  const stretch_web_teleop__action__MoveToPregrasp_Result * input,
  stretch_web_teleop__action__MoveToPregrasp_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  return true;
}

stretch_web_teleop__action__MoveToPregrasp_Result *
stretch_web_teleop__action__MoveToPregrasp_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stretch_web_teleop__action__MoveToPregrasp_Result * msg = (stretch_web_teleop__action__MoveToPregrasp_Result *)allocator.allocate(sizeof(stretch_web_teleop__action__MoveToPregrasp_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(stretch_web_teleop__action__MoveToPregrasp_Result));
  bool success = stretch_web_teleop__action__MoveToPregrasp_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
stretch_web_teleop__action__MoveToPregrasp_Result__destroy(stretch_web_teleop__action__MoveToPregrasp_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    stretch_web_teleop__action__MoveToPregrasp_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
stretch_web_teleop__action__MoveToPregrasp_Result__Sequence__init(stretch_web_teleop__action__MoveToPregrasp_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stretch_web_teleop__action__MoveToPregrasp_Result * data = NULL;

  if (size) {
    data = (stretch_web_teleop__action__MoveToPregrasp_Result *)allocator.zero_allocate(size, sizeof(stretch_web_teleop__action__MoveToPregrasp_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = stretch_web_teleop__action__MoveToPregrasp_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        stretch_web_teleop__action__MoveToPregrasp_Result__fini(&data[i - 1]);
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
stretch_web_teleop__action__MoveToPregrasp_Result__Sequence__fini(stretch_web_teleop__action__MoveToPregrasp_Result__Sequence * array)
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
      stretch_web_teleop__action__MoveToPregrasp_Result__fini(&array->data[i]);
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

stretch_web_teleop__action__MoveToPregrasp_Result__Sequence *
stretch_web_teleop__action__MoveToPregrasp_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stretch_web_teleop__action__MoveToPregrasp_Result__Sequence * array = (stretch_web_teleop__action__MoveToPregrasp_Result__Sequence *)allocator.allocate(sizeof(stretch_web_teleop__action__MoveToPregrasp_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = stretch_web_teleop__action__MoveToPregrasp_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
stretch_web_teleop__action__MoveToPregrasp_Result__Sequence__destroy(stretch_web_teleop__action__MoveToPregrasp_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    stretch_web_teleop__action__MoveToPregrasp_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
stretch_web_teleop__action__MoveToPregrasp_Result__Sequence__are_equal(const stretch_web_teleop__action__MoveToPregrasp_Result__Sequence * lhs, const stretch_web_teleop__action__MoveToPregrasp_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!stretch_web_teleop__action__MoveToPregrasp_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
stretch_web_teleop__action__MoveToPregrasp_Result__Sequence__copy(
  const stretch_web_teleop__action__MoveToPregrasp_Result__Sequence * input,
  stretch_web_teleop__action__MoveToPregrasp_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(stretch_web_teleop__action__MoveToPregrasp_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    stretch_web_teleop__action__MoveToPregrasp_Result * data =
      (stretch_web_teleop__action__MoveToPregrasp_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!stretch_web_teleop__action__MoveToPregrasp_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          stretch_web_teleop__action__MoveToPregrasp_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!stretch_web_teleop__action__MoveToPregrasp_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `elapsed_time`
#include "builtin_interfaces/msg/detail/duration__functions.h"

bool
stretch_web_teleop__action__MoveToPregrasp_Feedback__init(stretch_web_teleop__action__MoveToPregrasp_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // initial_distance_m
  // remaining_distance_m
  // elapsed_time
  if (!builtin_interfaces__msg__Duration__init(&msg->elapsed_time)) {
    stretch_web_teleop__action__MoveToPregrasp_Feedback__fini(msg);
    return false;
  }
  return true;
}

void
stretch_web_teleop__action__MoveToPregrasp_Feedback__fini(stretch_web_teleop__action__MoveToPregrasp_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // initial_distance_m
  // remaining_distance_m
  // elapsed_time
  builtin_interfaces__msg__Duration__fini(&msg->elapsed_time);
}

bool
stretch_web_teleop__action__MoveToPregrasp_Feedback__are_equal(const stretch_web_teleop__action__MoveToPregrasp_Feedback * lhs, const stretch_web_teleop__action__MoveToPregrasp_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // initial_distance_m
  if (lhs->initial_distance_m != rhs->initial_distance_m) {
    return false;
  }
  // remaining_distance_m
  if (lhs->remaining_distance_m != rhs->remaining_distance_m) {
    return false;
  }
  // elapsed_time
  if (!builtin_interfaces__msg__Duration__are_equal(
      &(lhs->elapsed_time), &(rhs->elapsed_time)))
  {
    return false;
  }
  return true;
}

bool
stretch_web_teleop__action__MoveToPregrasp_Feedback__copy(
  const stretch_web_teleop__action__MoveToPregrasp_Feedback * input,
  stretch_web_teleop__action__MoveToPregrasp_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // initial_distance_m
  output->initial_distance_m = input->initial_distance_m;
  // remaining_distance_m
  output->remaining_distance_m = input->remaining_distance_m;
  // elapsed_time
  if (!builtin_interfaces__msg__Duration__copy(
      &(input->elapsed_time), &(output->elapsed_time)))
  {
    return false;
  }
  return true;
}

stretch_web_teleop__action__MoveToPregrasp_Feedback *
stretch_web_teleop__action__MoveToPregrasp_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stretch_web_teleop__action__MoveToPregrasp_Feedback * msg = (stretch_web_teleop__action__MoveToPregrasp_Feedback *)allocator.allocate(sizeof(stretch_web_teleop__action__MoveToPregrasp_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(stretch_web_teleop__action__MoveToPregrasp_Feedback));
  bool success = stretch_web_teleop__action__MoveToPregrasp_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
stretch_web_teleop__action__MoveToPregrasp_Feedback__destroy(stretch_web_teleop__action__MoveToPregrasp_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    stretch_web_teleop__action__MoveToPregrasp_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence__init(stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stretch_web_teleop__action__MoveToPregrasp_Feedback * data = NULL;

  if (size) {
    data = (stretch_web_teleop__action__MoveToPregrasp_Feedback *)allocator.zero_allocate(size, sizeof(stretch_web_teleop__action__MoveToPregrasp_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = stretch_web_teleop__action__MoveToPregrasp_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        stretch_web_teleop__action__MoveToPregrasp_Feedback__fini(&data[i - 1]);
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
stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence__fini(stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence * array)
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
      stretch_web_teleop__action__MoveToPregrasp_Feedback__fini(&array->data[i]);
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

stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence *
stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence * array = (stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence *)allocator.allocate(sizeof(stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence__destroy(stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence__are_equal(const stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence * lhs, const stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!stretch_web_teleop__action__MoveToPregrasp_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence__copy(
  const stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence * input,
  stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(stretch_web_teleop__action__MoveToPregrasp_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    stretch_web_teleop__action__MoveToPregrasp_Feedback * data =
      (stretch_web_teleop__action__MoveToPregrasp_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!stretch_web_teleop__action__MoveToPregrasp_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          stretch_web_teleop__action__MoveToPregrasp_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!stretch_web_teleop__action__MoveToPregrasp_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "stretch_web_teleop/action/detail/move_to_pregrasp__functions.h"

bool
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__init(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!stretch_web_teleop__action__MoveToPregrasp_Goal__init(&msg->goal)) {
    stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__fini(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  stretch_web_teleop__action__MoveToPregrasp_Goal__fini(&msg->goal);
}

bool
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__are_equal(const stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request * lhs, const stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!stretch_web_teleop__action__MoveToPregrasp_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__copy(
  const stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request * input,
  stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!stretch_web_teleop__action__MoveToPregrasp_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request *
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request * msg = (stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request *)allocator.allocate(sizeof(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request));
  bool success = stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__destroy(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence__init(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request * data = NULL;

  if (size) {
    data = (stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request *)allocator.zero_allocate(size, sizeof(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__fini(&data[i - 1]);
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
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence__fini(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence * array)
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
      stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__fini(&array->data[i]);
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

stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence *
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence * array = (stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence *)allocator.allocate(sizeof(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence__destroy(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence__are_equal(const stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence * lhs, const stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence__copy(
  const stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence * input,
  stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request * data =
      (stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__init(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__fini(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__are_equal(const stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response * lhs, const stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__copy(
  const stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response * input,
  stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response *
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response * msg = (stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response *)allocator.allocate(sizeof(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response));
  bool success = stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__destroy(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence__init(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response * data = NULL;

  if (size) {
    data = (stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response *)allocator.zero_allocate(size, sizeof(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__fini(&data[i - 1]);
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
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence__fini(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence * array)
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
      stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__fini(&array->data[i]);
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

stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence *
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence * array = (stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence *)allocator.allocate(sizeof(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence__destroy(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence__are_equal(const stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence * lhs, const stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence__copy(
  const stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence * input,
  stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response * data =
      (stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__init(stretch_web_teleop__action__MoveToPregrasp_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__fini(stretch_web_teleop__action__MoveToPregrasp_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__are_equal(const stretch_web_teleop__action__MoveToPregrasp_GetResult_Request * lhs, const stretch_web_teleop__action__MoveToPregrasp_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__copy(
  const stretch_web_teleop__action__MoveToPregrasp_GetResult_Request * input,
  stretch_web_teleop__action__MoveToPregrasp_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

stretch_web_teleop__action__MoveToPregrasp_GetResult_Request *
stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stretch_web_teleop__action__MoveToPregrasp_GetResult_Request * msg = (stretch_web_teleop__action__MoveToPregrasp_GetResult_Request *)allocator.allocate(sizeof(stretch_web_teleop__action__MoveToPregrasp_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(stretch_web_teleop__action__MoveToPregrasp_GetResult_Request));
  bool success = stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__destroy(stretch_web_teleop__action__MoveToPregrasp_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence__init(stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stretch_web_teleop__action__MoveToPregrasp_GetResult_Request * data = NULL;

  if (size) {
    data = (stretch_web_teleop__action__MoveToPregrasp_GetResult_Request *)allocator.zero_allocate(size, sizeof(stretch_web_teleop__action__MoveToPregrasp_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__fini(&data[i - 1]);
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
stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence__fini(stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence * array)
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
      stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__fini(&array->data[i]);
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

stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence *
stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence * array = (stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence *)allocator.allocate(sizeof(stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence__destroy(stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence__are_equal(const stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence * lhs, const stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence__copy(
  const stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence * input,
  stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(stretch_web_teleop__action__MoveToPregrasp_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    stretch_web_teleop__action__MoveToPregrasp_GetResult_Request * data =
      (stretch_web_teleop__action__MoveToPregrasp_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "stretch_web_teleop/action/detail/move_to_pregrasp__functions.h"

bool
stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__init(stretch_web_teleop__action__MoveToPregrasp_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!stretch_web_teleop__action__MoveToPregrasp_Result__init(&msg->result)) {
    stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__fini(stretch_web_teleop__action__MoveToPregrasp_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  stretch_web_teleop__action__MoveToPregrasp_Result__fini(&msg->result);
}

bool
stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__are_equal(const stretch_web_teleop__action__MoveToPregrasp_GetResult_Response * lhs, const stretch_web_teleop__action__MoveToPregrasp_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!stretch_web_teleop__action__MoveToPregrasp_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__copy(
  const stretch_web_teleop__action__MoveToPregrasp_GetResult_Response * input,
  stretch_web_teleop__action__MoveToPregrasp_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!stretch_web_teleop__action__MoveToPregrasp_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

stretch_web_teleop__action__MoveToPregrasp_GetResult_Response *
stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stretch_web_teleop__action__MoveToPregrasp_GetResult_Response * msg = (stretch_web_teleop__action__MoveToPregrasp_GetResult_Response *)allocator.allocate(sizeof(stretch_web_teleop__action__MoveToPregrasp_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(stretch_web_teleop__action__MoveToPregrasp_GetResult_Response));
  bool success = stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__destroy(stretch_web_teleop__action__MoveToPregrasp_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence__init(stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stretch_web_teleop__action__MoveToPregrasp_GetResult_Response * data = NULL;

  if (size) {
    data = (stretch_web_teleop__action__MoveToPregrasp_GetResult_Response *)allocator.zero_allocate(size, sizeof(stretch_web_teleop__action__MoveToPregrasp_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__fini(&data[i - 1]);
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
stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence__fini(stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence * array)
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
      stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__fini(&array->data[i]);
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

stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence *
stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence * array = (stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence *)allocator.allocate(sizeof(stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence__destroy(stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence__are_equal(const stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence * lhs, const stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence__copy(
  const stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence * input,
  stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(stretch_web_teleop__action__MoveToPregrasp_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    stretch_web_teleop__action__MoveToPregrasp_GetResult_Response * data =
      (stretch_web_teleop__action__MoveToPregrasp_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "stretch_web_teleop/action/detail/move_to_pregrasp__functions.h"

bool
stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__init(stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!stretch_web_teleop__action__MoveToPregrasp_Feedback__init(&msg->feedback)) {
    stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__fini(stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  stretch_web_teleop__action__MoveToPregrasp_Feedback__fini(&msg->feedback);
}

bool
stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__are_equal(const stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage * lhs, const stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!stretch_web_teleop__action__MoveToPregrasp_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__copy(
  const stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage * input,
  stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!stretch_web_teleop__action__MoveToPregrasp_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage *
stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage * msg = (stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage *)allocator.allocate(sizeof(stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage));
  bool success = stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__destroy(stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence__init(stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage * data = NULL;

  if (size) {
    data = (stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage *)allocator.zero_allocate(size, sizeof(stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__fini(&data[i - 1]);
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
stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence__fini(stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence * array)
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
      stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__fini(&array->data[i]);
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

stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence *
stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence * array = (stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence *)allocator.allocate(sizeof(stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence__destroy(stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence__are_equal(const stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence * lhs, const stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence__copy(
  const stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence * input,
  stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage * data =
      (stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
