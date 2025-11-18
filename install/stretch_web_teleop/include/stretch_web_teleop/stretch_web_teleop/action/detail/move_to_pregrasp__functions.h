// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from stretch_web_teleop:action/MoveToPregrasp.idl
// generated code does not contain a copyright notice

#ifndef STRETCH_WEB_TELEOP__ACTION__DETAIL__MOVE_TO_PREGRASP__FUNCTIONS_H_
#define STRETCH_WEB_TELEOP__ACTION__DETAIL__MOVE_TO_PREGRASP__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "stretch_web_teleop/msg/rosidl_generator_c__visibility_control.h"

#include "stretch_web_teleop/action/detail/move_to_pregrasp__struct.h"

/// Initialize action/MoveToPregrasp message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * stretch_web_teleop__action__MoveToPregrasp_Goal
 * )) before or use
 * stretch_web_teleop__action__MoveToPregrasp_Goal__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_Goal__init(stretch_web_teleop__action__MoveToPregrasp_Goal * msg);

/// Finalize action/MoveToPregrasp message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_Goal__fini(stretch_web_teleop__action__MoveToPregrasp_Goal * msg);

/// Create action/MoveToPregrasp message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * stretch_web_teleop__action__MoveToPregrasp_Goal__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
stretch_web_teleop__action__MoveToPregrasp_Goal *
stretch_web_teleop__action__MoveToPregrasp_Goal__create();

/// Destroy action/MoveToPregrasp message.
/**
 * It calls
 * stretch_web_teleop__action__MoveToPregrasp_Goal__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_Goal__destroy(stretch_web_teleop__action__MoveToPregrasp_Goal * msg);

/// Check for action/MoveToPregrasp message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_Goal__are_equal(const stretch_web_teleop__action__MoveToPregrasp_Goal * lhs, const stretch_web_teleop__action__MoveToPregrasp_Goal * rhs);

/// Copy a action/MoveToPregrasp message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_Goal__copy(
  const stretch_web_teleop__action__MoveToPregrasp_Goal * input,
  stretch_web_teleop__action__MoveToPregrasp_Goal * output);

/// Initialize array of action/MoveToPregrasp messages.
/**
 * It allocates the memory for the number of elements and calls
 * stretch_web_teleop__action__MoveToPregrasp_Goal__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence__init(stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence * array, size_t size);

/// Finalize array of action/MoveToPregrasp messages.
/**
 * It calls
 * stretch_web_teleop__action__MoveToPregrasp_Goal__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence__fini(stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence * array);

/// Create array of action/MoveToPregrasp messages.
/**
 * It allocates the memory for the array and calls
 * stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence *
stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence__create(size_t size);

/// Destroy array of action/MoveToPregrasp messages.
/**
 * It calls
 * stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence__destroy(stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence * array);

/// Check for action/MoveToPregrasp message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence__are_equal(const stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence * lhs, const stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence * rhs);

/// Copy an array of action/MoveToPregrasp messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence__copy(
  const stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence * input,
  stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence * output);

/// Initialize action/MoveToPregrasp message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * stretch_web_teleop__action__MoveToPregrasp_Result
 * )) before or use
 * stretch_web_teleop__action__MoveToPregrasp_Result__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_Result__init(stretch_web_teleop__action__MoveToPregrasp_Result * msg);

/// Finalize action/MoveToPregrasp message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_Result__fini(stretch_web_teleop__action__MoveToPregrasp_Result * msg);

/// Create action/MoveToPregrasp message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * stretch_web_teleop__action__MoveToPregrasp_Result__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
stretch_web_teleop__action__MoveToPregrasp_Result *
stretch_web_teleop__action__MoveToPregrasp_Result__create();

/// Destroy action/MoveToPregrasp message.
/**
 * It calls
 * stretch_web_teleop__action__MoveToPregrasp_Result__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_Result__destroy(stretch_web_teleop__action__MoveToPregrasp_Result * msg);

/// Check for action/MoveToPregrasp message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_Result__are_equal(const stretch_web_teleop__action__MoveToPregrasp_Result * lhs, const stretch_web_teleop__action__MoveToPregrasp_Result * rhs);

/// Copy a action/MoveToPregrasp message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_Result__copy(
  const stretch_web_teleop__action__MoveToPregrasp_Result * input,
  stretch_web_teleop__action__MoveToPregrasp_Result * output);

/// Initialize array of action/MoveToPregrasp messages.
/**
 * It allocates the memory for the number of elements and calls
 * stretch_web_teleop__action__MoveToPregrasp_Result__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_Result__Sequence__init(stretch_web_teleop__action__MoveToPregrasp_Result__Sequence * array, size_t size);

/// Finalize array of action/MoveToPregrasp messages.
/**
 * It calls
 * stretch_web_teleop__action__MoveToPregrasp_Result__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_Result__Sequence__fini(stretch_web_teleop__action__MoveToPregrasp_Result__Sequence * array);

/// Create array of action/MoveToPregrasp messages.
/**
 * It allocates the memory for the array and calls
 * stretch_web_teleop__action__MoveToPregrasp_Result__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
stretch_web_teleop__action__MoveToPregrasp_Result__Sequence *
stretch_web_teleop__action__MoveToPregrasp_Result__Sequence__create(size_t size);

/// Destroy array of action/MoveToPregrasp messages.
/**
 * It calls
 * stretch_web_teleop__action__MoveToPregrasp_Result__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_Result__Sequence__destroy(stretch_web_teleop__action__MoveToPregrasp_Result__Sequence * array);

/// Check for action/MoveToPregrasp message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_Result__Sequence__are_equal(const stretch_web_teleop__action__MoveToPregrasp_Result__Sequence * lhs, const stretch_web_teleop__action__MoveToPregrasp_Result__Sequence * rhs);

/// Copy an array of action/MoveToPregrasp messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_Result__Sequence__copy(
  const stretch_web_teleop__action__MoveToPregrasp_Result__Sequence * input,
  stretch_web_teleop__action__MoveToPregrasp_Result__Sequence * output);

/// Initialize action/MoveToPregrasp message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * stretch_web_teleop__action__MoveToPregrasp_Feedback
 * )) before or use
 * stretch_web_teleop__action__MoveToPregrasp_Feedback__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_Feedback__init(stretch_web_teleop__action__MoveToPregrasp_Feedback * msg);

/// Finalize action/MoveToPregrasp message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_Feedback__fini(stretch_web_teleop__action__MoveToPregrasp_Feedback * msg);

/// Create action/MoveToPregrasp message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * stretch_web_teleop__action__MoveToPregrasp_Feedback__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
stretch_web_teleop__action__MoveToPregrasp_Feedback *
stretch_web_teleop__action__MoveToPregrasp_Feedback__create();

/// Destroy action/MoveToPregrasp message.
/**
 * It calls
 * stretch_web_teleop__action__MoveToPregrasp_Feedback__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_Feedback__destroy(stretch_web_teleop__action__MoveToPregrasp_Feedback * msg);

/// Check for action/MoveToPregrasp message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_Feedback__are_equal(const stretch_web_teleop__action__MoveToPregrasp_Feedback * lhs, const stretch_web_teleop__action__MoveToPregrasp_Feedback * rhs);

/// Copy a action/MoveToPregrasp message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_Feedback__copy(
  const stretch_web_teleop__action__MoveToPregrasp_Feedback * input,
  stretch_web_teleop__action__MoveToPregrasp_Feedback * output);

/// Initialize array of action/MoveToPregrasp messages.
/**
 * It allocates the memory for the number of elements and calls
 * stretch_web_teleop__action__MoveToPregrasp_Feedback__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence__init(stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence * array, size_t size);

/// Finalize array of action/MoveToPregrasp messages.
/**
 * It calls
 * stretch_web_teleop__action__MoveToPregrasp_Feedback__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence__fini(stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence * array);

/// Create array of action/MoveToPregrasp messages.
/**
 * It allocates the memory for the array and calls
 * stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence *
stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence__create(size_t size);

/// Destroy array of action/MoveToPregrasp messages.
/**
 * It calls
 * stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence__destroy(stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence * array);

/// Check for action/MoveToPregrasp message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence__are_equal(const stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence * lhs, const stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence * rhs);

/// Copy an array of action/MoveToPregrasp messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence__copy(
  const stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence * input,
  stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence * output);

/// Initialize action/MoveToPregrasp message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request
 * )) before or use
 * stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__init(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request * msg);

/// Finalize action/MoveToPregrasp message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__fini(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request * msg);

/// Create action/MoveToPregrasp message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request *
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__create();

/// Destroy action/MoveToPregrasp message.
/**
 * It calls
 * stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__destroy(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request * msg);

/// Check for action/MoveToPregrasp message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__are_equal(const stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request * lhs, const stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request * rhs);

/// Copy a action/MoveToPregrasp message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__copy(
  const stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request * input,
  stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request * output);

/// Initialize array of action/MoveToPregrasp messages.
/**
 * It allocates the memory for the number of elements and calls
 * stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence__init(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence * array, size_t size);

/// Finalize array of action/MoveToPregrasp messages.
/**
 * It calls
 * stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence__fini(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence * array);

/// Create array of action/MoveToPregrasp messages.
/**
 * It allocates the memory for the array and calls
 * stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence *
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence__create(size_t size);

/// Destroy array of action/MoveToPregrasp messages.
/**
 * It calls
 * stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence__destroy(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence * array);

/// Check for action/MoveToPregrasp message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence__are_equal(const stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence * lhs, const stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence * rhs);

/// Copy an array of action/MoveToPregrasp messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence__copy(
  const stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence * input,
  stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence * output);

/// Initialize action/MoveToPregrasp message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response
 * )) before or use
 * stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__init(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response * msg);

/// Finalize action/MoveToPregrasp message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__fini(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response * msg);

/// Create action/MoveToPregrasp message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response *
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__create();

/// Destroy action/MoveToPregrasp message.
/**
 * It calls
 * stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__destroy(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response * msg);

/// Check for action/MoveToPregrasp message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__are_equal(const stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response * lhs, const stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response * rhs);

/// Copy a action/MoveToPregrasp message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__copy(
  const stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response * input,
  stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response * output);

/// Initialize array of action/MoveToPregrasp messages.
/**
 * It allocates the memory for the number of elements and calls
 * stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence__init(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence * array, size_t size);

/// Finalize array of action/MoveToPregrasp messages.
/**
 * It calls
 * stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence__fini(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence * array);

/// Create array of action/MoveToPregrasp messages.
/**
 * It allocates the memory for the array and calls
 * stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence *
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence__create(size_t size);

/// Destroy array of action/MoveToPregrasp messages.
/**
 * It calls
 * stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence__destroy(stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence * array);

/// Check for action/MoveToPregrasp message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence__are_equal(const stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence * lhs, const stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence * rhs);

/// Copy an array of action/MoveToPregrasp messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence__copy(
  const stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence * input,
  stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence * output);

/// Initialize action/MoveToPregrasp message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * stretch_web_teleop__action__MoveToPregrasp_GetResult_Request
 * )) before or use
 * stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__init(stretch_web_teleop__action__MoveToPregrasp_GetResult_Request * msg);

/// Finalize action/MoveToPregrasp message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__fini(stretch_web_teleop__action__MoveToPregrasp_GetResult_Request * msg);

/// Create action/MoveToPregrasp message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
stretch_web_teleop__action__MoveToPregrasp_GetResult_Request *
stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__create();

/// Destroy action/MoveToPregrasp message.
/**
 * It calls
 * stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__destroy(stretch_web_teleop__action__MoveToPregrasp_GetResult_Request * msg);

/// Check for action/MoveToPregrasp message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__are_equal(const stretch_web_teleop__action__MoveToPregrasp_GetResult_Request * lhs, const stretch_web_teleop__action__MoveToPregrasp_GetResult_Request * rhs);

/// Copy a action/MoveToPregrasp message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__copy(
  const stretch_web_teleop__action__MoveToPregrasp_GetResult_Request * input,
  stretch_web_teleop__action__MoveToPregrasp_GetResult_Request * output);

/// Initialize array of action/MoveToPregrasp messages.
/**
 * It allocates the memory for the number of elements and calls
 * stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence__init(stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence * array, size_t size);

/// Finalize array of action/MoveToPregrasp messages.
/**
 * It calls
 * stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence__fini(stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence * array);

/// Create array of action/MoveToPregrasp messages.
/**
 * It allocates the memory for the array and calls
 * stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence *
stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence__create(size_t size);

/// Destroy array of action/MoveToPregrasp messages.
/**
 * It calls
 * stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence__destroy(stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence * array);

/// Check for action/MoveToPregrasp message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence__are_equal(const stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence * lhs, const stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence * rhs);

/// Copy an array of action/MoveToPregrasp messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence__copy(
  const stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence * input,
  stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence * output);

/// Initialize action/MoveToPregrasp message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * stretch_web_teleop__action__MoveToPregrasp_GetResult_Response
 * )) before or use
 * stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__init(stretch_web_teleop__action__MoveToPregrasp_GetResult_Response * msg);

/// Finalize action/MoveToPregrasp message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__fini(stretch_web_teleop__action__MoveToPregrasp_GetResult_Response * msg);

/// Create action/MoveToPregrasp message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
stretch_web_teleop__action__MoveToPregrasp_GetResult_Response *
stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__create();

/// Destroy action/MoveToPregrasp message.
/**
 * It calls
 * stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__destroy(stretch_web_teleop__action__MoveToPregrasp_GetResult_Response * msg);

/// Check for action/MoveToPregrasp message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__are_equal(const stretch_web_teleop__action__MoveToPregrasp_GetResult_Response * lhs, const stretch_web_teleop__action__MoveToPregrasp_GetResult_Response * rhs);

/// Copy a action/MoveToPregrasp message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__copy(
  const stretch_web_teleop__action__MoveToPregrasp_GetResult_Response * input,
  stretch_web_teleop__action__MoveToPregrasp_GetResult_Response * output);

/// Initialize array of action/MoveToPregrasp messages.
/**
 * It allocates the memory for the number of elements and calls
 * stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence__init(stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence * array, size_t size);

/// Finalize array of action/MoveToPregrasp messages.
/**
 * It calls
 * stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence__fini(stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence * array);

/// Create array of action/MoveToPregrasp messages.
/**
 * It allocates the memory for the array and calls
 * stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence *
stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence__create(size_t size);

/// Destroy array of action/MoveToPregrasp messages.
/**
 * It calls
 * stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence__destroy(stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence * array);

/// Check for action/MoveToPregrasp message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence__are_equal(const stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence * lhs, const stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence * rhs);

/// Copy an array of action/MoveToPregrasp messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence__copy(
  const stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence * input,
  stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence * output);

/// Initialize action/MoveToPregrasp message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage
 * )) before or use
 * stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__init(stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage * msg);

/// Finalize action/MoveToPregrasp message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__fini(stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage * msg);

/// Create action/MoveToPregrasp message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage *
stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__create();

/// Destroy action/MoveToPregrasp message.
/**
 * It calls
 * stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__destroy(stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage * msg);

/// Check for action/MoveToPregrasp message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__are_equal(const stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage * lhs, const stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage * rhs);

/// Copy a action/MoveToPregrasp message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__copy(
  const stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage * input,
  stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage * output);

/// Initialize array of action/MoveToPregrasp messages.
/**
 * It allocates the memory for the number of elements and calls
 * stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence__init(stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence * array, size_t size);

/// Finalize array of action/MoveToPregrasp messages.
/**
 * It calls
 * stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence__fini(stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence * array);

/// Create array of action/MoveToPregrasp messages.
/**
 * It allocates the memory for the array and calls
 * stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence *
stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence__create(size_t size);

/// Destroy array of action/MoveToPregrasp messages.
/**
 * It calls
 * stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
void
stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence__destroy(stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence * array);

/// Check for action/MoveToPregrasp message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence__are_equal(const stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence * lhs, const stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence * rhs);

/// Copy an array of action/MoveToPregrasp messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_stretch_web_teleop
bool
stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence__copy(
  const stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence * input,
  stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // STRETCH_WEB_TELEOP__ACTION__DETAIL__MOVE_TO_PREGRASP__FUNCTIONS_H_
