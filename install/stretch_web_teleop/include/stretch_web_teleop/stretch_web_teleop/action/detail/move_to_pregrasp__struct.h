// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stretch_web_teleop:action/MoveToPregrasp.idl
// generated code does not contain a copyright notice

#ifndef STRETCH_WEB_TELEOP__ACTION__DETAIL__MOVE_TO_PREGRASP__STRUCT_H_
#define STRETCH_WEB_TELEOP__ACTION__DETAIL__MOVE_TO_PREGRASP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'PREGRASP_DIRECTION_AUTO'.
enum
{
  stretch_web_teleop__action__MoveToPregrasp_Goal__PREGRASP_DIRECTION_AUTO = 0
};

/// Constant 'PREGRASP_DIRECTION_HORIZONTAL'.
enum
{
  stretch_web_teleop__action__MoveToPregrasp_Goal__PREGRASP_DIRECTION_HORIZONTAL = 1
};

/// Constant 'PREGRASP_DIRECTION_VERTICAL'.
enum
{
  stretch_web_teleop__action__MoveToPregrasp_Goal__PREGRASP_DIRECTION_VERTICAL = 2
};

/// Struct defined in action/MoveToPregrasp in the package stretch_web_teleop.
typedef struct stretch_web_teleop__action__MoveToPregrasp_Goal
{
  /// These should be in the range [0.0, 1.0]
  double scaled_u;
  double scaled_v;
  uint8_t pregrasp_direction;
} stretch_web_teleop__action__MoveToPregrasp_Goal;

// Struct for a sequence of stretch_web_teleop__action__MoveToPregrasp_Goal.
typedef struct stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence
{
  stretch_web_teleop__action__MoveToPregrasp_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stretch_web_teleop__action__MoveToPregrasp_Goal__Sequence;


// Constants defined in the message

/// Constant 'STATUS_SUCCESS'.
enum
{
  stretch_web_teleop__action__MoveToPregrasp_Result__STATUS_SUCCESS = 0
};

/// Constant 'STATUS_FAILURE'.
enum
{
  stretch_web_teleop__action__MoveToPregrasp_Result__STATUS_FAILURE = 1
};

/// Constant 'STATUS_CANCELLED'.
enum
{
  stretch_web_teleop__action__MoveToPregrasp_Result__STATUS_CANCELLED = 2
};

/// Constant 'STATUS_TIMEOUT'.
enum
{
  stretch_web_teleop__action__MoveToPregrasp_Result__STATUS_TIMEOUT = 3
};

/// Constant 'STATUS_GOAL_NOT_REACHABLE'.
enum
{
  stretch_web_teleop__action__MoveToPregrasp_Result__STATUS_GOAL_NOT_REACHABLE = 4
};

/// Constant 'STATUS_DEPROJECTION_FAILURE'.
enum
{
  stretch_web_teleop__action__MoveToPregrasp_Result__STATUS_DEPROJECTION_FAILURE = 5
};

/// Constant 'STATUS_STRETCH_DRIVER_FAILURE'.
enum
{
  stretch_web_teleop__action__MoveToPregrasp_Result__STATUS_STRETCH_DRIVER_FAILURE = 6
};

/// Constant 'STATUS_JOINTS_IN_COLLISION'.
enum
{
  stretch_web_teleop__action__MoveToPregrasp_Result__STATUS_JOINTS_IN_COLLISION = 7
};

/// Struct defined in action/MoveToPregrasp in the package stretch_web_teleop.
typedef struct stretch_web_teleop__action__MoveToPregrasp_Result
{
  uint8_t status;
} stretch_web_teleop__action__MoveToPregrasp_Result;

// Struct for a sequence of stretch_web_teleop__action__MoveToPregrasp_Result.
typedef struct stretch_web_teleop__action__MoveToPregrasp_Result__Sequence
{
  stretch_web_teleop__action__MoveToPregrasp_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stretch_web_teleop__action__MoveToPregrasp_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'elapsed_time'
#include "builtin_interfaces/msg/detail/duration__struct.h"

/// Struct defined in action/MoveToPregrasp in the package stretch_web_teleop.
typedef struct stretch_web_teleop__action__MoveToPregrasp_Feedback
{
  float initial_distance_m;
  float remaining_distance_m;
  builtin_interfaces__msg__Duration elapsed_time;
} stretch_web_teleop__action__MoveToPregrasp_Feedback;

// Struct for a sequence of stretch_web_teleop__action__MoveToPregrasp_Feedback.
typedef struct stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence
{
  stretch_web_teleop__action__MoveToPregrasp_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stretch_web_teleop__action__MoveToPregrasp_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "stretch_web_teleop/action/detail/move_to_pregrasp__struct.h"

/// Struct defined in action/MoveToPregrasp in the package stretch_web_teleop.
typedef struct stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  stretch_web_teleop__action__MoveToPregrasp_Goal goal;
} stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request;

// Struct for a sequence of stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request.
typedef struct stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence
{
  stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stretch_web_teleop__action__MoveToPregrasp_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/MoveToPregrasp in the package stretch_web_teleop.
typedef struct stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response;

// Struct for a sequence of stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response.
typedef struct stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence
{
  stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stretch_web_teleop__action__MoveToPregrasp_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/MoveToPregrasp in the package stretch_web_teleop.
typedef struct stretch_web_teleop__action__MoveToPregrasp_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} stretch_web_teleop__action__MoveToPregrasp_GetResult_Request;

// Struct for a sequence of stretch_web_teleop__action__MoveToPregrasp_GetResult_Request.
typedef struct stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence
{
  stretch_web_teleop__action__MoveToPregrasp_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stretch_web_teleop__action__MoveToPregrasp_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "stretch_web_teleop/action/detail/move_to_pregrasp__struct.h"

/// Struct defined in action/MoveToPregrasp in the package stretch_web_teleop.
typedef struct stretch_web_teleop__action__MoveToPregrasp_GetResult_Response
{
  int8_t status;
  stretch_web_teleop__action__MoveToPregrasp_Result result;
} stretch_web_teleop__action__MoveToPregrasp_GetResult_Response;

// Struct for a sequence of stretch_web_teleop__action__MoveToPregrasp_GetResult_Response.
typedef struct stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence
{
  stretch_web_teleop__action__MoveToPregrasp_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stretch_web_teleop__action__MoveToPregrasp_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "stretch_web_teleop/action/detail/move_to_pregrasp__struct.h"

/// Struct defined in action/MoveToPregrasp in the package stretch_web_teleop.
typedef struct stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  stretch_web_teleop__action__MoveToPregrasp_Feedback feedback;
} stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage;

// Struct for a sequence of stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage.
typedef struct stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence
{
  stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stretch_web_teleop__action__MoveToPregrasp_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STRETCH_WEB_TELEOP__ACTION__DETAIL__MOVE_TO_PREGRASP__STRUCT_H_
