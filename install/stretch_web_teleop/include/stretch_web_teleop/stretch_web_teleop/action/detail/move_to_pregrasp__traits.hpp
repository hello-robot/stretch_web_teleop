// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from stretch_web_teleop:action/MoveToPregrasp.idl
// generated code does not contain a copyright notice

#ifndef STRETCH_WEB_TELEOP__ACTION__DETAIL__MOVE_TO_PREGRASP__TRAITS_HPP_
#define STRETCH_WEB_TELEOP__ACTION__DETAIL__MOVE_TO_PREGRASP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "stretch_web_teleop/action/detail/move_to_pregrasp__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace stretch_web_teleop
{

namespace action
{

inline void to_flow_style_yaml(
  const MoveToPregrasp_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: scaled_u
  {
    out << "scaled_u: ";
    rosidl_generator_traits::value_to_yaml(msg.scaled_u, out);
    out << ", ";
  }

  // member: scaled_v
  {
    out << "scaled_v: ";
    rosidl_generator_traits::value_to_yaml(msg.scaled_v, out);
    out << ", ";
  }

  // member: pregrasp_direction
  {
    out << "pregrasp_direction: ";
    rosidl_generator_traits::value_to_yaml(msg.pregrasp_direction, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveToPregrasp_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: scaled_u
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "scaled_u: ";
    rosidl_generator_traits::value_to_yaml(msg.scaled_u, out);
    out << "\n";
  }

  // member: scaled_v
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "scaled_v: ";
    rosidl_generator_traits::value_to_yaml(msg.scaled_v, out);
    out << "\n";
  }

  // member: pregrasp_direction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pregrasp_direction: ";
    rosidl_generator_traits::value_to_yaml(msg.pregrasp_direction, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveToPregrasp_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace stretch_web_teleop

namespace rosidl_generator_traits
{

[[deprecated("use stretch_web_teleop::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const stretch_web_teleop::action::MoveToPregrasp_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  stretch_web_teleop::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stretch_web_teleop::action::to_yaml() instead")]]
inline std::string to_yaml(const stretch_web_teleop::action::MoveToPregrasp_Goal & msg)
{
  return stretch_web_teleop::action::to_yaml(msg);
}

template<>
inline const char * data_type<stretch_web_teleop::action::MoveToPregrasp_Goal>()
{
  return "stretch_web_teleop::action::MoveToPregrasp_Goal";
}

template<>
inline const char * name<stretch_web_teleop::action::MoveToPregrasp_Goal>()
{
  return "stretch_web_teleop/action/MoveToPregrasp_Goal";
}

template<>
struct has_fixed_size<stretch_web_teleop::action::MoveToPregrasp_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<stretch_web_teleop::action::MoveToPregrasp_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<stretch_web_teleop::action::MoveToPregrasp_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace stretch_web_teleop
{

namespace action
{

inline void to_flow_style_yaml(
  const MoveToPregrasp_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveToPregrasp_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveToPregrasp_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace stretch_web_teleop

namespace rosidl_generator_traits
{

[[deprecated("use stretch_web_teleop::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const stretch_web_teleop::action::MoveToPregrasp_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  stretch_web_teleop::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stretch_web_teleop::action::to_yaml() instead")]]
inline std::string to_yaml(const stretch_web_teleop::action::MoveToPregrasp_Result & msg)
{
  return stretch_web_teleop::action::to_yaml(msg);
}

template<>
inline const char * data_type<stretch_web_teleop::action::MoveToPregrasp_Result>()
{
  return "stretch_web_teleop::action::MoveToPregrasp_Result";
}

template<>
inline const char * name<stretch_web_teleop::action::MoveToPregrasp_Result>()
{
  return "stretch_web_teleop/action/MoveToPregrasp_Result";
}

template<>
struct has_fixed_size<stretch_web_teleop::action::MoveToPregrasp_Result>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<stretch_web_teleop::action::MoveToPregrasp_Result>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<stretch_web_teleop::action::MoveToPregrasp_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'elapsed_time'
#include "builtin_interfaces/msg/detail/duration__traits.hpp"

namespace stretch_web_teleop
{

namespace action
{

inline void to_flow_style_yaml(
  const MoveToPregrasp_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: initial_distance_m
  {
    out << "initial_distance_m: ";
    rosidl_generator_traits::value_to_yaml(msg.initial_distance_m, out);
    out << ", ";
  }

  // member: remaining_distance_m
  {
    out << "remaining_distance_m: ";
    rosidl_generator_traits::value_to_yaml(msg.remaining_distance_m, out);
    out << ", ";
  }

  // member: elapsed_time
  {
    out << "elapsed_time: ";
    to_flow_style_yaml(msg.elapsed_time, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveToPregrasp_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: initial_distance_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "initial_distance_m: ";
    rosidl_generator_traits::value_to_yaml(msg.initial_distance_m, out);
    out << "\n";
  }

  // member: remaining_distance_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "remaining_distance_m: ";
    rosidl_generator_traits::value_to_yaml(msg.remaining_distance_m, out);
    out << "\n";
  }

  // member: elapsed_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "elapsed_time:\n";
    to_block_style_yaml(msg.elapsed_time, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveToPregrasp_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace stretch_web_teleop

namespace rosidl_generator_traits
{

[[deprecated("use stretch_web_teleop::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const stretch_web_teleop::action::MoveToPregrasp_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  stretch_web_teleop::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stretch_web_teleop::action::to_yaml() instead")]]
inline std::string to_yaml(const stretch_web_teleop::action::MoveToPregrasp_Feedback & msg)
{
  return stretch_web_teleop::action::to_yaml(msg);
}

template<>
inline const char * data_type<stretch_web_teleop::action::MoveToPregrasp_Feedback>()
{
  return "stretch_web_teleop::action::MoveToPregrasp_Feedback";
}

template<>
inline const char * name<stretch_web_teleop::action::MoveToPregrasp_Feedback>()
{
  return "stretch_web_teleop/action/MoveToPregrasp_Feedback";
}

template<>
struct has_fixed_size<stretch_web_teleop::action::MoveToPregrasp_Feedback>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Duration>::value> {};

template<>
struct has_bounded_size<stretch_web_teleop::action::MoveToPregrasp_Feedback>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Duration>::value> {};

template<>
struct is_message<stretch_web_teleop::action::MoveToPregrasp_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "stretch_web_teleop/action/detail/move_to_pregrasp__traits.hpp"

namespace stretch_web_teleop
{

namespace action
{

inline void to_flow_style_yaml(
  const MoveToPregrasp_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveToPregrasp_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveToPregrasp_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace stretch_web_teleop

namespace rosidl_generator_traits
{

[[deprecated("use stretch_web_teleop::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const stretch_web_teleop::action::MoveToPregrasp_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  stretch_web_teleop::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stretch_web_teleop::action::to_yaml() instead")]]
inline std::string to_yaml(const stretch_web_teleop::action::MoveToPregrasp_SendGoal_Request & msg)
{
  return stretch_web_teleop::action::to_yaml(msg);
}

template<>
inline const char * data_type<stretch_web_teleop::action::MoveToPregrasp_SendGoal_Request>()
{
  return "stretch_web_teleop::action::MoveToPregrasp_SendGoal_Request";
}

template<>
inline const char * name<stretch_web_teleop::action::MoveToPregrasp_SendGoal_Request>()
{
  return "stretch_web_teleop/action/MoveToPregrasp_SendGoal_Request";
}

template<>
struct has_fixed_size<stretch_web_teleop::action::MoveToPregrasp_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<stretch_web_teleop::action::MoveToPregrasp_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<stretch_web_teleop::action::MoveToPregrasp_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<stretch_web_teleop::action::MoveToPregrasp_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<stretch_web_teleop::action::MoveToPregrasp_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace stretch_web_teleop
{

namespace action
{

inline void to_flow_style_yaml(
  const MoveToPregrasp_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveToPregrasp_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveToPregrasp_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace stretch_web_teleop

namespace rosidl_generator_traits
{

[[deprecated("use stretch_web_teleop::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const stretch_web_teleop::action::MoveToPregrasp_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  stretch_web_teleop::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stretch_web_teleop::action::to_yaml() instead")]]
inline std::string to_yaml(const stretch_web_teleop::action::MoveToPregrasp_SendGoal_Response & msg)
{
  return stretch_web_teleop::action::to_yaml(msg);
}

template<>
inline const char * data_type<stretch_web_teleop::action::MoveToPregrasp_SendGoal_Response>()
{
  return "stretch_web_teleop::action::MoveToPregrasp_SendGoal_Response";
}

template<>
inline const char * name<stretch_web_teleop::action::MoveToPregrasp_SendGoal_Response>()
{
  return "stretch_web_teleop/action/MoveToPregrasp_SendGoal_Response";
}

template<>
struct has_fixed_size<stretch_web_teleop::action::MoveToPregrasp_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<stretch_web_teleop::action::MoveToPregrasp_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<stretch_web_teleop::action::MoveToPregrasp_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<stretch_web_teleop::action::MoveToPregrasp_SendGoal>()
{
  return "stretch_web_teleop::action::MoveToPregrasp_SendGoal";
}

template<>
inline const char * name<stretch_web_teleop::action::MoveToPregrasp_SendGoal>()
{
  return "stretch_web_teleop/action/MoveToPregrasp_SendGoal";
}

template<>
struct has_fixed_size<stretch_web_teleop::action::MoveToPregrasp_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<stretch_web_teleop::action::MoveToPregrasp_SendGoal_Request>::value &&
    has_fixed_size<stretch_web_teleop::action::MoveToPregrasp_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<stretch_web_teleop::action::MoveToPregrasp_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<stretch_web_teleop::action::MoveToPregrasp_SendGoal_Request>::value &&
    has_bounded_size<stretch_web_teleop::action::MoveToPregrasp_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<stretch_web_teleop::action::MoveToPregrasp_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<stretch_web_teleop::action::MoveToPregrasp_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<stretch_web_teleop::action::MoveToPregrasp_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace stretch_web_teleop
{

namespace action
{

inline void to_flow_style_yaml(
  const MoveToPregrasp_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveToPregrasp_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveToPregrasp_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace stretch_web_teleop

namespace rosidl_generator_traits
{

[[deprecated("use stretch_web_teleop::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const stretch_web_teleop::action::MoveToPregrasp_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  stretch_web_teleop::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stretch_web_teleop::action::to_yaml() instead")]]
inline std::string to_yaml(const stretch_web_teleop::action::MoveToPregrasp_GetResult_Request & msg)
{
  return stretch_web_teleop::action::to_yaml(msg);
}

template<>
inline const char * data_type<stretch_web_teleop::action::MoveToPregrasp_GetResult_Request>()
{
  return "stretch_web_teleop::action::MoveToPregrasp_GetResult_Request";
}

template<>
inline const char * name<stretch_web_teleop::action::MoveToPregrasp_GetResult_Request>()
{
  return "stretch_web_teleop/action/MoveToPregrasp_GetResult_Request";
}

template<>
struct has_fixed_size<stretch_web_teleop::action::MoveToPregrasp_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<stretch_web_teleop::action::MoveToPregrasp_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<stretch_web_teleop::action::MoveToPregrasp_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "stretch_web_teleop/action/detail/move_to_pregrasp__traits.hpp"

namespace stretch_web_teleop
{

namespace action
{

inline void to_flow_style_yaml(
  const MoveToPregrasp_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveToPregrasp_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveToPregrasp_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace stretch_web_teleop

namespace rosidl_generator_traits
{

[[deprecated("use stretch_web_teleop::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const stretch_web_teleop::action::MoveToPregrasp_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  stretch_web_teleop::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stretch_web_teleop::action::to_yaml() instead")]]
inline std::string to_yaml(const stretch_web_teleop::action::MoveToPregrasp_GetResult_Response & msg)
{
  return stretch_web_teleop::action::to_yaml(msg);
}

template<>
inline const char * data_type<stretch_web_teleop::action::MoveToPregrasp_GetResult_Response>()
{
  return "stretch_web_teleop::action::MoveToPregrasp_GetResult_Response";
}

template<>
inline const char * name<stretch_web_teleop::action::MoveToPregrasp_GetResult_Response>()
{
  return "stretch_web_teleop/action/MoveToPregrasp_GetResult_Response";
}

template<>
struct has_fixed_size<stretch_web_teleop::action::MoveToPregrasp_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<stretch_web_teleop::action::MoveToPregrasp_Result>::value> {};

template<>
struct has_bounded_size<stretch_web_teleop::action::MoveToPregrasp_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<stretch_web_teleop::action::MoveToPregrasp_Result>::value> {};

template<>
struct is_message<stretch_web_teleop::action::MoveToPregrasp_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<stretch_web_teleop::action::MoveToPregrasp_GetResult>()
{
  return "stretch_web_teleop::action::MoveToPregrasp_GetResult";
}

template<>
inline const char * name<stretch_web_teleop::action::MoveToPregrasp_GetResult>()
{
  return "stretch_web_teleop/action/MoveToPregrasp_GetResult";
}

template<>
struct has_fixed_size<stretch_web_teleop::action::MoveToPregrasp_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<stretch_web_teleop::action::MoveToPregrasp_GetResult_Request>::value &&
    has_fixed_size<stretch_web_teleop::action::MoveToPregrasp_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<stretch_web_teleop::action::MoveToPregrasp_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<stretch_web_teleop::action::MoveToPregrasp_GetResult_Request>::value &&
    has_bounded_size<stretch_web_teleop::action::MoveToPregrasp_GetResult_Response>::value
  >
{
};

template<>
struct is_service<stretch_web_teleop::action::MoveToPregrasp_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<stretch_web_teleop::action::MoveToPregrasp_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<stretch_web_teleop::action::MoveToPregrasp_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "stretch_web_teleop/action/detail/move_to_pregrasp__traits.hpp"

namespace stretch_web_teleop
{

namespace action
{

inline void to_flow_style_yaml(
  const MoveToPregrasp_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveToPregrasp_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveToPregrasp_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace stretch_web_teleop

namespace rosidl_generator_traits
{

[[deprecated("use stretch_web_teleop::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const stretch_web_teleop::action::MoveToPregrasp_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  stretch_web_teleop::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stretch_web_teleop::action::to_yaml() instead")]]
inline std::string to_yaml(const stretch_web_teleop::action::MoveToPregrasp_FeedbackMessage & msg)
{
  return stretch_web_teleop::action::to_yaml(msg);
}

template<>
inline const char * data_type<stretch_web_teleop::action::MoveToPregrasp_FeedbackMessage>()
{
  return "stretch_web_teleop::action::MoveToPregrasp_FeedbackMessage";
}

template<>
inline const char * name<stretch_web_teleop::action::MoveToPregrasp_FeedbackMessage>()
{
  return "stretch_web_teleop/action/MoveToPregrasp_FeedbackMessage";
}

template<>
struct has_fixed_size<stretch_web_teleop::action::MoveToPregrasp_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<stretch_web_teleop::action::MoveToPregrasp_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<stretch_web_teleop::action::MoveToPregrasp_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<stretch_web_teleop::action::MoveToPregrasp_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<stretch_web_teleop::action::MoveToPregrasp_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<stretch_web_teleop::action::MoveToPregrasp>
  : std::true_type
{
};

template<>
struct is_action_goal<stretch_web_teleop::action::MoveToPregrasp_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<stretch_web_teleop::action::MoveToPregrasp_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<stretch_web_teleop::action::MoveToPregrasp_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // STRETCH_WEB_TELEOP__ACTION__DETAIL__MOVE_TO_PREGRASP__TRAITS_HPP_
