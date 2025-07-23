// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from stretch_web_teleop:action/MoveToPregrasp.idl
// generated code does not contain a copyright notice

#ifndef STRETCH_WEB_TELEOP__ACTION__DETAIL__MOVE_TO_PREGRASP__BUILDER_HPP_
#define STRETCH_WEB_TELEOP__ACTION__DETAIL__MOVE_TO_PREGRASP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "stretch_web_teleop/action/detail/move_to_pregrasp__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace stretch_web_teleop
{

namespace action
{

namespace builder
{

class Init_MoveToPregrasp_Goal_pregrasp_direction
{
public:
  explicit Init_MoveToPregrasp_Goal_pregrasp_direction(::stretch_web_teleop::action::MoveToPregrasp_Goal & msg)
  : msg_(msg)
  {}
  ::stretch_web_teleop::action::MoveToPregrasp_Goal pregrasp_direction(::stretch_web_teleop::action::MoveToPregrasp_Goal::_pregrasp_direction_type arg)
  {
    msg_.pregrasp_direction = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stretch_web_teleop::action::MoveToPregrasp_Goal msg_;
};

class Init_MoveToPregrasp_Goal_scaled_v
{
public:
  explicit Init_MoveToPregrasp_Goal_scaled_v(::stretch_web_teleop::action::MoveToPregrasp_Goal & msg)
  : msg_(msg)
  {}
  Init_MoveToPregrasp_Goal_pregrasp_direction scaled_v(::stretch_web_teleop::action::MoveToPregrasp_Goal::_scaled_v_type arg)
  {
    msg_.scaled_v = std::move(arg);
    return Init_MoveToPregrasp_Goal_pregrasp_direction(msg_);
  }

private:
  ::stretch_web_teleop::action::MoveToPregrasp_Goal msg_;
};

class Init_MoveToPregrasp_Goal_scaled_u
{
public:
  Init_MoveToPregrasp_Goal_scaled_u()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToPregrasp_Goal_scaled_v scaled_u(::stretch_web_teleop::action::MoveToPregrasp_Goal::_scaled_u_type arg)
  {
    msg_.scaled_u = std::move(arg);
    return Init_MoveToPregrasp_Goal_scaled_v(msg_);
  }

private:
  ::stretch_web_teleop::action::MoveToPregrasp_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::stretch_web_teleop::action::MoveToPregrasp_Goal>()
{
  return stretch_web_teleop::action::builder::Init_MoveToPregrasp_Goal_scaled_u();
}

}  // namespace stretch_web_teleop


namespace stretch_web_teleop
{

namespace action
{

namespace builder
{

class Init_MoveToPregrasp_Result_status
{
public:
  Init_MoveToPregrasp_Result_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::stretch_web_teleop::action::MoveToPregrasp_Result status(::stretch_web_teleop::action::MoveToPregrasp_Result::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stretch_web_teleop::action::MoveToPregrasp_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::stretch_web_teleop::action::MoveToPregrasp_Result>()
{
  return stretch_web_teleop::action::builder::Init_MoveToPregrasp_Result_status();
}

}  // namespace stretch_web_teleop


namespace stretch_web_teleop
{

namespace action
{

namespace builder
{

class Init_MoveToPregrasp_Feedback_elapsed_time
{
public:
  explicit Init_MoveToPregrasp_Feedback_elapsed_time(::stretch_web_teleop::action::MoveToPregrasp_Feedback & msg)
  : msg_(msg)
  {}
  ::stretch_web_teleop::action::MoveToPregrasp_Feedback elapsed_time(::stretch_web_teleop::action::MoveToPregrasp_Feedback::_elapsed_time_type arg)
  {
    msg_.elapsed_time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stretch_web_teleop::action::MoveToPregrasp_Feedback msg_;
};

class Init_MoveToPregrasp_Feedback_remaining_distance_m
{
public:
  explicit Init_MoveToPregrasp_Feedback_remaining_distance_m(::stretch_web_teleop::action::MoveToPregrasp_Feedback & msg)
  : msg_(msg)
  {}
  Init_MoveToPregrasp_Feedback_elapsed_time remaining_distance_m(::stretch_web_teleop::action::MoveToPregrasp_Feedback::_remaining_distance_m_type arg)
  {
    msg_.remaining_distance_m = std::move(arg);
    return Init_MoveToPregrasp_Feedback_elapsed_time(msg_);
  }

private:
  ::stretch_web_teleop::action::MoveToPregrasp_Feedback msg_;
};

class Init_MoveToPregrasp_Feedback_initial_distance_m
{
public:
  Init_MoveToPregrasp_Feedback_initial_distance_m()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToPregrasp_Feedback_remaining_distance_m initial_distance_m(::stretch_web_teleop::action::MoveToPregrasp_Feedback::_initial_distance_m_type arg)
  {
    msg_.initial_distance_m = std::move(arg);
    return Init_MoveToPregrasp_Feedback_remaining_distance_m(msg_);
  }

private:
  ::stretch_web_teleop::action::MoveToPregrasp_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::stretch_web_teleop::action::MoveToPregrasp_Feedback>()
{
  return stretch_web_teleop::action::builder::Init_MoveToPregrasp_Feedback_initial_distance_m();
}

}  // namespace stretch_web_teleop


namespace stretch_web_teleop
{

namespace action
{

namespace builder
{

class Init_MoveToPregrasp_SendGoal_Request_goal
{
public:
  explicit Init_MoveToPregrasp_SendGoal_Request_goal(::stretch_web_teleop::action::MoveToPregrasp_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::stretch_web_teleop::action::MoveToPregrasp_SendGoal_Request goal(::stretch_web_teleop::action::MoveToPregrasp_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stretch_web_teleop::action::MoveToPregrasp_SendGoal_Request msg_;
};

class Init_MoveToPregrasp_SendGoal_Request_goal_id
{
public:
  Init_MoveToPregrasp_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToPregrasp_SendGoal_Request_goal goal_id(::stretch_web_teleop::action::MoveToPregrasp_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_MoveToPregrasp_SendGoal_Request_goal(msg_);
  }

private:
  ::stretch_web_teleop::action::MoveToPregrasp_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::stretch_web_teleop::action::MoveToPregrasp_SendGoal_Request>()
{
  return stretch_web_teleop::action::builder::Init_MoveToPregrasp_SendGoal_Request_goal_id();
}

}  // namespace stretch_web_teleop


namespace stretch_web_teleop
{

namespace action
{

namespace builder
{

class Init_MoveToPregrasp_SendGoal_Response_stamp
{
public:
  explicit Init_MoveToPregrasp_SendGoal_Response_stamp(::stretch_web_teleop::action::MoveToPregrasp_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::stretch_web_teleop::action::MoveToPregrasp_SendGoal_Response stamp(::stretch_web_teleop::action::MoveToPregrasp_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stretch_web_teleop::action::MoveToPregrasp_SendGoal_Response msg_;
};

class Init_MoveToPregrasp_SendGoal_Response_accepted
{
public:
  Init_MoveToPregrasp_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToPregrasp_SendGoal_Response_stamp accepted(::stretch_web_teleop::action::MoveToPregrasp_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_MoveToPregrasp_SendGoal_Response_stamp(msg_);
  }

private:
  ::stretch_web_teleop::action::MoveToPregrasp_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::stretch_web_teleop::action::MoveToPregrasp_SendGoal_Response>()
{
  return stretch_web_teleop::action::builder::Init_MoveToPregrasp_SendGoal_Response_accepted();
}

}  // namespace stretch_web_teleop


namespace stretch_web_teleop
{

namespace action
{

namespace builder
{

class Init_MoveToPregrasp_GetResult_Request_goal_id
{
public:
  Init_MoveToPregrasp_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::stretch_web_teleop::action::MoveToPregrasp_GetResult_Request goal_id(::stretch_web_teleop::action::MoveToPregrasp_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stretch_web_teleop::action::MoveToPregrasp_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::stretch_web_teleop::action::MoveToPregrasp_GetResult_Request>()
{
  return stretch_web_teleop::action::builder::Init_MoveToPregrasp_GetResult_Request_goal_id();
}

}  // namespace stretch_web_teleop


namespace stretch_web_teleop
{

namespace action
{

namespace builder
{

class Init_MoveToPregrasp_GetResult_Response_result
{
public:
  explicit Init_MoveToPregrasp_GetResult_Response_result(::stretch_web_teleop::action::MoveToPregrasp_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::stretch_web_teleop::action::MoveToPregrasp_GetResult_Response result(::stretch_web_teleop::action::MoveToPregrasp_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stretch_web_teleop::action::MoveToPregrasp_GetResult_Response msg_;
};

class Init_MoveToPregrasp_GetResult_Response_status
{
public:
  Init_MoveToPregrasp_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToPregrasp_GetResult_Response_result status(::stretch_web_teleop::action::MoveToPregrasp_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_MoveToPregrasp_GetResult_Response_result(msg_);
  }

private:
  ::stretch_web_teleop::action::MoveToPregrasp_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::stretch_web_teleop::action::MoveToPregrasp_GetResult_Response>()
{
  return stretch_web_teleop::action::builder::Init_MoveToPregrasp_GetResult_Response_status();
}

}  // namespace stretch_web_teleop


namespace stretch_web_teleop
{

namespace action
{

namespace builder
{

class Init_MoveToPregrasp_FeedbackMessage_feedback
{
public:
  explicit Init_MoveToPregrasp_FeedbackMessage_feedback(::stretch_web_teleop::action::MoveToPregrasp_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::stretch_web_teleop::action::MoveToPregrasp_FeedbackMessage feedback(::stretch_web_teleop::action::MoveToPregrasp_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stretch_web_teleop::action::MoveToPregrasp_FeedbackMessage msg_;
};

class Init_MoveToPregrasp_FeedbackMessage_goal_id
{
public:
  Init_MoveToPregrasp_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToPregrasp_FeedbackMessage_feedback goal_id(::stretch_web_teleop::action::MoveToPregrasp_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_MoveToPregrasp_FeedbackMessage_feedback(msg_);
  }

private:
  ::stretch_web_teleop::action::MoveToPregrasp_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::stretch_web_teleop::action::MoveToPregrasp_FeedbackMessage>()
{
  return stretch_web_teleop::action::builder::Init_MoveToPregrasp_FeedbackMessage_goal_id();
}

}  // namespace stretch_web_teleop

#endif  // STRETCH_WEB_TELEOP__ACTION__DETAIL__MOVE_TO_PREGRASP__BUILDER_HPP_
