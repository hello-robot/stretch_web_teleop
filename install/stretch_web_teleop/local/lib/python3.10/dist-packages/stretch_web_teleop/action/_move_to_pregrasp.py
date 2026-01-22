# generated from rosidl_generator_py/resource/_idl.py.em
# with input from stretch_web_teleop:action/MoveToPregrasp.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MoveToPregrasp_Goal(type):
    """Metaclass of message 'MoveToPregrasp_Goal'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'PREGRASP_DIRECTION_AUTO': 0,
        'PREGRASP_DIRECTION_HORIZONTAL': 1,
        'PREGRASP_DIRECTION_VERTICAL': 2,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('stretch_web_teleop')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'stretch_web_teleop.action.MoveToPregrasp_Goal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__move_to_pregrasp__goal
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__move_to_pregrasp__goal
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__move_to_pregrasp__goal
            cls._TYPE_SUPPORT = module.type_support_msg__action__move_to_pregrasp__goal
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__move_to_pregrasp__goal

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'PREGRASP_DIRECTION_AUTO': cls.__constants['PREGRASP_DIRECTION_AUTO'],
            'PREGRASP_DIRECTION_HORIZONTAL': cls.__constants['PREGRASP_DIRECTION_HORIZONTAL'],
            'PREGRASP_DIRECTION_VERTICAL': cls.__constants['PREGRASP_DIRECTION_VERTICAL'],
        }

    @property
    def PREGRASP_DIRECTION_AUTO(self):
        """Message constant 'PREGRASP_DIRECTION_AUTO'."""
        return Metaclass_MoveToPregrasp_Goal.__constants['PREGRASP_DIRECTION_AUTO']

    @property
    def PREGRASP_DIRECTION_HORIZONTAL(self):
        """Message constant 'PREGRASP_DIRECTION_HORIZONTAL'."""
        return Metaclass_MoveToPregrasp_Goal.__constants['PREGRASP_DIRECTION_HORIZONTAL']

    @property
    def PREGRASP_DIRECTION_VERTICAL(self):
        """Message constant 'PREGRASP_DIRECTION_VERTICAL'."""
        return Metaclass_MoveToPregrasp_Goal.__constants['PREGRASP_DIRECTION_VERTICAL']


class MoveToPregrasp_Goal(metaclass=Metaclass_MoveToPregrasp_Goal):
    """
    Message class 'MoveToPregrasp_Goal'.

    Constants:
      PREGRASP_DIRECTION_AUTO
      PREGRASP_DIRECTION_HORIZONTAL
      PREGRASP_DIRECTION_VERTICAL
    """

    __slots__ = [
        '_scaled_u',
        '_scaled_v',
        '_pregrasp_direction',
    ]

    _fields_and_field_types = {
        'scaled_u': 'double',
        'scaled_v': 'double',
        'pregrasp_direction': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.scaled_u = kwargs.get('scaled_u', float())
        self.scaled_v = kwargs.get('scaled_v', float())
        self.pregrasp_direction = kwargs.get('pregrasp_direction', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.scaled_u != other.scaled_u:
            return False
        if self.scaled_v != other.scaled_v:
            return False
        if self.pregrasp_direction != other.pregrasp_direction:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def scaled_u(self):
        """Message field 'scaled_u'."""
        return self._scaled_u

    @scaled_u.setter
    def scaled_u(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'scaled_u' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'scaled_u' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._scaled_u = value

    @builtins.property
    def scaled_v(self):
        """Message field 'scaled_v'."""
        return self._scaled_v

    @scaled_v.setter
    def scaled_v(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'scaled_v' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'scaled_v' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._scaled_v = value

    @builtins.property
    def pregrasp_direction(self):
        """Message field 'pregrasp_direction'."""
        return self._pregrasp_direction

    @pregrasp_direction.setter
    def pregrasp_direction(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'pregrasp_direction' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'pregrasp_direction' field must be an unsigned integer in [0, 255]"
        self._pregrasp_direction = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_MoveToPregrasp_Result(type):
    """Metaclass of message 'MoveToPregrasp_Result'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'STATUS_SUCCESS': 0,
        'STATUS_FAILURE': 1,
        'STATUS_CANCELLED': 2,
        'STATUS_TIMEOUT': 3,
        'STATUS_GOAL_NOT_REACHABLE': 4,
        'STATUS_DEPROJECTION_FAILURE': 5,
        'STATUS_STRETCH_DRIVER_FAILURE': 6,
        'STATUS_JOINTS_IN_COLLISION': 7,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('stretch_web_teleop')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'stretch_web_teleop.action.MoveToPregrasp_Result')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__move_to_pregrasp__result
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__move_to_pregrasp__result
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__move_to_pregrasp__result
            cls._TYPE_SUPPORT = module.type_support_msg__action__move_to_pregrasp__result
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__move_to_pregrasp__result

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'STATUS_SUCCESS': cls.__constants['STATUS_SUCCESS'],
            'STATUS_FAILURE': cls.__constants['STATUS_FAILURE'],
            'STATUS_CANCELLED': cls.__constants['STATUS_CANCELLED'],
            'STATUS_TIMEOUT': cls.__constants['STATUS_TIMEOUT'],
            'STATUS_GOAL_NOT_REACHABLE': cls.__constants['STATUS_GOAL_NOT_REACHABLE'],
            'STATUS_DEPROJECTION_FAILURE': cls.__constants['STATUS_DEPROJECTION_FAILURE'],
            'STATUS_STRETCH_DRIVER_FAILURE': cls.__constants['STATUS_STRETCH_DRIVER_FAILURE'],
            'STATUS_JOINTS_IN_COLLISION': cls.__constants['STATUS_JOINTS_IN_COLLISION'],
        }

    @property
    def STATUS_SUCCESS(self):
        """Message constant 'STATUS_SUCCESS'."""
        return Metaclass_MoveToPregrasp_Result.__constants['STATUS_SUCCESS']

    @property
    def STATUS_FAILURE(self):
        """Message constant 'STATUS_FAILURE'."""
        return Metaclass_MoveToPregrasp_Result.__constants['STATUS_FAILURE']

    @property
    def STATUS_CANCELLED(self):
        """Message constant 'STATUS_CANCELLED'."""
        return Metaclass_MoveToPregrasp_Result.__constants['STATUS_CANCELLED']

    @property
    def STATUS_TIMEOUT(self):
        """Message constant 'STATUS_TIMEOUT'."""
        return Metaclass_MoveToPregrasp_Result.__constants['STATUS_TIMEOUT']

    @property
    def STATUS_GOAL_NOT_REACHABLE(self):
        """Message constant 'STATUS_GOAL_NOT_REACHABLE'."""
        return Metaclass_MoveToPregrasp_Result.__constants['STATUS_GOAL_NOT_REACHABLE']

    @property
    def STATUS_DEPROJECTION_FAILURE(self):
        """Message constant 'STATUS_DEPROJECTION_FAILURE'."""
        return Metaclass_MoveToPregrasp_Result.__constants['STATUS_DEPROJECTION_FAILURE']

    @property
    def STATUS_STRETCH_DRIVER_FAILURE(self):
        """Message constant 'STATUS_STRETCH_DRIVER_FAILURE'."""
        return Metaclass_MoveToPregrasp_Result.__constants['STATUS_STRETCH_DRIVER_FAILURE']

    @property
    def STATUS_JOINTS_IN_COLLISION(self):
        """Message constant 'STATUS_JOINTS_IN_COLLISION'."""
        return Metaclass_MoveToPregrasp_Result.__constants['STATUS_JOINTS_IN_COLLISION']


class MoveToPregrasp_Result(metaclass=Metaclass_MoveToPregrasp_Result):
    """
    Message class 'MoveToPregrasp_Result'.

    Constants:
      STATUS_SUCCESS
      STATUS_FAILURE
      STATUS_CANCELLED
      STATUS_TIMEOUT
      STATUS_GOAL_NOT_REACHABLE
      STATUS_DEPROJECTION_FAILURE
      STATUS_STRETCH_DRIVER_FAILURE
      STATUS_JOINTS_IN_COLLISION
    """

    __slots__ = [
        '_status',
    ]

    _fields_and_field_types = {
        'status': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.status = kwargs.get('status', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.status != other.status:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'status' field must be an unsigned integer in [0, 255]"
        self._status = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import math

# already imported above
# import rosidl_parser.definition


class Metaclass_MoveToPregrasp_Feedback(type):
    """Metaclass of message 'MoveToPregrasp_Feedback'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('stretch_web_teleop')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'stretch_web_teleop.action.MoveToPregrasp_Feedback')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__move_to_pregrasp__feedback
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__move_to_pregrasp__feedback
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__move_to_pregrasp__feedback
            cls._TYPE_SUPPORT = module.type_support_msg__action__move_to_pregrasp__feedback
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__move_to_pregrasp__feedback

            from builtin_interfaces.msg import Duration
            if Duration.__class__._TYPE_SUPPORT is None:
                Duration.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MoveToPregrasp_Feedback(metaclass=Metaclass_MoveToPregrasp_Feedback):
    """Message class 'MoveToPregrasp_Feedback'."""

    __slots__ = [
        '_initial_distance_m',
        '_remaining_distance_m',
        '_elapsed_time',
    ]

    _fields_and_field_types = {
        'initial_distance_m': 'float',
        'remaining_distance_m': 'float',
        'elapsed_time': 'builtin_interfaces/Duration',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Duration'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.initial_distance_m = kwargs.get('initial_distance_m', float())
        self.remaining_distance_m = kwargs.get('remaining_distance_m', float())
        from builtin_interfaces.msg import Duration
        self.elapsed_time = kwargs.get('elapsed_time', Duration())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.initial_distance_m != other.initial_distance_m:
            return False
        if self.remaining_distance_m != other.remaining_distance_m:
            return False
        if self.elapsed_time != other.elapsed_time:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def initial_distance_m(self):
        """Message field 'initial_distance_m'."""
        return self._initial_distance_m

    @initial_distance_m.setter
    def initial_distance_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'initial_distance_m' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'initial_distance_m' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._initial_distance_m = value

    @builtins.property
    def remaining_distance_m(self):
        """Message field 'remaining_distance_m'."""
        return self._remaining_distance_m

    @remaining_distance_m.setter
    def remaining_distance_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'remaining_distance_m' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'remaining_distance_m' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._remaining_distance_m = value

    @builtins.property
    def elapsed_time(self):
        """Message field 'elapsed_time'."""
        return self._elapsed_time

    @elapsed_time.setter
    def elapsed_time(self, value):
        if __debug__:
            from builtin_interfaces.msg import Duration
            assert \
                isinstance(value, Duration), \
                "The 'elapsed_time' field must be a sub message of type 'Duration'"
        self._elapsed_time = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_MoveToPregrasp_SendGoal_Request(type):
    """Metaclass of message 'MoveToPregrasp_SendGoal_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('stretch_web_teleop')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'stretch_web_teleop.action.MoveToPregrasp_SendGoal_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__move_to_pregrasp__send_goal__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__move_to_pregrasp__send_goal__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__move_to_pregrasp__send_goal__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__move_to_pregrasp__send_goal__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__move_to_pregrasp__send_goal__request

            from stretch_web_teleop.action import MoveToPregrasp
            if MoveToPregrasp.Goal.__class__._TYPE_SUPPORT is None:
                MoveToPregrasp.Goal.__class__.__import_type_support__()

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MoveToPregrasp_SendGoal_Request(metaclass=Metaclass_MoveToPregrasp_SendGoal_Request):
    """Message class 'MoveToPregrasp_SendGoal_Request'."""

    __slots__ = [
        '_goal_id',
        '_goal',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'goal': 'stretch_web_teleop/MoveToPregrasp_Goal',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['stretch_web_teleop', 'action'], 'MoveToPregrasp_Goal'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from stretch_web_teleop.action._move_to_pregrasp import MoveToPregrasp_Goal
        self.goal = kwargs.get('goal', MoveToPregrasp_Goal())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.goal_id != other.goal_id:
            return False
        if self.goal != other.goal:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def goal(self):
        """Message field 'goal'."""
        return self._goal

    @goal.setter
    def goal(self, value):
        if __debug__:
            from stretch_web_teleop.action._move_to_pregrasp import MoveToPregrasp_Goal
            assert \
                isinstance(value, MoveToPregrasp_Goal), \
                "The 'goal' field must be a sub message of type 'MoveToPregrasp_Goal'"
        self._goal = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_MoveToPregrasp_SendGoal_Response(type):
    """Metaclass of message 'MoveToPregrasp_SendGoal_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('stretch_web_teleop')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'stretch_web_teleop.action.MoveToPregrasp_SendGoal_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__move_to_pregrasp__send_goal__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__move_to_pregrasp__send_goal__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__move_to_pregrasp__send_goal__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__move_to_pregrasp__send_goal__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__move_to_pregrasp__send_goal__response

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MoveToPregrasp_SendGoal_Response(metaclass=Metaclass_MoveToPregrasp_SendGoal_Response):
    """Message class 'MoveToPregrasp_SendGoal_Response'."""

    __slots__ = [
        '_accepted',
        '_stamp',
    ]

    _fields_and_field_types = {
        'accepted': 'boolean',
        'stamp': 'builtin_interfaces/Time',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.accepted = kwargs.get('accepted', bool())
        from builtin_interfaces.msg import Time
        self.stamp = kwargs.get('stamp', Time())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.accepted != other.accepted:
            return False
        if self.stamp != other.stamp:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def accepted(self):
        """Message field 'accepted'."""
        return self._accepted

    @accepted.setter
    def accepted(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'accepted' field must be of type 'bool'"
        self._accepted = value

    @builtins.property
    def stamp(self):
        """Message field 'stamp'."""
        return self._stamp

    @stamp.setter
    def stamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'stamp' field must be a sub message of type 'Time'"
        self._stamp = value


class Metaclass_MoveToPregrasp_SendGoal(type):
    """Metaclass of service 'MoveToPregrasp_SendGoal'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('stretch_web_teleop')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'stretch_web_teleop.action.MoveToPregrasp_SendGoal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__move_to_pregrasp__send_goal

            from stretch_web_teleop.action import _move_to_pregrasp
            if _move_to_pregrasp.Metaclass_MoveToPregrasp_SendGoal_Request._TYPE_SUPPORT is None:
                _move_to_pregrasp.Metaclass_MoveToPregrasp_SendGoal_Request.__import_type_support__()
            if _move_to_pregrasp.Metaclass_MoveToPregrasp_SendGoal_Response._TYPE_SUPPORT is None:
                _move_to_pregrasp.Metaclass_MoveToPregrasp_SendGoal_Response.__import_type_support__()


class MoveToPregrasp_SendGoal(metaclass=Metaclass_MoveToPregrasp_SendGoal):
    from stretch_web_teleop.action._move_to_pregrasp import MoveToPregrasp_SendGoal_Request as Request
    from stretch_web_teleop.action._move_to_pregrasp import MoveToPregrasp_SendGoal_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_MoveToPregrasp_GetResult_Request(type):
    """Metaclass of message 'MoveToPregrasp_GetResult_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('stretch_web_teleop')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'stretch_web_teleop.action.MoveToPregrasp_GetResult_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__move_to_pregrasp__get_result__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__move_to_pregrasp__get_result__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__move_to_pregrasp__get_result__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__move_to_pregrasp__get_result__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__move_to_pregrasp__get_result__request

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MoveToPregrasp_GetResult_Request(metaclass=Metaclass_MoveToPregrasp_GetResult_Request):
    """Message class 'MoveToPregrasp_GetResult_Request'."""

    __slots__ = [
        '_goal_id',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.goal_id != other.goal_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_MoveToPregrasp_GetResult_Response(type):
    """Metaclass of message 'MoveToPregrasp_GetResult_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('stretch_web_teleop')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'stretch_web_teleop.action.MoveToPregrasp_GetResult_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__move_to_pregrasp__get_result__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__move_to_pregrasp__get_result__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__move_to_pregrasp__get_result__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__move_to_pregrasp__get_result__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__move_to_pregrasp__get_result__response

            from stretch_web_teleop.action import MoveToPregrasp
            if MoveToPregrasp.Result.__class__._TYPE_SUPPORT is None:
                MoveToPregrasp.Result.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MoveToPregrasp_GetResult_Response(metaclass=Metaclass_MoveToPregrasp_GetResult_Response):
    """Message class 'MoveToPregrasp_GetResult_Response'."""

    __slots__ = [
        '_status',
        '_result',
    ]

    _fields_and_field_types = {
        'status': 'int8',
        'result': 'stretch_web_teleop/MoveToPregrasp_Result',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['stretch_web_teleop', 'action'], 'MoveToPregrasp_Result'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.status = kwargs.get('status', int())
        from stretch_web_teleop.action._move_to_pregrasp import MoveToPregrasp_Result
        self.result = kwargs.get('result', MoveToPregrasp_Result())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.status != other.status:
            return False
        if self.result != other.result:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'status' field must be an integer in [-128, 127]"
        self._status = value

    @builtins.property
    def result(self):
        """Message field 'result'."""
        return self._result

    @result.setter
    def result(self, value):
        if __debug__:
            from stretch_web_teleop.action._move_to_pregrasp import MoveToPregrasp_Result
            assert \
                isinstance(value, MoveToPregrasp_Result), \
                "The 'result' field must be a sub message of type 'MoveToPregrasp_Result'"
        self._result = value


class Metaclass_MoveToPregrasp_GetResult(type):
    """Metaclass of service 'MoveToPregrasp_GetResult'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('stretch_web_teleop')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'stretch_web_teleop.action.MoveToPregrasp_GetResult')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__move_to_pregrasp__get_result

            from stretch_web_teleop.action import _move_to_pregrasp
            if _move_to_pregrasp.Metaclass_MoveToPregrasp_GetResult_Request._TYPE_SUPPORT is None:
                _move_to_pregrasp.Metaclass_MoveToPregrasp_GetResult_Request.__import_type_support__()
            if _move_to_pregrasp.Metaclass_MoveToPregrasp_GetResult_Response._TYPE_SUPPORT is None:
                _move_to_pregrasp.Metaclass_MoveToPregrasp_GetResult_Response.__import_type_support__()


class MoveToPregrasp_GetResult(metaclass=Metaclass_MoveToPregrasp_GetResult):
    from stretch_web_teleop.action._move_to_pregrasp import MoveToPregrasp_GetResult_Request as Request
    from stretch_web_teleop.action._move_to_pregrasp import MoveToPregrasp_GetResult_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_MoveToPregrasp_FeedbackMessage(type):
    """Metaclass of message 'MoveToPregrasp_FeedbackMessage'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('stretch_web_teleop')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'stretch_web_teleop.action.MoveToPregrasp_FeedbackMessage')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__move_to_pregrasp__feedback_message
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__move_to_pregrasp__feedback_message
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__move_to_pregrasp__feedback_message
            cls._TYPE_SUPPORT = module.type_support_msg__action__move_to_pregrasp__feedback_message
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__move_to_pregrasp__feedback_message

            from stretch_web_teleop.action import MoveToPregrasp
            if MoveToPregrasp.Feedback.__class__._TYPE_SUPPORT is None:
                MoveToPregrasp.Feedback.__class__.__import_type_support__()

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MoveToPregrasp_FeedbackMessage(metaclass=Metaclass_MoveToPregrasp_FeedbackMessage):
    """Message class 'MoveToPregrasp_FeedbackMessage'."""

    __slots__ = [
        '_goal_id',
        '_feedback',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'feedback': 'stretch_web_teleop/MoveToPregrasp_Feedback',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['stretch_web_teleop', 'action'], 'MoveToPregrasp_Feedback'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from stretch_web_teleop.action._move_to_pregrasp import MoveToPregrasp_Feedback
        self.feedback = kwargs.get('feedback', MoveToPregrasp_Feedback())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.goal_id != other.goal_id:
            return False
        if self.feedback != other.feedback:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def feedback(self):
        """Message field 'feedback'."""
        return self._feedback

    @feedback.setter
    def feedback(self, value):
        if __debug__:
            from stretch_web_teleop.action._move_to_pregrasp import MoveToPregrasp_Feedback
            assert \
                isinstance(value, MoveToPregrasp_Feedback), \
                "The 'feedback' field must be a sub message of type 'MoveToPregrasp_Feedback'"
        self._feedback = value


class Metaclass_MoveToPregrasp(type):
    """Metaclass of action 'MoveToPregrasp'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('stretch_web_teleop')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'stretch_web_teleop.action.MoveToPregrasp')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_action__action__move_to_pregrasp

            from action_msgs.msg import _goal_status_array
            if _goal_status_array.Metaclass_GoalStatusArray._TYPE_SUPPORT is None:
                _goal_status_array.Metaclass_GoalStatusArray.__import_type_support__()
            from action_msgs.srv import _cancel_goal
            if _cancel_goal.Metaclass_CancelGoal._TYPE_SUPPORT is None:
                _cancel_goal.Metaclass_CancelGoal.__import_type_support__()

            from stretch_web_teleop.action import _move_to_pregrasp
            if _move_to_pregrasp.Metaclass_MoveToPregrasp_SendGoal._TYPE_SUPPORT is None:
                _move_to_pregrasp.Metaclass_MoveToPregrasp_SendGoal.__import_type_support__()
            if _move_to_pregrasp.Metaclass_MoveToPregrasp_GetResult._TYPE_SUPPORT is None:
                _move_to_pregrasp.Metaclass_MoveToPregrasp_GetResult.__import_type_support__()
            if _move_to_pregrasp.Metaclass_MoveToPregrasp_FeedbackMessage._TYPE_SUPPORT is None:
                _move_to_pregrasp.Metaclass_MoveToPregrasp_FeedbackMessage.__import_type_support__()


class MoveToPregrasp(metaclass=Metaclass_MoveToPregrasp):

    # The goal message defined in the action definition.
    from stretch_web_teleop.action._move_to_pregrasp import MoveToPregrasp_Goal as Goal
    # The result message defined in the action definition.
    from stretch_web_teleop.action._move_to_pregrasp import MoveToPregrasp_Result as Result
    # The feedback message defined in the action definition.
    from stretch_web_teleop.action._move_to_pregrasp import MoveToPregrasp_Feedback as Feedback

    class Impl:

        # The send_goal service using a wrapped version of the goal message as a request.
        from stretch_web_teleop.action._move_to_pregrasp import MoveToPregrasp_SendGoal as SendGoalService
        # The get_result service using a wrapped version of the result message as a response.
        from stretch_web_teleop.action._move_to_pregrasp import MoveToPregrasp_GetResult as GetResultService
        # The feedback message with generic fields which wraps the feedback message.
        from stretch_web_teleop.action._move_to_pregrasp import MoveToPregrasp_FeedbackMessage as FeedbackMessage

        # The generic service to cancel a goal.
        from action_msgs.srv._cancel_goal import CancelGoal as CancelGoalService
        # The generic message for get the status of a goal.
        from action_msgs.msg._goal_status_array import GoalStatusArray as GoalStatusMessage

    def __init__(self):
        raise NotImplementedError('Action classes can not be instantiated')
