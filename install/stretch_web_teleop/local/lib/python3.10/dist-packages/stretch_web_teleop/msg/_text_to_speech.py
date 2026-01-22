# generated from rosidl_generator_py/resource/_idl.py.em
# with input from stretch_web_teleop:msg/TextToSpeech.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TextToSpeech(type):
    """Metaclass of message 'TextToSpeech'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'OVERRIDE_BEHAVIOR_QUEUE': 0,
        'OVERRIDE_BEHAVIOR_INTERRUPT': 1,
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
                'stretch_web_teleop.msg.TextToSpeech')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__text_to_speech
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__text_to_speech
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__text_to_speech
            cls._TYPE_SUPPORT = module.type_support_msg__msg__text_to_speech
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__text_to_speech

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'OVERRIDE_BEHAVIOR_QUEUE': cls.__constants['OVERRIDE_BEHAVIOR_QUEUE'],
            'OVERRIDE_BEHAVIOR_INTERRUPT': cls.__constants['OVERRIDE_BEHAVIOR_INTERRUPT'],
        }

    @property
    def OVERRIDE_BEHAVIOR_QUEUE(self):
        """Message constant 'OVERRIDE_BEHAVIOR_QUEUE'."""
        return Metaclass_TextToSpeech.__constants['OVERRIDE_BEHAVIOR_QUEUE']

    @property
    def OVERRIDE_BEHAVIOR_INTERRUPT(self):
        """Message constant 'OVERRIDE_BEHAVIOR_INTERRUPT'."""
        return Metaclass_TextToSpeech.__constants['OVERRIDE_BEHAVIOR_INTERRUPT']


class TextToSpeech(metaclass=Metaclass_TextToSpeech):
    """
    Message class 'TextToSpeech'.

    Constants:
      OVERRIDE_BEHAVIOR_QUEUE
      OVERRIDE_BEHAVIOR_INTERRUPT
    """

    __slots__ = [
        '_text',
        '_voice',
        '_is_slow',
        '_override_behavior',
    ]

    _fields_and_field_types = {
        'text': 'string',
        'voice': 'string',
        'is_slow': 'boolean',
        'override_behavior': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.text = kwargs.get('text', str())
        self.voice = kwargs.get('voice', str())
        self.is_slow = kwargs.get('is_slow', bool())
        self.override_behavior = kwargs.get('override_behavior', int())

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
        if self.text != other.text:
            return False
        if self.voice != other.voice:
            return False
        if self.is_slow != other.is_slow:
            return False
        if self.override_behavior != other.override_behavior:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def text(self):
        """Message field 'text'."""
        return self._text

    @text.setter
    def text(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'text' field must be of type 'str'"
        self._text = value

    @builtins.property
    def voice(self):
        """Message field 'voice'."""
        return self._voice

    @voice.setter
    def voice(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'voice' field must be of type 'str'"
        self._voice = value

    @builtins.property
    def is_slow(self):
        """Message field 'is_slow'."""
        return self._is_slow

    @is_slow.setter
    def is_slow(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_slow' field must be of type 'bool'"
        self._is_slow = value

    @builtins.property
    def override_behavior(self):
        """Message field 'override_behavior'."""
        return self._override_behavior

    @override_behavior.setter
    def override_behavior(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'override_behavior' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'override_behavior' field must be an unsigned integer in [0, 255]"
        self._override_behavior = value
