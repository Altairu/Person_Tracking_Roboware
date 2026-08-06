# generated from rosidl_generator_py/resource/_idl.py.em
# with input from altair_interfaces:msg/MddFeedback.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'angles'
# Member 'speeds'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MddFeedback(type):
    """Metaclass of message 'MddFeedback'."""

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
            module = import_type_support('altair_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'altair_interfaces.msg.MddFeedback')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__mdd_feedback
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__mdd_feedback
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__mdd_feedback
            cls._TYPE_SUPPORT = module.type_support_msg__msg__mdd_feedback
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__mdd_feedback

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MddFeedback(metaclass=Metaclass_MddFeedback):
    """Message class 'MddFeedback'."""

    __slots__ = [
        '_limit_switches',
        '_error_code',
        '_system_status',
        '_angles',
        '_speeds',
    ]

    _fields_and_field_types = {
        'limit_switches': 'boolean[4]',
        'error_code': 'uint8',
        'system_status': 'uint8',
        'angles': 'float[4]',
        'speeds': 'float[4]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('boolean'), 4),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 4),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 4),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.limit_switches = kwargs.get(
            'limit_switches',
            [bool() for x in range(4)]
        )
        self.error_code = kwargs.get('error_code', int())
        self.system_status = kwargs.get('system_status', int())
        if 'angles' not in kwargs:
            self.angles = numpy.zeros(4, dtype=numpy.float32)
        else:
            self.angles = kwargs.get('angles')
        if 'speeds' not in kwargs:
            self.speeds = numpy.zeros(4, dtype=numpy.float32)
        else:
            self.speeds = kwargs.get('speeds')

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
        if self.limit_switches != other.limit_switches:
            return False
        if self.error_code != other.error_code:
            return False
        if self.system_status != other.system_status:
            return False
        if any(self.angles != other.angles):
            return False
        if any(self.speeds != other.speeds):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def limit_switches(self):
        """Message field 'limit_switches'."""
        return self._limit_switches

    @limit_switches.setter
    def limit_switches(self, value):
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 4 and
                 all(isinstance(v, bool) for v in value) and
                 True), \
                "The 'limit_switches' field must be a set or sequence with length 4 and each value of type 'bool'"
        self._limit_switches = value

    @builtins.property
    def error_code(self):
        """Message field 'error_code'."""
        return self._error_code

    @error_code.setter
    def error_code(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'error_code' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'error_code' field must be an unsigned integer in [0, 255]"
        self._error_code = value

    @builtins.property
    def system_status(self):
        """Message field 'system_status'."""
        return self._system_status

    @system_status.setter
    def system_status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'system_status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'system_status' field must be an unsigned integer in [0, 255]"
        self._system_status = value

    @builtins.property
    def angles(self):
        """Message field 'angles'."""
        return self._angles

    @angles.setter
    def angles(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'angles' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 4, \
                "The 'angles' numpy.ndarray() must have a size of 4"
            self._angles = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 4 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'angles' field must be a set or sequence with length 4 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._angles = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def speeds(self):
        """Message field 'speeds'."""
        return self._speeds

    @speeds.setter
    def speeds(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'speeds' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 4, \
                "The 'speeds' numpy.ndarray() must have a size of 4"
            self._speeds = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 4 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'speeds' field must be a set or sequence with length 4 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._speeds = numpy.array(value, dtype=numpy.float32)
