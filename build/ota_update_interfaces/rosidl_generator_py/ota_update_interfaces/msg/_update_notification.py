# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ota_update_interfaces:msg/UpdateNotification.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_UpdateNotification(type):
    """Metaclass of message 'UpdateNotification'."""

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
            module = import_type_support('ota_update_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ota_update_interfaces.msg.UpdateNotification')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__update_notification
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__update_notification
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__update_notification
            cls._TYPE_SUPPORT = module.type_support_msg__msg__update_notification
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__update_notification

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class UpdateNotification(metaclass=Metaclass_UpdateNotification):
    """Message class 'UpdateNotification'."""

    __slots__ = [
        '_target',
        '_version',
        '_file_path',
        '_file_size',
        '_checksum',
    ]

    _fields_and_field_types = {
        'target': 'string',
        'version': 'string',
        'file_path': 'string',
        'file_size': 'uint64',
        'checksum': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.target = kwargs.get('target', str())
        self.version = kwargs.get('version', str())
        self.file_path = kwargs.get('file_path', str())
        self.file_size = kwargs.get('file_size', int())
        self.checksum = kwargs.get('checksum', str())

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
        if self.target != other.target:
            return False
        if self.version != other.version:
            return False
        if self.file_path != other.file_path:
            return False
        if self.file_size != other.file_size:
            return False
        if self.checksum != other.checksum:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def target(self):
        """Message field 'target'."""
        return self._target

    @target.setter
    def target(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'target' field must be of type 'str'"
        self._target = value

    @builtins.property
    def version(self):
        """Message field 'version'."""
        return self._version

    @version.setter
    def version(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'version' field must be of type 'str'"
        self._version = value

    @builtins.property
    def file_path(self):
        """Message field 'file_path'."""
        return self._file_path

    @file_path.setter
    def file_path(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'file_path' field must be of type 'str'"
        self._file_path = value

    @builtins.property
    def file_size(self):
        """Message field 'file_size'."""
        return self._file_size

    @file_size.setter
    def file_size(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'file_size' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'file_size' field must be an unsigned integer in [0, 18446744073709551615]"
        self._file_size = value

    @builtins.property
    def checksum(self):
        """Message field 'checksum'."""
        return self._checksum

    @checksum.setter
    def checksum(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'checksum' field must be of type 'str'"
        self._checksum = value
