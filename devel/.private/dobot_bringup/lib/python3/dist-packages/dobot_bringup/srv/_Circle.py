# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from dobot_bringup/CircleRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class CircleRequest(genpy.Message):
  _md5sum = "5f601c879842fb49a2308443b5eaa00b"
  _type = "dobot_bringup/CircleRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """int32 count
float64 x1
float64 y1
float64 z1
float64 a1
float64 b1
float64 c1

float64 x2
float64 y2
float64 z2
float64 a2
float64 b2
float64 c2
string[] paramValue

"""
  __slots__ = ['count','x1','y1','z1','a1','b1','c1','x2','y2','z2','a2','b2','c2','paramValue']
  _slot_types = ['int32','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','string[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       count,x1,y1,z1,a1,b1,c1,x2,y2,z2,a2,b2,c2,paramValue

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(CircleRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.count is None:
        self.count = 0
      if self.x1 is None:
        self.x1 = 0.
      if self.y1 is None:
        self.y1 = 0.
      if self.z1 is None:
        self.z1 = 0.
      if self.a1 is None:
        self.a1 = 0.
      if self.b1 is None:
        self.b1 = 0.
      if self.c1 is None:
        self.c1 = 0.
      if self.x2 is None:
        self.x2 = 0.
      if self.y2 is None:
        self.y2 = 0.
      if self.z2 is None:
        self.z2 = 0.
      if self.a2 is None:
        self.a2 = 0.
      if self.b2 is None:
        self.b2 = 0.
      if self.c2 is None:
        self.c2 = 0.
      if self.paramValue is None:
        self.paramValue = []
    else:
      self.count = 0
      self.x1 = 0.
      self.y1 = 0.
      self.z1 = 0.
      self.a1 = 0.
      self.b1 = 0.
      self.c1 = 0.
      self.x2 = 0.
      self.y2 = 0.
      self.z2 = 0.
      self.a2 = 0.
      self.b2 = 0.
      self.c2 = 0.
      self.paramValue = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_i12d().pack(_x.count, _x.x1, _x.y1, _x.z1, _x.a1, _x.b1, _x.c1, _x.x2, _x.y2, _x.z2, _x.a2, _x.b2, _x.c2))
      length = len(self.paramValue)
      buff.write(_struct_I.pack(length))
      for val1 in self.paramValue:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.Struct('<I%ss'%length).pack(length, val1))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 100
      (_x.count, _x.x1, _x.y1, _x.z1, _x.a1, _x.b1, _x.c1, _x.x2, _x.y2, _x.z2, _x.a2, _x.b2, _x.c2,) = _get_struct_i12d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.paramValue = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1 = str[start:end]
        self.paramValue.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_i12d().pack(_x.count, _x.x1, _x.y1, _x.z1, _x.a1, _x.b1, _x.c1, _x.x2, _x.y2, _x.z2, _x.a2, _x.b2, _x.c2))
      length = len(self.paramValue)
      buff.write(_struct_I.pack(length))
      for val1 in self.paramValue:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.Struct('<I%ss'%length).pack(length, val1))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 100
      (_x.count, _x.x1, _x.y1, _x.z1, _x.a1, _x.b1, _x.c1, _x.x2, _x.y2, _x.z2, _x.a2, _x.b2, _x.c2,) = _get_struct_i12d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.paramValue = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1 = str[start:end]
        self.paramValue.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_i12d = None
def _get_struct_i12d():
    global _struct_i12d
    if _struct_i12d is None:
        _struct_i12d = struct.Struct("<i12d")
    return _struct_i12d
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from dobot_bringup/CircleResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class CircleResponse(genpy.Message):
  _md5sum = "ca16cfbd5443ad97f6cc7ffd6bb67292"
  _type = "dobot_bringup/CircleResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """
int32 res
"""
  __slots__ = ['res']
  _slot_types = ['int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       res

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(CircleResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.res is None:
        self.res = 0
    else:
      self.res = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self.res
      buff.write(_get_struct_i().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 4
      (self.res,) = _get_struct_i().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.res
      buff.write(_get_struct_i().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 4
      (self.res,) = _get_struct_i().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_i = None
def _get_struct_i():
    global _struct_i
    if _struct_i is None:
        _struct_i = struct.Struct("<i")
    return _struct_i
class Circle(object):
  _type          = 'dobot_bringup/Circle'
  _md5sum = '595948d4551fca3138762c937ac2d5fd'
  _request_class  = CircleRequest
  _response_class = CircleResponse
