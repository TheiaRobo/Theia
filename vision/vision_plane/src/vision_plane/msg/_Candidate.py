"""autogenerated by genpy from vision_plane/Candidate.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Candidate(genpy.Message):
  _md5sum = "17d5a688673dd37068bec707e6d07ab3"
  _type = "vision_plane/Candidate"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """#Candidate message
float32 minLatitude
float32 maxLatitude
float32 minLongitude
float32 maxLongitude
"""
  __slots__ = ['minLatitude','maxLatitude','minLongitude','maxLongitude']
  _slot_types = ['float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       minLatitude,maxLatitude,minLongitude,maxLongitude

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Candidate, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.minLatitude is None:
        self.minLatitude = 0.
      if self.maxLatitude is None:
        self.maxLatitude = 0.
      if self.minLongitude is None:
        self.minLongitude = 0.
      if self.maxLongitude is None:
        self.maxLongitude = 0.
    else:
      self.minLatitude = 0.
      self.maxLatitude = 0.
      self.minLongitude = 0.
      self.maxLongitude = 0.

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
      buff.write(_struct_4f.pack(_x.minLatitude, _x.maxLatitude, _x.minLongitude, _x.maxLongitude))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 16
      (_x.minLatitude, _x.maxLatitude, _x.minLongitude, _x.maxLongitude,) = _struct_4f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_4f.pack(_x.minLatitude, _x.maxLatitude, _x.minLongitude, _x.maxLongitude))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 16
      (_x.minLatitude, _x.maxLatitude, _x.minLongitude, _x.maxLongitude,) = _struct_4f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_4f = struct.Struct("<4f")
