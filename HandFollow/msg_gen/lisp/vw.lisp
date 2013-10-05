; Auto-generated. Do not edit!


(cl:in-package HandFollow-msg)


;//! \htmlinclude vw.msg.html

(cl:defclass <vw> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (v
    :reader v
    :initarg :v
    :type cl:float
    :initform 0.0)
   (w
    :reader w
    :initarg :w
    :type cl:float
    :initform 0.0))
)

(cl:defclass vw (<vw>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <vw>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'vw)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name HandFollow-msg:<vw> is deprecated: use HandFollow-msg:vw instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <vw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader HandFollow-msg:header-val is deprecated.  Use HandFollow-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'v-val :lambda-list '(m))
(cl:defmethod v-val ((m <vw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader HandFollow-msg:v-val is deprecated.  Use HandFollow-msg:v instead.")
  (v m))

(cl:ensure-generic-function 'w-val :lambda-list '(m))
(cl:defmethod w-val ((m <vw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader HandFollow-msg:w-val is deprecated.  Use HandFollow-msg:w instead.")
  (w m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <vw>) ostream)
  "Serializes a message object of type '<vw>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'v))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'w))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <vw>) istream)
  "Deserializes a message object of type '<vw>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'v) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'w) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<vw>)))
  "Returns string type for a message object of type '<vw>"
  "HandFollow/vw")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'vw)))
  "Returns string type for a message object of type 'vw"
  "HandFollow/vw")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<vw>)))
  "Returns md5sum for a message object of type '<vw>"
  "c9739f01512ce85d8ac1ccdd6bde650b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'vw)))
  "Returns md5sum for a message object of type 'vw"
  "c9739f01512ce85d8ac1ccdd6bde650b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<vw>)))
  "Returns full string definition for message of type '<vw>"
  (cl:format cl:nil "Header header~%float32 v~%float32 w~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'vw)))
  "Returns full string definition for message of type 'vw"
  (cl:format cl:nil "Header header~%float32 v~%float32 w~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <vw>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <vw>))
  "Converts a ROS message object to a list"
  (cl:list 'vw
    (cl:cons ':header (header msg))
    (cl:cons ':v (v msg))
    (cl:cons ':w (w msg))
))
