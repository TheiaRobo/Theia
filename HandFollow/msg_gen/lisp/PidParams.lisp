; Auto-generated. Do not edit!


(cl:in-package HandFollow-msg)


;//! \htmlinclude PidParams.msg.html

(cl:defclass <PidParams> (roslisp-msg-protocol:ros-message)
  ((p_x
    :reader p_x
    :initarg :p_x
    :type cl:float
    :initform 0.0)
   (p_z
    :reader p_z
    :initarg :p_z
    :type cl:float
    :initform 0.0))
)

(cl:defclass PidParams (<PidParams>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PidParams>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PidParams)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name HandFollow-msg:<PidParams> is deprecated: use HandFollow-msg:PidParams instead.")))

(cl:ensure-generic-function 'p_x-val :lambda-list '(m))
(cl:defmethod p_x-val ((m <PidParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader HandFollow-msg:p_x-val is deprecated.  Use HandFollow-msg:p_x instead.")
  (p_x m))

(cl:ensure-generic-function 'p_z-val :lambda-list '(m))
(cl:defmethod p_z-val ((m <PidParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader HandFollow-msg:p_z-val is deprecated.  Use HandFollow-msg:p_z instead.")
  (p_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PidParams>) ostream)
  "Serializes a message object of type '<PidParams>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'p_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'p_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PidParams>) istream)
  "Deserializes a message object of type '<PidParams>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'p_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'p_z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PidParams>)))
  "Returns string type for a message object of type '<PidParams>"
  "HandFollow/PidParams")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PidParams)))
  "Returns string type for a message object of type 'PidParams"
  "HandFollow/PidParams")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PidParams>)))
  "Returns md5sum for a message object of type '<PidParams>"
  "5a5e0c267dd7fe0632fbcd19cf9e2030")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PidParams)))
  "Returns md5sum for a message object of type 'PidParams"
  "5a5e0c267dd7fe0632fbcd19cf9e2030")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PidParams>)))
  "Returns full string definition for message of type '<PidParams>"
  (cl:format cl:nil "float32 p_x~%float32 p_z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PidParams)))
  "Returns full string definition for message of type 'PidParams"
  (cl:format cl:nil "float32 p_x~%float32 p_z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PidParams>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PidParams>))
  "Converts a ROS message object to a list"
  (cl:list 'PidParams
    (cl:cons ':p_x (p_x msg))
    (cl:cons ':p_z (p_z msg))
))
