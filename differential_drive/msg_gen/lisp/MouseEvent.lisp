; Auto-generated. Do not edit!


(cl:in-package differential_drive-msg)


;//! \htmlinclude MouseEvent.msg.html

(cl:defclass <MouseEvent> (roslisp-msg-protocol:ros-message)
  ((timestamp
    :reader timestamp
    :initarg :timestamp
    :type cl:float
    :initform 0.0)
   (dx
    :reader dx
    :initarg :dx
    :type cl:fixnum
    :initform 0)
   (dy
    :reader dy
    :initarg :dy
    :type cl:fixnum
    :initform 0)
   (button
    :reader button
    :initarg :button
    :type cl:fixnum
    :initform 0)
   (pressed
    :reader pressed
    :initarg :pressed
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MouseEvent (<MouseEvent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MouseEvent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MouseEvent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name differential_drive-msg:<MouseEvent> is deprecated: use differential_drive-msg:MouseEvent instead.")))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <MouseEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:timestamp-val is deprecated.  Use differential_drive-msg:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'dx-val :lambda-list '(m))
(cl:defmethod dx-val ((m <MouseEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:dx-val is deprecated.  Use differential_drive-msg:dx instead.")
  (dx m))

(cl:ensure-generic-function 'dy-val :lambda-list '(m))
(cl:defmethod dy-val ((m <MouseEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:dy-val is deprecated.  Use differential_drive-msg:dy instead.")
  (dy m))

(cl:ensure-generic-function 'button-val :lambda-list '(m))
(cl:defmethod button-val ((m <MouseEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:button-val is deprecated.  Use differential_drive-msg:button instead.")
  (button m))

(cl:ensure-generic-function 'pressed-val :lambda-list '(m))
(cl:defmethod pressed-val ((m <MouseEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:pressed-val is deprecated.  Use differential_drive-msg:pressed instead.")
  (pressed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MouseEvent>) ostream)
  "Serializes a message object of type '<MouseEvent>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'timestamp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'dx)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'dy)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'button)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pressed)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MouseEvent>) istream)
  "Deserializes a message object of type '<MouseEvent>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'timestamp) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dx) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dy) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'button)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pressed)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MouseEvent>)))
  "Returns string type for a message object of type '<MouseEvent>"
  "differential_drive/MouseEvent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MouseEvent)))
  "Returns string type for a message object of type 'MouseEvent"
  "differential_drive/MouseEvent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MouseEvent>)))
  "Returns md5sum for a message object of type '<MouseEvent>"
  "ef8c9f3ac43b76bcf800e7d5001209fb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MouseEvent)))
  "Returns md5sum for a message object of type 'MouseEvent"
  "ef8c9f3ac43b76bcf800e7d5001209fb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MouseEvent>)))
  "Returns full string definition for message of type '<MouseEvent>"
  (cl:format cl:nil "float64 timestamp~%int16 dx~%int16 dy~%uint8 button~%uint8 pressed~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MouseEvent)))
  "Returns full string definition for message of type 'MouseEvent"
  (cl:format cl:nil "float64 timestamp~%int16 dx~%int16 dy~%uint8 button~%uint8 pressed~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MouseEvent>))
  (cl:+ 0
     8
     2
     2
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MouseEvent>))
  "Converts a ROS message object to a list"
  (cl:list 'MouseEvent
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':dx (dx msg))
    (cl:cons ':dy (dy msg))
    (cl:cons ':button (button msg))
    (cl:cons ':pressed (pressed msg))
))
