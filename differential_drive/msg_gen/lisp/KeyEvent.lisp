; Auto-generated. Do not edit!


(cl:in-package differential_drive-msg)


;//! \htmlinclude KeyEvent.msg.html

(cl:defclass <KeyEvent> (roslisp-msg-protocol:ros-message)
  ((timestamp
    :reader timestamp
    :initarg :timestamp
    :type cl:float
    :initform 0.0)
   (sym
    :reader sym
    :initarg :sym
    :type cl:fixnum
    :initform 0)
   (pressed
    :reader pressed
    :initarg :pressed
    :type cl:fixnum
    :initform 0)
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass KeyEvent (<KeyEvent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <KeyEvent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'KeyEvent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name differential_drive-msg:<KeyEvent> is deprecated: use differential_drive-msg:KeyEvent instead.")))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <KeyEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:timestamp-val is deprecated.  Use differential_drive-msg:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'sym-val :lambda-list '(m))
(cl:defmethod sym-val ((m <KeyEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:sym-val is deprecated.  Use differential_drive-msg:sym instead.")
  (sym m))

(cl:ensure-generic-function 'pressed-val :lambda-list '(m))
(cl:defmethod pressed-val ((m <KeyEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:pressed-val is deprecated.  Use differential_drive-msg:pressed instead.")
  (pressed m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <KeyEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:name-val is deprecated.  Use differential_drive-msg:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <KeyEvent>) ostream)
  "Serializes a message object of type '<KeyEvent>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'timestamp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sym)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sym)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pressed)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <KeyEvent>) istream)
  "Deserializes a message object of type '<KeyEvent>"
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
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sym)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sym)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pressed)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<KeyEvent>)))
  "Returns string type for a message object of type '<KeyEvent>"
  "differential_drive/KeyEvent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'KeyEvent)))
  "Returns string type for a message object of type 'KeyEvent"
  "differential_drive/KeyEvent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<KeyEvent>)))
  "Returns md5sum for a message object of type '<KeyEvent>"
  "644813ba9e39b249cc8e56e2fa499967")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'KeyEvent)))
  "Returns md5sum for a message object of type 'KeyEvent"
  "644813ba9e39b249cc8e56e2fa499967")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<KeyEvent>)))
  "Returns full string definition for message of type '<KeyEvent>"
  (cl:format cl:nil "float64 timestamp~%uint16 sym~%uint8 pressed~%string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'KeyEvent)))
  "Returns full string definition for message of type 'KeyEvent"
  (cl:format cl:nil "float64 timestamp~%uint16 sym~%uint8 pressed~%string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <KeyEvent>))
  (cl:+ 0
     8
     2
     1
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <KeyEvent>))
  "Converts a ROS message object to a list"
  (cl:list 'KeyEvent
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':sym (sym msg))
    (cl:cons ':pressed (pressed msg))
    (cl:cons ':name (name msg))
))
