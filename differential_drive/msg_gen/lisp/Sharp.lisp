; Auto-generated. Do not edit!


(cl:in-package differential_drive-msg)


;//! \htmlinclude Sharp.msg.html

(cl:defclass <Sharp> (roslisp-msg-protocol:ros-message)
  ((u0
    :reader u0
    :initarg :u0
    :type cl:fixnum
    :initform 0)
   (u1
    :reader u1
    :initarg :u1
    :type cl:fixnum
    :initform 0)
   (u2
    :reader u2
    :initarg :u2
    :type cl:fixnum
    :initform 0)
   (u3
    :reader u3
    :initarg :u3
    :type cl:fixnum
    :initform 0)
   (u4
    :reader u4
    :initarg :u4
    :type cl:fixnum
    :initform 0)
   (u5
    :reader u5
    :initarg :u5
    :type cl:fixnum
    :initform 0)
   (u6
    :reader u6
    :initarg :u6
    :type cl:fixnum
    :initform 0)
   (u7
    :reader u7
    :initarg :u7
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Sharp (<Sharp>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Sharp>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Sharp)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name differential_drive-msg:<Sharp> is deprecated: use differential_drive-msg:Sharp instead.")))

(cl:ensure-generic-function 'u0-val :lambda-list '(m))
(cl:defmethod u0-val ((m <Sharp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:u0-val is deprecated.  Use differential_drive-msg:u0 instead.")
  (u0 m))

(cl:ensure-generic-function 'u1-val :lambda-list '(m))
(cl:defmethod u1-val ((m <Sharp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:u1-val is deprecated.  Use differential_drive-msg:u1 instead.")
  (u1 m))

(cl:ensure-generic-function 'u2-val :lambda-list '(m))
(cl:defmethod u2-val ((m <Sharp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:u2-val is deprecated.  Use differential_drive-msg:u2 instead.")
  (u2 m))

(cl:ensure-generic-function 'u3-val :lambda-list '(m))
(cl:defmethod u3-val ((m <Sharp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:u3-val is deprecated.  Use differential_drive-msg:u3 instead.")
  (u3 m))

(cl:ensure-generic-function 'u4-val :lambda-list '(m))
(cl:defmethod u4-val ((m <Sharp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:u4-val is deprecated.  Use differential_drive-msg:u4 instead.")
  (u4 m))

(cl:ensure-generic-function 'u5-val :lambda-list '(m))
(cl:defmethod u5-val ((m <Sharp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:u5-val is deprecated.  Use differential_drive-msg:u5 instead.")
  (u5 m))

(cl:ensure-generic-function 'u6-val :lambda-list '(m))
(cl:defmethod u6-val ((m <Sharp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:u6-val is deprecated.  Use differential_drive-msg:u6 instead.")
  (u6 m))

(cl:ensure-generic-function 'u7-val :lambda-list '(m))
(cl:defmethod u7-val ((m <Sharp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:u7-val is deprecated.  Use differential_drive-msg:u7 instead.")
  (u7 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Sharp>) ostream)
  "Serializes a message object of type '<Sharp>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'u0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'u0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'u1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'u1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'u2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'u2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'u3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'u3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'u4)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'u4)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'u5)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'u5)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'u6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'u6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'u7)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'u7)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Sharp>) istream)
  "Deserializes a message object of type '<Sharp>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'u0)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'u0)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'u1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'u1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'u2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'u2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'u3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'u3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'u4)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'u4)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'u5)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'u5)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'u6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'u6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'u7)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'u7)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Sharp>)))
  "Returns string type for a message object of type '<Sharp>"
  "differential_drive/Sharp")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Sharp)))
  "Returns string type for a message object of type 'Sharp"
  "differential_drive/Sharp")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Sharp>)))
  "Returns md5sum for a message object of type '<Sharp>"
  "9e17fb25fb2f8b8cf0b60887d5e8cc66")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Sharp)))
  "Returns md5sum for a message object of type 'Sharp"
  "9e17fb25fb2f8b8cf0b60887d5e8cc66")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Sharp>)))
  "Returns full string definition for message of type '<Sharp>"
  (cl:format cl:nil "uint16 u0~%uint16 u1~%uint16 u2~%uint16 u3~%uint16 u4~%uint16 u5~%uint16 u6~%uint16 u7~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Sharp)))
  "Returns full string definition for message of type 'Sharp"
  (cl:format cl:nil "uint16 u0~%uint16 u1~%uint16 u2~%uint16 u3~%uint16 u4~%uint16 u5~%uint16 u6~%uint16 u7~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Sharp>))
  (cl:+ 0
     2
     2
     2
     2
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Sharp>))
  "Converts a ROS message object to a list"
  (cl:list 'Sharp
    (cl:cons ':u0 (u0 msg))
    (cl:cons ':u1 (u1 msg))
    (cl:cons ':u2 (u2 msg))
    (cl:cons ':u3 (u3 msg))
    (cl:cons ':u4 (u4 msg))
    (cl:cons ':u5 (u5 msg))
    (cl:cons ':u6 (u6 msg))
    (cl:cons ':u7 (u7 msg))
))
