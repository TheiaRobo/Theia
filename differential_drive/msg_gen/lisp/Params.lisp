; Auto-generated. Do not edit!


(cl:in-package differential_drive-msg)


;//! \htmlinclude Params.msg.html

(cl:defclass <Params> (roslisp-msg-protocol:ros-message)
  ((K
    :reader K
    :initarg :K
    :type cl:fixnum
    :initform 0)
   (KI
    :reader KI
    :initarg :KI
    :type cl:fixnum
    :initform 0)
   (INT_MAX
    :reader INT_MAX
    :initarg :INT_MAX
    :type cl:fixnum
    :initform 0)
   (ticks
    :reader ticks
    :initarg :ticks
    :type cl:fixnum
    :initform 0)
   (r
    :reader r
    :initarg :r
    :type cl:float
    :initform 0.0)
   (r_l
    :reader r_l
    :initarg :r_l
    :type cl:float
    :initform 0.0)
   (r_r
    :reader r_r
    :initarg :r_r
    :type cl:float
    :initform 0.0)
   (B
    :reader B
    :initarg :B
    :type cl:float
    :initform 0.0))
)

(cl:defclass Params (<Params>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Params>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Params)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name differential_drive-msg:<Params> is deprecated: use differential_drive-msg:Params instead.")))

(cl:ensure-generic-function 'K-val :lambda-list '(m))
(cl:defmethod K-val ((m <Params>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:K-val is deprecated.  Use differential_drive-msg:K instead.")
  (K m))

(cl:ensure-generic-function 'KI-val :lambda-list '(m))
(cl:defmethod KI-val ((m <Params>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:KI-val is deprecated.  Use differential_drive-msg:KI instead.")
  (KI m))

(cl:ensure-generic-function 'INT_MAX-val :lambda-list '(m))
(cl:defmethod INT_MAX-val ((m <Params>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:INT_MAX-val is deprecated.  Use differential_drive-msg:INT_MAX instead.")
  (INT_MAX m))

(cl:ensure-generic-function 'ticks-val :lambda-list '(m))
(cl:defmethod ticks-val ((m <Params>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:ticks-val is deprecated.  Use differential_drive-msg:ticks instead.")
  (ticks m))

(cl:ensure-generic-function 'r-val :lambda-list '(m))
(cl:defmethod r-val ((m <Params>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:r-val is deprecated.  Use differential_drive-msg:r instead.")
  (r m))

(cl:ensure-generic-function 'r_l-val :lambda-list '(m))
(cl:defmethod r_l-val ((m <Params>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:r_l-val is deprecated.  Use differential_drive-msg:r_l instead.")
  (r_l m))

(cl:ensure-generic-function 'r_r-val :lambda-list '(m))
(cl:defmethod r_r-val ((m <Params>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:r_r-val is deprecated.  Use differential_drive-msg:r_r instead.")
  (r_r m))

(cl:ensure-generic-function 'B-val :lambda-list '(m))
(cl:defmethod B-val ((m <Params>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:B-val is deprecated.  Use differential_drive-msg:B instead.")
  (B m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Params>) ostream)
  "Serializes a message object of type '<Params>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'K)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'K)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'KI)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'KI)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'INT_MAX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'INT_MAX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ticks)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ticks)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'r))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'r_l))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'r_r))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'B))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Params>) istream)
  "Deserializes a message object of type '<Params>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'K)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'K)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'KI)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'KI)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'INT_MAX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'INT_MAX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ticks)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ticks)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'r) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'r_l) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'r_r) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'B) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Params>)))
  "Returns string type for a message object of type '<Params>"
  "differential_drive/Params")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Params)))
  "Returns string type for a message object of type 'Params"
  "differential_drive/Params")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Params>)))
  "Returns md5sum for a message object of type '<Params>"
  "8950f824993d2867219a00186d4620f7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Params)))
  "Returns md5sum for a message object of type 'Params"
  "8950f824993d2867219a00186d4620f7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Params>)))
  "Returns full string definition for message of type '<Params>"
  (cl:format cl:nil "# PI Control parameters~%uint16 K~%uint16 KI~%uint16 INT_MAX~%uint16 ticks~%~%# Robot dimensions~%float32 r~%float32 r_l~%float32 r_r~%float32 B~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Params)))
  "Returns full string definition for message of type 'Params"
  (cl:format cl:nil "# PI Control parameters~%uint16 K~%uint16 KI~%uint16 INT_MAX~%uint16 ticks~%~%# Robot dimensions~%float32 r~%float32 r_l~%float32 r_r~%float32 B~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Params>))
  (cl:+ 0
     2
     2
     2
     2
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Params>))
  "Converts a ROS message object to a list"
  (cl:list 'Params
    (cl:cons ':K (K msg))
    (cl:cons ':KI (KI msg))
    (cl:cons ':INT_MAX (INT_MAX msg))
    (cl:cons ':ticks (ticks msg))
    (cl:cons ':r (r msg))
    (cl:cons ':r_l (r_l msg))
    (cl:cons ':r_r (r_r msg))
    (cl:cons ':B (B msg))
))
