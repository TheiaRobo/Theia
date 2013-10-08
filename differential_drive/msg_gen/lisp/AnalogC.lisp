; Auto-generated. Do not edit!


(cl:in-package differential_drive-msg)


;//! \htmlinclude AnalogC.msg.html

(cl:defclass <AnalogC> (roslisp-msg-protocol:ros-message)
  ((ch1
    :reader ch1
    :initarg :ch1
    :type cl:fixnum
    :initform 0)
   (ch2
    :reader ch2
    :initarg :ch2
    :type cl:fixnum
    :initform 0)
   (ch3
    :reader ch3
    :initarg :ch3
    :type cl:fixnum
    :initform 0)
   (ch4
    :reader ch4
    :initarg :ch4
    :type cl:fixnum
    :initform 0)
   (ch5
    :reader ch5
    :initarg :ch5
    :type cl:fixnum
    :initform 0)
   (ch6
    :reader ch6
    :initarg :ch6
    :type cl:fixnum
    :initform 0)
   (ch7
    :reader ch7
    :initarg :ch7
    :type cl:fixnum
    :initform 0)
   (ch8
    :reader ch8
    :initarg :ch8
    :type cl:fixnum
    :initform 0)
   (on_batt
    :reader on_batt
    :initarg :on_batt
    :type cl:fixnum
    :initform 0)
   (cell1
    :reader cell1
    :initarg :cell1
    :type cl:float
    :initform 0.0)
   (cell2
    :reader cell2
    :initarg :cell2
    :type cl:float
    :initform 0.0)
   (cell3
    :reader cell3
    :initarg :cell3
    :type cl:float
    :initform 0.0))
)

(cl:defclass AnalogC (<AnalogC>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AnalogC>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AnalogC)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name differential_drive-msg:<AnalogC> is deprecated: use differential_drive-msg:AnalogC instead.")))

(cl:ensure-generic-function 'ch1-val :lambda-list '(m))
(cl:defmethod ch1-val ((m <AnalogC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:ch1-val is deprecated.  Use differential_drive-msg:ch1 instead.")
  (ch1 m))

(cl:ensure-generic-function 'ch2-val :lambda-list '(m))
(cl:defmethod ch2-val ((m <AnalogC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:ch2-val is deprecated.  Use differential_drive-msg:ch2 instead.")
  (ch2 m))

(cl:ensure-generic-function 'ch3-val :lambda-list '(m))
(cl:defmethod ch3-val ((m <AnalogC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:ch3-val is deprecated.  Use differential_drive-msg:ch3 instead.")
  (ch3 m))

(cl:ensure-generic-function 'ch4-val :lambda-list '(m))
(cl:defmethod ch4-val ((m <AnalogC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:ch4-val is deprecated.  Use differential_drive-msg:ch4 instead.")
  (ch4 m))

(cl:ensure-generic-function 'ch5-val :lambda-list '(m))
(cl:defmethod ch5-val ((m <AnalogC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:ch5-val is deprecated.  Use differential_drive-msg:ch5 instead.")
  (ch5 m))

(cl:ensure-generic-function 'ch6-val :lambda-list '(m))
(cl:defmethod ch6-val ((m <AnalogC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:ch6-val is deprecated.  Use differential_drive-msg:ch6 instead.")
  (ch6 m))

(cl:ensure-generic-function 'ch7-val :lambda-list '(m))
(cl:defmethod ch7-val ((m <AnalogC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:ch7-val is deprecated.  Use differential_drive-msg:ch7 instead.")
  (ch7 m))

(cl:ensure-generic-function 'ch8-val :lambda-list '(m))
(cl:defmethod ch8-val ((m <AnalogC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:ch8-val is deprecated.  Use differential_drive-msg:ch8 instead.")
  (ch8 m))

(cl:ensure-generic-function 'on_batt-val :lambda-list '(m))
(cl:defmethod on_batt-val ((m <AnalogC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:on_batt-val is deprecated.  Use differential_drive-msg:on_batt instead.")
  (on_batt m))

(cl:ensure-generic-function 'cell1-val :lambda-list '(m))
(cl:defmethod cell1-val ((m <AnalogC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:cell1-val is deprecated.  Use differential_drive-msg:cell1 instead.")
  (cell1 m))

(cl:ensure-generic-function 'cell2-val :lambda-list '(m))
(cl:defmethod cell2-val ((m <AnalogC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:cell2-val is deprecated.  Use differential_drive-msg:cell2 instead.")
  (cell2 m))

(cl:ensure-generic-function 'cell3-val :lambda-list '(m))
(cl:defmethod cell3-val ((m <AnalogC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:cell3-val is deprecated.  Use differential_drive-msg:cell3 instead.")
  (cell3 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AnalogC>) ostream)
  "Serializes a message object of type '<AnalogC>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ch1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ch1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ch2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ch2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ch3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ch3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ch4)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ch4)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ch5)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ch5)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ch6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ch6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ch7)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ch7)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ch8)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ch8)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'on_batt)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cell1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cell2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cell3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AnalogC>) istream)
  "Deserializes a message object of type '<AnalogC>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ch1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ch1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ch2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ch2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ch3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ch3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ch4)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ch4)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ch5)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ch5)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ch6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ch6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ch7)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ch7)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ch8)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ch8)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'on_batt)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cell1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cell2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cell3) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AnalogC>)))
  "Returns string type for a message object of type '<AnalogC>"
  "differential_drive/AnalogC")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnalogC)))
  "Returns string type for a message object of type 'AnalogC"
  "differential_drive/AnalogC")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AnalogC>)))
  "Returns md5sum for a message object of type '<AnalogC>"
  "2e0957cb939ed78036365a4d8012012f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AnalogC)))
  "Returns md5sum for a message object of type 'AnalogC"
  "2e0957cb939ed78036365a4d8012012f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AnalogC>)))
  "Returns full string definition for message of type '<AnalogC>"
  (cl:format cl:nil "# 8 * uint16 for IR sensors~%uint16 ch1~%uint16 ch2~%uint16 ch3~%uint16 ch4~%uint16 ch5~%uint16 ch6~%uint16 ch7~%uint16 ch8~%~%# Say if battery is in use, of if a wall adaptator is present~%# value : 1 if on battery, 0 if adaptator present~%uint8 on_batt~%~%# cell voltage~%float32 cell1~%float32 cell2~%float32 cell3~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AnalogC)))
  "Returns full string definition for message of type 'AnalogC"
  (cl:format cl:nil "# 8 * uint16 for IR sensors~%uint16 ch1~%uint16 ch2~%uint16 ch3~%uint16 ch4~%uint16 ch5~%uint16 ch6~%uint16 ch7~%uint16 ch8~%~%# Say if battery is in use, of if a wall adaptator is present~%# value : 1 if on battery, 0 if adaptator present~%uint8 on_batt~%~%# cell voltage~%float32 cell1~%float32 cell2~%float32 cell3~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AnalogC>))
  (cl:+ 0
     2
     2
     2
     2
     2
     2
     2
     2
     1
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AnalogC>))
  "Converts a ROS message object to a list"
  (cl:list 'AnalogC
    (cl:cons ':ch1 (ch1 msg))
    (cl:cons ':ch2 (ch2 msg))
    (cl:cons ':ch3 (ch3 msg))
    (cl:cons ':ch4 (ch4 msg))
    (cl:cons ':ch5 (ch5 msg))
    (cl:cons ':ch6 (ch6 msg))
    (cl:cons ':ch7 (ch7 msg))
    (cl:cons ':ch8 (ch8 msg))
    (cl:cons ':on_batt (on_batt msg))
    (cl:cons ':cell1 (cell1 msg))
    (cl:cons ':cell2 (cell2 msg))
    (cl:cons ':cell3 (cell3 msg))
))
