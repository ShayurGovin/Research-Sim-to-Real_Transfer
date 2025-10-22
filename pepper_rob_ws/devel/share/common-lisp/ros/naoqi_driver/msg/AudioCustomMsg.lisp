; Auto-generated. Do not edit!


(cl:in-package naoqi_driver-msg)


;//! \htmlinclude AudioCustomMsg.msg.html

(cl:defclass <AudioCustomMsg> (roslisp-msg-protocol:ros-message)
  ((rearLeft
    :reader rearLeft
    :initarg :rearLeft
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (rearRight
    :reader rearRight
    :initarg :rearRight
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (frontLeft
    :reader frontLeft
    :initarg :frontLeft
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (frontRight
    :reader frontRight
    :initarg :frontRight
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass AudioCustomMsg (<AudioCustomMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AudioCustomMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AudioCustomMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name naoqi_driver-msg:<AudioCustomMsg> is deprecated: use naoqi_driver-msg:AudioCustomMsg instead.")))

(cl:ensure-generic-function 'rearLeft-val :lambda-list '(m))
(cl:defmethod rearLeft-val ((m <AudioCustomMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader naoqi_driver-msg:rearLeft-val is deprecated.  Use naoqi_driver-msg:rearLeft instead.")
  (rearLeft m))

(cl:ensure-generic-function 'rearRight-val :lambda-list '(m))
(cl:defmethod rearRight-val ((m <AudioCustomMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader naoqi_driver-msg:rearRight-val is deprecated.  Use naoqi_driver-msg:rearRight instead.")
  (rearRight m))

(cl:ensure-generic-function 'frontLeft-val :lambda-list '(m))
(cl:defmethod frontLeft-val ((m <AudioCustomMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader naoqi_driver-msg:frontLeft-val is deprecated.  Use naoqi_driver-msg:frontLeft instead.")
  (frontLeft m))

(cl:ensure-generic-function 'frontRight-val :lambda-list '(m))
(cl:defmethod frontRight-val ((m <AudioCustomMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader naoqi_driver-msg:frontRight-val is deprecated.  Use naoqi_driver-msg:frontRight instead.")
  (frontRight m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AudioCustomMsg>) ostream)
  "Serializes a message object of type '<AudioCustomMsg>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'rearLeft))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'rearLeft))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'rearRight))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'rearRight))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'frontLeft))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'frontLeft))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'frontRight))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'frontRight))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AudioCustomMsg>) istream)
  "Deserializes a message object of type '<AudioCustomMsg>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'rearLeft) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'rearLeft)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'rearRight) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'rearRight)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'frontLeft) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'frontLeft)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'frontRight) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'frontRight)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AudioCustomMsg>)))
  "Returns string type for a message object of type '<AudioCustomMsg>"
  "naoqi_driver/AudioCustomMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AudioCustomMsg)))
  "Returns string type for a message object of type 'AudioCustomMsg"
  "naoqi_driver/AudioCustomMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AudioCustomMsg>)))
  "Returns md5sum for a message object of type '<AudioCustomMsg>"
  "bcac904b0030b6d70386338d161f4882")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AudioCustomMsg)))
  "Returns md5sum for a message object of type 'AudioCustomMsg"
  "bcac904b0030b6d70386338d161f4882")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AudioCustomMsg>)))
  "Returns full string definition for message of type '<AudioCustomMsg>"
  (cl:format cl:nil "# AudioCustomMsg.msg~%int16[] rearLeft~%int16[] rearRight~%int16[] frontLeft~%int16[] frontRight~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AudioCustomMsg)))
  "Returns full string definition for message of type 'AudioCustomMsg"
  (cl:format cl:nil "# AudioCustomMsg.msg~%int16[] rearLeft~%int16[] rearRight~%int16[] frontLeft~%int16[] frontRight~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AudioCustomMsg>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'rearLeft) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'rearRight) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'frontLeft) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'frontRight) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AudioCustomMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'AudioCustomMsg
    (cl:cons ':rearLeft (rearLeft msg))
    (cl:cons ':rearRight (rearRight msg))
    (cl:cons ':frontLeft (frontLeft msg))
    (cl:cons ':frontRight (frontRight msg))
))
