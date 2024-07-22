; Auto-generated. Do not edit!


(cl:in-package ucar_nav-srv)


;//! \htmlinclude Boxinfo-request.msg.html

(cl:defclass <Boxinfo-request> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (box_x
    :reader box_x
    :initarg :box_x
    :type cl:fixnum
    :initform 0)
   (box_y
    :reader box_y
    :initarg :box_y
    :type cl:fixnum
    :initform 0)
   (box_w
    :reader box_w
    :initarg :box_w
    :type cl:fixnum
    :initform 0)
   (box_h
    :reader box_h
    :initarg :box_h
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Boxinfo-request (<Boxinfo-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Boxinfo-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Boxinfo-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ucar_nav-srv:<Boxinfo-request> is deprecated: use ucar_nav-srv:Boxinfo-request instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Boxinfo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_nav-srv:header-val is deprecated.  Use ucar_nav-srv:header instead.")
  (header m))

(cl:ensure-generic-function 'box_x-val :lambda-list '(m))
(cl:defmethod box_x-val ((m <Boxinfo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_nav-srv:box_x-val is deprecated.  Use ucar_nav-srv:box_x instead.")
  (box_x m))

(cl:ensure-generic-function 'box_y-val :lambda-list '(m))
(cl:defmethod box_y-val ((m <Boxinfo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_nav-srv:box_y-val is deprecated.  Use ucar_nav-srv:box_y instead.")
  (box_y m))

(cl:ensure-generic-function 'box_w-val :lambda-list '(m))
(cl:defmethod box_w-val ((m <Boxinfo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_nav-srv:box_w-val is deprecated.  Use ucar_nav-srv:box_w instead.")
  (box_w m))

(cl:ensure-generic-function 'box_h-val :lambda-list '(m))
(cl:defmethod box_h-val ((m <Boxinfo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_nav-srv:box_h-val is deprecated.  Use ucar_nav-srv:box_h instead.")
  (box_h m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Boxinfo-request>) ostream)
  "Serializes a message object of type '<Boxinfo-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'box_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'box_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'box_y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'box_y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'box_w)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'box_w)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'box_h)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'box_h)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Boxinfo-request>) istream)
  "Deserializes a message object of type '<Boxinfo-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'box_x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'box_x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'box_y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'box_y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'box_w)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'box_w)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'box_h)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'box_h)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Boxinfo-request>)))
  "Returns string type for a service object of type '<Boxinfo-request>"
  "ucar_nav/BoxinfoRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Boxinfo-request)))
  "Returns string type for a service object of type 'Boxinfo-request"
  "ucar_nav/BoxinfoRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Boxinfo-request>)))
  "Returns md5sum for a message object of type '<Boxinfo-request>"
  "dc4ccb29fbc5c9e5a5b7a5542979e09e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Boxinfo-request)))
  "Returns md5sum for a message object of type 'Boxinfo-request"
  "dc4ccb29fbc5c9e5a5b7a5542979e09e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Boxinfo-request>)))
  "Returns full string definition for message of type '<Boxinfo-request>"
  (cl:format cl:nil "Header header~%uint16 box_x~%uint16 box_y~%uint16 box_w~%uint16 box_h~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Boxinfo-request)))
  "Returns full string definition for message of type 'Boxinfo-request"
  (cl:format cl:nil "Header header~%uint16 box_x~%uint16 box_y~%uint16 box_w~%uint16 box_h~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Boxinfo-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Boxinfo-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Boxinfo-request
    (cl:cons ':header (header msg))
    (cl:cons ':box_x (box_x msg))
    (cl:cons ':box_y (box_y msg))
    (cl:cons ':box_w (box_w msg))
    (cl:cons ':box_h (box_h msg))
))
;//! \htmlinclude Boxinfo-response.msg.html

(cl:defclass <Boxinfo-response> (roslisp-msg-protocol:ros-message)
  ((pla_x
    :reader pla_x
    :initarg :pla_x
    :type cl:float
    :initform 0.0)
   (pla_y
    :reader pla_y
    :initarg :pla_y
    :type cl:float
    :initform 0.0)
   (road
    :reader road
    :initarg :road
    :type cl:float
    :initform 0.0))
)

(cl:defclass Boxinfo-response (<Boxinfo-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Boxinfo-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Boxinfo-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ucar_nav-srv:<Boxinfo-response> is deprecated: use ucar_nav-srv:Boxinfo-response instead.")))

(cl:ensure-generic-function 'pla_x-val :lambda-list '(m))
(cl:defmethod pla_x-val ((m <Boxinfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_nav-srv:pla_x-val is deprecated.  Use ucar_nav-srv:pla_x instead.")
  (pla_x m))

(cl:ensure-generic-function 'pla_y-val :lambda-list '(m))
(cl:defmethod pla_y-val ((m <Boxinfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_nav-srv:pla_y-val is deprecated.  Use ucar_nav-srv:pla_y instead.")
  (pla_y m))

(cl:ensure-generic-function 'road-val :lambda-list '(m))
(cl:defmethod road-val ((m <Boxinfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_nav-srv:road-val is deprecated.  Use ucar_nav-srv:road instead.")
  (road m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Boxinfo-response>) ostream)
  "Serializes a message object of type '<Boxinfo-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pla_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pla_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'road))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Boxinfo-response>) istream)
  "Deserializes a message object of type '<Boxinfo-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pla_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pla_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'road) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Boxinfo-response>)))
  "Returns string type for a service object of type '<Boxinfo-response>"
  "ucar_nav/BoxinfoResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Boxinfo-response)))
  "Returns string type for a service object of type 'Boxinfo-response"
  "ucar_nav/BoxinfoResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Boxinfo-response>)))
  "Returns md5sum for a message object of type '<Boxinfo-response>"
  "dc4ccb29fbc5c9e5a5b7a5542979e09e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Boxinfo-response)))
  "Returns md5sum for a message object of type 'Boxinfo-response"
  "dc4ccb29fbc5c9e5a5b7a5542979e09e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Boxinfo-response>)))
  "Returns full string definition for message of type '<Boxinfo-response>"
  (cl:format cl:nil "float32 pla_x~%float32 pla_y~%float32 road~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Boxinfo-response)))
  "Returns full string definition for message of type 'Boxinfo-response"
  (cl:format cl:nil "float32 pla_x~%float32 pla_y~%float32 road~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Boxinfo-response>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Boxinfo-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Boxinfo-response
    (cl:cons ':pla_x (pla_x msg))
    (cl:cons ':pla_y (pla_y msg))
    (cl:cons ':road (road msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Boxinfo)))
  'Boxinfo-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Boxinfo)))
  'Boxinfo-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Boxinfo)))
  "Returns string type for a service object of type '<Boxinfo>"
  "ucar_nav/Boxinfo")