; Auto-generated. Do not edit!


(cl:in-package ucar_nav-msg)


;//! \htmlinclude Boxinfo.msg.html

(cl:defclass <Boxinfo> (roslisp-msg-protocol:ros-message)
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

(cl:defclass Boxinfo (<Boxinfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Boxinfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Boxinfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ucar_nav-msg:<Boxinfo> is deprecated: use ucar_nav-msg:Boxinfo instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Boxinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_nav-msg:header-val is deprecated.  Use ucar_nav-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'box_x-val :lambda-list '(m))
(cl:defmethod box_x-val ((m <Boxinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_nav-msg:box_x-val is deprecated.  Use ucar_nav-msg:box_x instead.")
  (box_x m))

(cl:ensure-generic-function 'box_y-val :lambda-list '(m))
(cl:defmethod box_y-val ((m <Boxinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_nav-msg:box_y-val is deprecated.  Use ucar_nav-msg:box_y instead.")
  (box_y m))

(cl:ensure-generic-function 'box_w-val :lambda-list '(m))
(cl:defmethod box_w-val ((m <Boxinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_nav-msg:box_w-val is deprecated.  Use ucar_nav-msg:box_w instead.")
  (box_w m))

(cl:ensure-generic-function 'box_h-val :lambda-list '(m))
(cl:defmethod box_h-val ((m <Boxinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ucar_nav-msg:box_h-val is deprecated.  Use ucar_nav-msg:box_h instead.")
  (box_h m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Boxinfo>) ostream)
  "Serializes a message object of type '<Boxinfo>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Boxinfo>) istream)
  "Deserializes a message object of type '<Boxinfo>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Boxinfo>)))
  "Returns string type for a message object of type '<Boxinfo>"
  "ucar_nav/Boxinfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Boxinfo)))
  "Returns string type for a message object of type 'Boxinfo"
  "ucar_nav/Boxinfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Boxinfo>)))
  "Returns md5sum for a message object of type '<Boxinfo>"
  "beb47c590a7a389074e521a1e16ec95d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Boxinfo)))
  "Returns md5sum for a message object of type 'Boxinfo"
  "beb47c590a7a389074e521a1e16ec95d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Boxinfo>)))
  "Returns full string definition for message of type '<Boxinfo>"
  (cl:format cl:nil "Header header~%uint16 box_x~%uint16 box_y~%uint16 box_w~%uint16 box_h~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Boxinfo)))
  "Returns full string definition for message of type 'Boxinfo"
  (cl:format cl:nil "Header header~%uint16 box_x~%uint16 box_y~%uint16 box_w~%uint16 box_h~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Boxinfo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Boxinfo>))
  "Converts a ROS message object to a list"
  (cl:list 'Boxinfo
    (cl:cons ':header (header msg))
    (cl:cons ':box_x (box_x msg))
    (cl:cons ':box_y (box_y msg))
    (cl:cons ':box_w (box_w msg))
    (cl:cons ':box_h (box_h msg))
))
