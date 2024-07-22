; Auto-generated. Do not edit!


(cl:in-package unitree_legged_msgs-msg)


;//! \htmlinclude LowState.msg.html

(cl:defclass <LowState> (roslisp-msg-protocol:ros-message)
  ((levelFlag
    :reader levelFlag
    :initarg :levelFlag
    :type cl:fixnum
    :initform 0)
   (commVersion
    :reader commVersion
    :initarg :commVersion
    :type cl:fixnum
    :initform 0)
   (robotID
    :reader robotID
    :initarg :robotID
    :type cl:fixnum
    :initform 0)
   (SN
    :reader SN
    :initarg :SN
    :type cl:integer
    :initform 0)
   (bandWidth
    :reader bandWidth
    :initarg :bandWidth
    :type cl:fixnum
    :initform 0)
   (imu
    :reader imu
    :initarg :imu
    :type unitree_legged_msgs-msg:IMU
    :initform (cl:make-instance 'unitree_legged_msgs-msg:IMU))
   (motorState
    :reader motorState
    :initarg :motorState
    :type (cl:vector unitree_legged_msgs-msg:MotorState)
   :initform (cl:make-array 20 :element-type 'unitree_legged_msgs-msg:MotorState :initial-element (cl:make-instance 'unitree_legged_msgs-msg:MotorState)))
   (footForce
    :reader footForce
    :initarg :footForce
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 4 :element-type 'cl:fixnum :initial-element 0))
   (footForceEst
    :reader footForceEst
    :initarg :footForceEst
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 4 :element-type 'cl:fixnum :initial-element 0))
   (tick
    :reader tick
    :initarg :tick
    :type cl:integer
    :initform 0)
   (wirelessRemote
    :reader wirelessRemote
    :initarg :wirelessRemote
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 40 :element-type 'cl:fixnum :initial-element 0))
   (reserve
    :reader reserve
    :initarg :reserve
    :type cl:integer
    :initform 0)
   (crc
    :reader crc
    :initarg :crc
    :type cl:integer
    :initform 0))
)

(cl:defclass LowState (<LowState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LowState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LowState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name unitree_legged_msgs-msg:<LowState> is deprecated: use unitree_legged_msgs-msg:LowState instead.")))

(cl:ensure-generic-function 'levelFlag-val :lambda-list '(m))
(cl:defmethod levelFlag-val ((m <LowState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:levelFlag-val is deprecated.  Use unitree_legged_msgs-msg:levelFlag instead.")
  (levelFlag m))

(cl:ensure-generic-function 'commVersion-val :lambda-list '(m))
(cl:defmethod commVersion-val ((m <LowState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:commVersion-val is deprecated.  Use unitree_legged_msgs-msg:commVersion instead.")
  (commVersion m))

(cl:ensure-generic-function 'robotID-val :lambda-list '(m))
(cl:defmethod robotID-val ((m <LowState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:robotID-val is deprecated.  Use unitree_legged_msgs-msg:robotID instead.")
  (robotID m))

(cl:ensure-generic-function 'SN-val :lambda-list '(m))
(cl:defmethod SN-val ((m <LowState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:SN-val is deprecated.  Use unitree_legged_msgs-msg:SN instead.")
  (SN m))

(cl:ensure-generic-function 'bandWidth-val :lambda-list '(m))
(cl:defmethod bandWidth-val ((m <LowState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:bandWidth-val is deprecated.  Use unitree_legged_msgs-msg:bandWidth instead.")
  (bandWidth m))

(cl:ensure-generic-function 'imu-val :lambda-list '(m))
(cl:defmethod imu-val ((m <LowState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:imu-val is deprecated.  Use unitree_legged_msgs-msg:imu instead.")
  (imu m))

(cl:ensure-generic-function 'motorState-val :lambda-list '(m))
(cl:defmethod motorState-val ((m <LowState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:motorState-val is deprecated.  Use unitree_legged_msgs-msg:motorState instead.")
  (motorState m))

(cl:ensure-generic-function 'footForce-val :lambda-list '(m))
(cl:defmethod footForce-val ((m <LowState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:footForce-val is deprecated.  Use unitree_legged_msgs-msg:footForce instead.")
  (footForce m))

(cl:ensure-generic-function 'footForceEst-val :lambda-list '(m))
(cl:defmethod footForceEst-val ((m <LowState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:footForceEst-val is deprecated.  Use unitree_legged_msgs-msg:footForceEst instead.")
  (footForceEst m))

(cl:ensure-generic-function 'tick-val :lambda-list '(m))
(cl:defmethod tick-val ((m <LowState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:tick-val is deprecated.  Use unitree_legged_msgs-msg:tick instead.")
  (tick m))

(cl:ensure-generic-function 'wirelessRemote-val :lambda-list '(m))
(cl:defmethod wirelessRemote-val ((m <LowState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:wirelessRemote-val is deprecated.  Use unitree_legged_msgs-msg:wirelessRemote instead.")
  (wirelessRemote m))

(cl:ensure-generic-function 'reserve-val :lambda-list '(m))
(cl:defmethod reserve-val ((m <LowState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:reserve-val is deprecated.  Use unitree_legged_msgs-msg:reserve instead.")
  (reserve m))

(cl:ensure-generic-function 'crc-val :lambda-list '(m))
(cl:defmethod crc-val ((m <LowState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:crc-val is deprecated.  Use unitree_legged_msgs-msg:crc instead.")
  (crc m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LowState>) ostream)
  "Serializes a message object of type '<LowState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'levelFlag)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'commVersion)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'commVersion)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'robotID)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'robotID)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'SN)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'SN)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'SN)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'SN)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bandWidth)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'imu) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'motorState))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'footForce))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'footForceEst))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tick)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tick)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'tick)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'tick)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'wirelessRemote))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reserve)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'reserve)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'reserve)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'reserve)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'crc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'crc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'crc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'crc)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LowState>) istream)
  "Deserializes a message object of type '<LowState>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'levelFlag)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'commVersion)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'commVersion)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'robotID)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'robotID)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'SN)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'SN)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'SN)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'SN)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bandWidth)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'imu) istream)
  (cl:setf (cl:slot-value msg 'motorState) (cl:make-array 20))
  (cl:let ((vals (cl:slot-value msg 'motorState)))
    (cl:dotimes (i 20)
    (cl:setf (cl:aref vals i) (cl:make-instance 'unitree_legged_msgs-msg:MotorState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  (cl:setf (cl:slot-value msg 'footForce) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'footForce)))
    (cl:dotimes (i 4)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  (cl:setf (cl:slot-value msg 'footForceEst) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'footForceEst)))
    (cl:dotimes (i 4)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tick)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tick)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'tick)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'tick)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'wirelessRemote) (cl:make-array 40))
  (cl:let ((vals (cl:slot-value msg 'wirelessRemote)))
    (cl:dotimes (i 40)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reserve)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'reserve)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'reserve)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'reserve)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'crc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'crc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'crc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'crc)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LowState>)))
  "Returns string type for a message object of type '<LowState>"
  "unitree_legged_msgs/LowState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LowState)))
  "Returns string type for a message object of type 'LowState"
  "unitree_legged_msgs/LowState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LowState>)))
  "Returns md5sum for a message object of type '<LowState>"
  "7e531980fef61bce444b1f441321eee6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LowState)))
  "Returns md5sum for a message object of type 'LowState"
  "7e531980fef61bce444b1f441321eee6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LowState>)))
  "Returns full string definition for message of type '<LowState>"
  (cl:format cl:nil "uint8 levelFlag~%uint16 commVersion~%uint16 robotID~%uint32 SN~%uint8 bandWidth~%IMU imu~%MotorState[20] motorState~%int16[4] footForce	~%int16[4] footForceEst~%uint32 tick	~%uint8[40] wirelessRemote ~%uint32 reserve~%uint32 crc~%~%~%	~%					~%~%~%~%~%~%~%================================================================================~%MSG: unitree_legged_msgs/IMU~%float32[4] quaternion~%float32[3] gyroscope~%float32[3] accelerometer~%float32[3] rpy~%int8 temperature~%================================================================================~%MSG: unitree_legged_msgs/MotorState~%uint8 mode           # motor current mode ~%float32 q            # motor current position（rad）~%float32 dq           # motor current speed（rad/s）~%float32 ddq          # motor current speed（rad/s）~%float32 tauEst       # current estimated output torque（N*m）~%float32 q_raw        # motor current position（rad）~%float32 dq_raw       # motor current speed（rad/s）~%float32 ddq_raw      # motor current speed（rad/s）~%int8 temperature     # motor temperature（slow conduction of temperature leads to lag）~%uint32[2] reserve~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LowState)))
  "Returns full string definition for message of type 'LowState"
  (cl:format cl:nil "uint8 levelFlag~%uint16 commVersion~%uint16 robotID~%uint32 SN~%uint8 bandWidth~%IMU imu~%MotorState[20] motorState~%int16[4] footForce	~%int16[4] footForceEst~%uint32 tick	~%uint8[40] wirelessRemote ~%uint32 reserve~%uint32 crc~%~%~%	~%					~%~%~%~%~%~%~%================================================================================~%MSG: unitree_legged_msgs/IMU~%float32[4] quaternion~%float32[3] gyroscope~%float32[3] accelerometer~%float32[3] rpy~%int8 temperature~%================================================================================~%MSG: unitree_legged_msgs/MotorState~%uint8 mode           # motor current mode ~%float32 q            # motor current position（rad）~%float32 dq           # motor current speed（rad/s）~%float32 ddq          # motor current speed（rad/s）~%float32 tauEst       # current estimated output torque（N*m）~%float32 q_raw        # motor current position（rad）~%float32 dq_raw       # motor current speed（rad/s）~%float32 ddq_raw      # motor current speed（rad/s）~%int8 temperature     # motor temperature（slow conduction of temperature leads to lag）~%uint32[2] reserve~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LowState>))
  (cl:+ 0
     1
     2
     2
     4
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'imu))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'motorState) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'footForce) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'footForceEst) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'wirelessRemote) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LowState>))
  "Converts a ROS message object to a list"
  (cl:list 'LowState
    (cl:cons ':levelFlag (levelFlag msg))
    (cl:cons ':commVersion (commVersion msg))
    (cl:cons ':robotID (robotID msg))
    (cl:cons ':SN (SN msg))
    (cl:cons ':bandWidth (bandWidth msg))
    (cl:cons ':imu (imu msg))
    (cl:cons ':motorState (motorState msg))
    (cl:cons ':footForce (footForce msg))
    (cl:cons ':footForceEst (footForceEst msg))
    (cl:cons ':tick (tick msg))
    (cl:cons ':wirelessRemote (wirelessRemote msg))
    (cl:cons ':reserve (reserve msg))
    (cl:cons ':crc (crc msg))
))
