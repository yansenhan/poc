;; Auto-generated. Do not edit!


(when (boundp 'unitree_legged_msgs::HighState)
  (if (not (find-package "UNITREE_LEGGED_MSGS"))
    (make-package "UNITREE_LEGGED_MSGS"))
  (shadow 'HighState (find-package "UNITREE_LEGGED_MSGS")))
(unless (find-package "UNITREE_LEGGED_MSGS::HIGHSTATE")
  (make-package "UNITREE_LEGGED_MSGS::HIGHSTATE"))

(in-package "ROS")
;;//! \htmlinclude HighState.msg.html


(defclass unitree_legged_msgs::HighState
  :super ros::object
  :slots (_levelFlag _commVersion _robotID _SN _bandWidth _mode _imu _position _velocity _yawSpeed _footPosition2Body _footSpeed2Body _footForce _wirelessRemote _reserve _crc ))

(defmethod unitree_legged_msgs::HighState
  (:init
   (&key
    ((:levelFlag __levelFlag) 0)
    ((:commVersion __commVersion) 0)
    ((:robotID __robotID) 0)
    ((:SN __SN) 0)
    ((:bandWidth __bandWidth) 0)
    ((:mode __mode) 0)
    ((:imu __imu) (instance unitree_legged_msgs::IMU :init))
    ((:position __position) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:velocity __velocity) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:yawSpeed __yawSpeed) 0.0)
    ((:footPosition2Body __footPosition2Body) (let (r) (dotimes (i 4) (push (instance unitree_legged_msgs::Cartesian :init) r)) r))
    ((:footSpeed2Body __footSpeed2Body) (let (r) (dotimes (i 4) (push (instance unitree_legged_msgs::Cartesian :init) r)) r))
    ((:footForce __footForce) (make-array 4 :initial-element 0 :element-type :integer))
    ((:wirelessRemote __wirelessRemote) (make-array 40 :initial-element 0 :element-type :char))
    ((:reserve __reserve) 0)
    ((:crc __crc) 0)
    )
   (send-super :init)
   (setq _levelFlag (round __levelFlag))
   (setq _commVersion (round __commVersion))
   (setq _robotID (round __robotID))
   (setq _SN (round __SN))
   (setq _bandWidth (round __bandWidth))
   (setq _mode (round __mode))
   (setq _imu __imu)
   (setq _position __position)
   (setq _velocity __velocity)
   (setq _yawSpeed (float __yawSpeed))
   (setq _footPosition2Body __footPosition2Body)
   (setq _footSpeed2Body __footSpeed2Body)
   (setq _footForce __footForce)
   (setq _wirelessRemote __wirelessRemote)
   (setq _reserve (round __reserve))
   (setq _crc (round __crc))
   self)
  (:levelFlag
   (&optional __levelFlag)
   (if __levelFlag (setq _levelFlag __levelFlag)) _levelFlag)
  (:commVersion
   (&optional __commVersion)
   (if __commVersion (setq _commVersion __commVersion)) _commVersion)
  (:robotID
   (&optional __robotID)
   (if __robotID (setq _robotID __robotID)) _robotID)
  (:SN
   (&optional __SN)
   (if __SN (setq _SN __SN)) _SN)
  (:bandWidth
   (&optional __bandWidth)
   (if __bandWidth (setq _bandWidth __bandWidth)) _bandWidth)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:imu
   (&rest __imu)
   (if (keywordp (car __imu))
       (send* _imu __imu)
     (progn
       (if __imu (setq _imu (car __imu)))
       _imu)))
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:yawSpeed
   (&optional __yawSpeed)
   (if __yawSpeed (setq _yawSpeed __yawSpeed)) _yawSpeed)
  (:footPosition2Body
   (&rest __footPosition2Body)
   (if (keywordp (car __footPosition2Body))
       (send* _footPosition2Body __footPosition2Body)
     (progn
       (if __footPosition2Body (setq _footPosition2Body (car __footPosition2Body)))
       _footPosition2Body)))
  (:footSpeed2Body
   (&rest __footSpeed2Body)
   (if (keywordp (car __footSpeed2Body))
       (send* _footSpeed2Body __footSpeed2Body)
     (progn
       (if __footSpeed2Body (setq _footSpeed2Body (car __footSpeed2Body)))
       _footSpeed2Body)))
  (:footForce
   (&optional __footForce)
   (if __footForce (setq _footForce __footForce)) _footForce)
  (:wirelessRemote
   (&optional __wirelessRemote)
   (if __wirelessRemote (setq _wirelessRemote __wirelessRemote)) _wirelessRemote)
  (:reserve
   (&optional __reserve)
   (if __reserve (setq _reserve __reserve)) _reserve)
  (:crc
   (&optional __crc)
   (if __crc (setq _crc __crc)) _crc)
  (:serialization-length
   ()
   (+
    ;; uint8 _levelFlag
    1
    ;; uint16 _commVersion
    2
    ;; uint16 _robotID
    2
    ;; uint32 _SN
    4
    ;; uint8 _bandWidth
    1
    ;; uint8 _mode
    1
    ;; unitree_legged_msgs/IMU _imu
    (send _imu :serialization-length)
    ;; float32[3] _position
    (* 4    3)
    ;; float32[3] _velocity
    (* 4    3)
    ;; float32 _yawSpeed
    4
    ;; unitree_legged_msgs/Cartesian[4] _footPosition2Body
    (apply #'+ (send-all _footPosition2Body :serialization-length))
    ;; unitree_legged_msgs/Cartesian[4] _footSpeed2Body
    (apply #'+ (send-all _footSpeed2Body :serialization-length))
    ;; int16[4] _footForce
    (* 2    4)
    ;; uint8[40] _wirelessRemote
    (* 1    40)
    ;; uint32 _reserve
    4
    ;; uint32 _crc
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _levelFlag
       (write-byte _levelFlag s)
     ;; uint16 _commVersion
       (write-word _commVersion s)
     ;; uint16 _robotID
       (write-word _robotID s)
     ;; uint32 _SN
       (write-long _SN s)
     ;; uint8 _bandWidth
       (write-byte _bandWidth s)
     ;; uint8 _mode
       (write-byte _mode s)
     ;; unitree_legged_msgs/IMU _imu
       (send _imu :serialize s)
     ;; float32[3] _position
     (dotimes (i 3)
       (sys::poke (elt _position i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[3] _velocity
     (dotimes (i 3)
       (sys::poke (elt _velocity i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _yawSpeed
       (sys::poke _yawSpeed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; unitree_legged_msgs/Cartesian[4] _footPosition2Body
     (dolist (elem _footPosition2Body)
       (send elem :serialize s)
       )
     ;; unitree_legged_msgs/Cartesian[4] _footSpeed2Body
     (dolist (elem _footSpeed2Body)
       (send elem :serialize s)
       )
     ;; int16[4] _footForce
     (dotimes (i 4)
       (write-word (elt _footForce i) s)
       )
     ;; uint8[40] _wirelessRemote
     (princ _wirelessRemote s)
     ;; uint32 _reserve
       (write-long _reserve s)
     ;; uint32 _crc
       (write-long _crc s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _levelFlag
     (setq _levelFlag (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _commVersion
     (setq _commVersion (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _robotID
     (setq _robotID (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint32 _SN
     (setq _SN (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _bandWidth
     (setq _bandWidth (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; unitree_legged_msgs/IMU _imu
     (send _imu :deserialize buf ptr-) (incf ptr- (send _imu :serialization-length))
   ;; float32[3] _position
   (dotimes (i (length _position))
     (setf (elt _position i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[3] _velocity
   (dotimes (i (length _velocity))
     (setf (elt _velocity i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32 _yawSpeed
     (setq _yawSpeed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; unitree_legged_msgs/Cartesian[4] _footPosition2Body
   (dotimes (i 4)
     (send (elt _footPosition2Body i) :deserialize buf ptr-) (incf ptr- (send (elt _footPosition2Body i) :serialization-length))
     )
   ;; unitree_legged_msgs/Cartesian[4] _footSpeed2Body
   (dotimes (i 4)
     (send (elt _footSpeed2Body i) :deserialize buf ptr-) (incf ptr- (send (elt _footSpeed2Body i) :serialization-length))
     )
   ;; int16[4] _footForce
   (dotimes (i (length _footForce))
     (setf (elt _footForce i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; uint8[40] _wirelessRemote
   (setq _wirelessRemote (make-array 40 :element-type :char))
   (replace _wirelessRemote buf :start2 ptr-) (incf ptr- 40)
   ;; uint32 _reserve
     (setq _reserve (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _crc
     (setq _crc (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get unitree_legged_msgs::HighState :md5sum-) "5ac674d268805caf9dbab61ef025c30a")
(setf (get unitree_legged_msgs::HighState :datatype-) "unitree_legged_msgs/HighState")
(setf (get unitree_legged_msgs::HighState :definition-)
      "uint8 levelFlag
uint16 commVersion
uint16 robotID
uint32 SN
uint8 bandWidth
uint8 mode
IMU imu
float32[3] position 
float32[3] velocity 
float32 yawSpeed
Cartesian[4] footPosition2Body 
Cartesian[4] footSpeed2Body	
int16[4] footForce
uint8[40] wirelessRemote
uint32 reserve
uint32 crc






================================================================================
MSG: unitree_legged_msgs/IMU
float32[4] quaternion
float32[3] gyroscope
float32[3] accelerometer
float32[3] rpy
int8 temperature
================================================================================
MSG: unitree_legged_msgs/Cartesian
float32 x
float32 y
float32 z
")



(provide :unitree_legged_msgs/HighState "5ac674d268805caf9dbab61ef025c30a")


