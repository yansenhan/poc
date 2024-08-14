;; Auto-generated. Do not edit!


(when (boundp 'unitree_legged_msgs::HighCmd)
  (if (not (find-package "UNITREE_LEGGED_MSGS"))
    (make-package "UNITREE_LEGGED_MSGS"))
  (shadow 'HighCmd (find-package "UNITREE_LEGGED_MSGS")))
(unless (find-package "UNITREE_LEGGED_MSGS::HIGHCMD")
  (make-package "UNITREE_LEGGED_MSGS::HIGHCMD"))

(in-package "ROS")
;;//! \htmlinclude HighCmd.msg.html


(defclass unitree_legged_msgs::HighCmd
  :super ros::object
  :slots (_levelFlag _commVersion _robotID _SN _bandWidth _mode _gaitType _speedLevel _dFootRaiseHeight _dBodyHeight _position _rpy _velocity _yawSpeed _led _wirelessRemote _reserve _crc ))

(defmethod unitree_legged_msgs::HighCmd
  (:init
   (&key
    ((:levelFlag __levelFlag) 0)
    ((:commVersion __commVersion) 0)
    ((:robotID __robotID) 0)
    ((:SN __SN) 0)
    ((:bandWidth __bandWidth) 0)
    ((:mode __mode) 0)
    ((:gaitType __gaitType) 0)
    ((:speedLevel __speedLevel) 0)
    ((:dFootRaiseHeight __dFootRaiseHeight) 0.0)
    ((:dBodyHeight __dBodyHeight) 0.0)
    ((:position __position) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:rpy __rpy) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:velocity __velocity) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:yawSpeed __yawSpeed) 0.0)
    ((:led __led) (let (r) (dotimes (i 4) (push (instance unitree_legged_msgs::LED :init) r)) r))
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
   (setq _gaitType (round __gaitType))
   (setq _speedLevel (round __speedLevel))
   (setq _dFootRaiseHeight (float __dFootRaiseHeight))
   (setq _dBodyHeight (float __dBodyHeight))
   (setq _position __position)
   (setq _rpy __rpy)
   (setq _velocity __velocity)
   (setq _yawSpeed (float __yawSpeed))
   (setq _led __led)
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
  (:gaitType
   (&optional __gaitType)
   (if __gaitType (setq _gaitType __gaitType)) _gaitType)
  (:speedLevel
   (&optional __speedLevel)
   (if __speedLevel (setq _speedLevel __speedLevel)) _speedLevel)
  (:dFootRaiseHeight
   (&optional __dFootRaiseHeight)
   (if __dFootRaiseHeight (setq _dFootRaiseHeight __dFootRaiseHeight)) _dFootRaiseHeight)
  (:dBodyHeight
   (&optional __dBodyHeight)
   (if __dBodyHeight (setq _dBodyHeight __dBodyHeight)) _dBodyHeight)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:rpy
   (&optional __rpy)
   (if __rpy (setq _rpy __rpy)) _rpy)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:yawSpeed
   (&optional __yawSpeed)
   (if __yawSpeed (setq _yawSpeed __yawSpeed)) _yawSpeed)
  (:led
   (&rest __led)
   (if (keywordp (car __led))
       (send* _led __led)
     (progn
       (if __led (setq _led (car __led)))
       _led)))
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
    ;; uint8 _gaitType
    1
    ;; uint8 _speedLevel
    1
    ;; float32 _dFootRaiseHeight
    4
    ;; float32 _dBodyHeight
    4
    ;; float32[2] _position
    (* 4    2)
    ;; float32[3] _rpy
    (* 4    3)
    ;; float32[2] _velocity
    (* 4    2)
    ;; float32 _yawSpeed
    4
    ;; unitree_legged_msgs/LED[4] _led
    (apply #'+ (send-all _led :serialization-length))
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
     ;; uint8 _gaitType
       (write-byte _gaitType s)
     ;; uint8 _speedLevel
       (write-byte _speedLevel s)
     ;; float32 _dFootRaiseHeight
       (sys::poke _dFootRaiseHeight (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _dBodyHeight
       (sys::poke _dBodyHeight (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32[2] _position
     (dotimes (i 2)
       (sys::poke (elt _position i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[3] _rpy
     (dotimes (i 3)
       (sys::poke (elt _rpy i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[2] _velocity
     (dotimes (i 2)
       (sys::poke (elt _velocity i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _yawSpeed
       (sys::poke _yawSpeed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; unitree_legged_msgs/LED[4] _led
     (dolist (elem _led)
       (send elem :serialize s)
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
   ;; uint8 _gaitType
     (setq _gaitType (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _speedLevel
     (setq _speedLevel (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _dFootRaiseHeight
     (setq _dFootRaiseHeight (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _dBodyHeight
     (setq _dBodyHeight (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32[2] _position
   (dotimes (i (length _position))
     (setf (elt _position i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[3] _rpy
   (dotimes (i (length _rpy))
     (setf (elt _rpy i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[2] _velocity
   (dotimes (i (length _velocity))
     (setf (elt _velocity i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32 _yawSpeed
     (setq _yawSpeed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; unitree_legged_msgs/LED[4] _led
   (dotimes (i 4)
     (send (elt _led i) :deserialize buf ptr-) (incf ptr- (send (elt _led i) :serialization-length))
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

(setf (get unitree_legged_msgs::HighCmd :md5sum-) "59333cac6112fd0a7a3d3c5b564af567")
(setf (get unitree_legged_msgs::HighCmd :datatype-) "unitree_legged_msgs/HighCmd")
(setf (get unitree_legged_msgs::HighCmd :definition-)
      "uint8 levelFlag
uint16 commVersion
uint16 robotID
uint32 SN
uint8 bandWidth
uint8 mode 
uint8 gaitType		   
uint8 speedLevel	
float32 dFootRaiseHeight		   
float32 dBodyHeight	 
float32[2] position 
float32[3] rpy	   
float32[2] velocity 
float32 yawSpeed		   
LED[4] led
uint8[40] wirelessRemote
uint32 reserve
uint32 crc
================================================================================
MSG: unitree_legged_msgs/LED
uint8 r
uint8 g
uint8 b
")



(provide :unitree_legged_msgs/HighCmd "59333cac6112fd0a7a3d3c5b564af567")


