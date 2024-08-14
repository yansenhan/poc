;; Auto-generated. Do not edit!


(when (boundp 'ucar_nav::Boxinfo)
  (if (not (find-package "UCAR_NAV"))
    (make-package "UCAR_NAV"))
  (shadow 'Boxinfo (find-package "UCAR_NAV")))
(unless (find-package "UCAR_NAV::BOXINFO")
  (make-package "UCAR_NAV::BOXINFO"))
(unless (find-package "UCAR_NAV::BOXINFOREQUEST")
  (make-package "UCAR_NAV::BOXINFOREQUEST"))
(unless (find-package "UCAR_NAV::BOXINFORESPONSE")
  (make-package "UCAR_NAV::BOXINFORESPONSE"))

(in-package "ROS")

(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))




(defclass ucar_nav::BoxinfoRequest
  :super ros::object
  :slots (_header _box_x _box_y _box_w _box_h ))

(defmethod ucar_nav::BoxinfoRequest
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:box_x __box_x) 0)
    ((:box_y __box_y) 0)
    ((:box_w __box_w) 0)
    ((:box_h __box_h) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _box_x (round __box_x))
   (setq _box_y (round __box_y))
   (setq _box_w (round __box_w))
   (setq _box_h (round __box_h))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:box_x
   (&optional __box_x)
   (if __box_x (setq _box_x __box_x)) _box_x)
  (:box_y
   (&optional __box_y)
   (if __box_y (setq _box_y __box_y)) _box_y)
  (:box_w
   (&optional __box_w)
   (if __box_w (setq _box_w __box_w)) _box_w)
  (:box_h
   (&optional __box_h)
   (if __box_h (setq _box_h __box_h)) _box_h)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint16 _box_x
    2
    ;; uint16 _box_y
    2
    ;; uint16 _box_w
    2
    ;; uint16 _box_h
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint16 _box_x
       (write-word _box_x s)
     ;; uint16 _box_y
       (write-word _box_y s)
     ;; uint16 _box_w
       (write-word _box_w s)
     ;; uint16 _box_h
       (write-word _box_h s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint16 _box_x
     (setq _box_x (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _box_y
     (setq _box_y (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _box_w
     (setq _box_w (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _box_h
     (setq _box_h (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass ucar_nav::BoxinfoResponse
  :super ros::object
  :slots (_pla_x _pla_y _road ))

(defmethod ucar_nav::BoxinfoResponse
  (:init
   (&key
    ((:pla_x __pla_x) 0.0)
    ((:pla_y __pla_y) 0.0)
    ((:road __road) 0.0)
    )
   (send-super :init)
   (setq _pla_x (float __pla_x))
   (setq _pla_y (float __pla_y))
   (setq _road (float __road))
   self)
  (:pla_x
   (&optional __pla_x)
   (if __pla_x (setq _pla_x __pla_x)) _pla_x)
  (:pla_y
   (&optional __pla_y)
   (if __pla_y (setq _pla_y __pla_y)) _pla_y)
  (:road
   (&optional __road)
   (if __road (setq _road __road)) _road)
  (:serialization-length
   ()
   (+
    ;; float32 _pla_x
    4
    ;; float32 _pla_y
    4
    ;; float32 _road
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _pla_x
       (sys::poke _pla_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pla_y
       (sys::poke _pla_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _road
       (sys::poke _road (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _pla_x
     (setq _pla_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pla_y
     (setq _pla_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _road
     (setq _road (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass ucar_nav::Boxinfo
  :super ros::object
  :slots ())

(setf (get ucar_nav::Boxinfo :md5sum-) "dc4ccb29fbc5c9e5a5b7a5542979e09e")
(setf (get ucar_nav::Boxinfo :datatype-) "ucar_nav/Boxinfo")
(setf (get ucar_nav::Boxinfo :request) ucar_nav::BoxinfoRequest)
(setf (get ucar_nav::Boxinfo :response) ucar_nav::BoxinfoResponse)

(defmethod ucar_nav::BoxinfoRequest
  (:response () (instance ucar_nav::BoxinfoResponse :init)))

(setf (get ucar_nav::BoxinfoRequest :md5sum-) "dc4ccb29fbc5c9e5a5b7a5542979e09e")
(setf (get ucar_nav::BoxinfoRequest :datatype-) "ucar_nav/BoxinfoRequest")
(setf (get ucar_nav::BoxinfoRequest :definition-)
      "Header header
uint16 box_x
uint16 box_y
uint16 box_w
uint16 box_h

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
---
float32 pla_x
float32 pla_y
float32 road

")

(setf (get ucar_nav::BoxinfoResponse :md5sum-) "dc4ccb29fbc5c9e5a5b7a5542979e09e")
(setf (get ucar_nav::BoxinfoResponse :datatype-) "ucar_nav/BoxinfoResponse")
(setf (get ucar_nav::BoxinfoResponse :definition-)
      "Header header
uint16 box_x
uint16 box_y
uint16 box_w
uint16 box_h

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
---
float32 pla_x
float32 pla_y
float32 road

")



(provide :ucar_nav/Boxinfo "dc4ccb29fbc5c9e5a5b7a5542979e09e")


