;; Auto-generated. Do not edit!


(when (boundp 'ucar_nav::Boxinfo)
  (if (not (find-package "UCAR_NAV"))
    (make-package "UCAR_NAV"))
  (shadow 'Boxinfo (find-package "UCAR_NAV")))
(unless (find-package "UCAR_NAV::BOXINFO")
  (make-package "UCAR_NAV::BOXINFO"))

(in-package "ROS")
;;//! \htmlinclude Boxinfo.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ucar_nav::Boxinfo
  :super ros::object
  :slots (_header _box_x _box_y _box_w _box_h ))

(defmethod ucar_nav::Boxinfo
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

(setf (get ucar_nav::Boxinfo :md5sum-) "beb47c590a7a389074e521a1e16ec95d")
(setf (get ucar_nav::Boxinfo :datatype-) "ucar_nav/Boxinfo")
(setf (get ucar_nav::Boxinfo :definition-)
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

")



(provide :ucar_nav/Boxinfo "beb47c590a7a389074e521a1e16ec95d")


