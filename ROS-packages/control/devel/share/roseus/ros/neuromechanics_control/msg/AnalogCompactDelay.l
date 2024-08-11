;; Auto-generated. Do not edit!


(when (boundp 'neuromechanics_control::AnalogCompactDelay)
  (if (not (find-package "NEUROMECHANICS_CONTROL"))
    (make-package "NEUROMECHANICS_CONTROL"))
  (shadow 'AnalogCompactDelay (find-package "NEUROMECHANICS_CONTROL")))
(unless (find-package "NEUROMECHANICS_CONTROL::ANALOGCOMPACTDELAY")
  (make-package "NEUROMECHANICS_CONTROL::ANALOGCOMPACTDELAY"))

(in-package "ROS")
;;//! \htmlinclude AnalogCompactDelay.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass neuromechanics_control::AnalogCompactDelay
  :super ros::object
  :slots (_header _data _names _delay ))

(defmethod neuromechanics_control::AnalogCompactDelay
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:data __data) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:names __names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:delay __delay) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _data __data)
   (setq _names __names)
   (setq _delay (float __delay))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:names
   (&optional __names)
   (if __names (setq _names __names)) _names)
  (:delay
   (&optional __delay)
   (if __delay (setq _delay __delay)) _delay)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[] _data
    (* 8    (length _data)) 4
    ;; string[] _names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _names)) 4
    ;; float64 _delay
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (sys::poke (elt _data i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; string[] _names
     (write-long (length _names) s)
     (dolist (elem _names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; float64 _delay
       (sys::poke _delay (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; string[] _names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; float64 _delay
     (setq _delay (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get neuromechanics_control::AnalogCompactDelay :md5sum-) "92d2748960dc0b01dc07b6d597269341")
(setf (get neuromechanics_control::AnalogCompactDelay :datatype-) "neuromechanics_control/AnalogCompactDelay")
(setf (get neuromechanics_control::AnalogCompactDelay :definition-)
      "Header header
float64[] data
string[]  names
float64 delay

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
# 0: no frame
# 1: global frame
string frame_id

")



(provide :neuromechanics_control/AnalogCompactDelay "92d2748960dc0b01dc07b6d597269341")


