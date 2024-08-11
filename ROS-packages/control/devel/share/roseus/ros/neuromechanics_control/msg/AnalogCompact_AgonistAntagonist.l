;; Auto-generated. Do not edit!


(when (boundp 'neuromechanics_control::AnalogCompact_AgonistAntagonist)
  (if (not (find-package "NEUROMECHANICS_CONTROL"))
    (make-package "NEUROMECHANICS_CONTROL"))
  (shadow 'AnalogCompact_AgonistAntagonist (find-package "NEUROMECHANICS_CONTROL")))
(unless (find-package "NEUROMECHANICS_CONTROL::ANALOGCOMPACT_AGONISTANTAGONIST")
  (make-package "NEUROMECHANICS_CONTROL::ANALOGCOMPACT_AGONISTANTAGONIST"))

(in-package "ROS")
;;//! \htmlinclude AnalogCompact_AgonistAntagonist.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass neuromechanics_control::AnalogCompact_AgonistAntagonist
  :super ros::object
  :slots (_header _agonist _antagonist _names ))

(defmethod neuromechanics_control::AnalogCompact_AgonistAntagonist
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:agonist __agonist) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:antagonist __antagonist) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:names __names) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _agonist __agonist)
   (setq _antagonist __antagonist)
   (setq _names __names)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:agonist
   (&optional __agonist)
   (if __agonist (setq _agonist __agonist)) _agonist)
  (:antagonist
   (&optional __antagonist)
   (if __antagonist (setq _antagonist __antagonist)) _antagonist)
  (:names
   (&optional __names)
   (if __names (setq _names __names)) _names)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[] _agonist
    (* 8    (length _agonist)) 4
    ;; float64[] _antagonist
    (* 8    (length _antagonist)) 4
    ;; string[] _names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _names)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64[] _agonist
     (write-long (length _agonist) s)
     (dotimes (i (length _agonist))
       (sys::poke (elt _agonist i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _antagonist
     (write-long (length _antagonist) s)
     (dotimes (i (length _antagonist))
       (sys::poke (elt _antagonist i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; string[] _names
     (write-long (length _names) s)
     (dolist (elem _names)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64[] _agonist
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _agonist (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _agonist i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _antagonist
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _antagonist (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _antagonist i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; string[] _names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(setf (get neuromechanics_control::AnalogCompact_AgonistAntagonist :md5sum-) "cbcf5b90b78f051386b46f54c96ce043")
(setf (get neuromechanics_control::AnalogCompact_AgonistAntagonist :datatype-) "neuromechanics_control/AnalogCompact_AgonistAntagonist")
(setf (get neuromechanics_control::AnalogCompact_AgonistAntagonist :definition-)
      "Header header
float64[] agonist
float64[] antagonist
string[]  names

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



(provide :neuromechanics_control/AnalogCompact_AgonistAntagonist "cbcf5b90b78f051386b46f54c96ce043")


