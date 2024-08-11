; Auto-generated. Do not edit!


(cl:in-package neuromechanics_control-msg)


;//! \htmlinclude LearningState.msg.html

(cl:defclass <LearningState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (learning
    :reader learning
    :initarg :learning
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass LearningState (<LearningState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LearningState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LearningState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name neuromechanics_control-msg:<LearningState> is deprecated: use neuromechanics_control-msg:LearningState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <LearningState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neuromechanics_control-msg:header-val is deprecated.  Use neuromechanics_control-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'learning-val :lambda-list '(m))
(cl:defmethod learning-val ((m <LearningState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neuromechanics_control-msg:learning-val is deprecated.  Use neuromechanics_control-msg:learning instead.")
  (learning m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LearningState>) ostream)
  "Serializes a message object of type '<LearningState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'learning) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LearningState>) istream)
  "Deserializes a message object of type '<LearningState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'learning) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LearningState>)))
  "Returns string type for a message object of type '<LearningState>"
  "neuromechanics_control/LearningState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LearningState)))
  "Returns string type for a message object of type 'LearningState"
  "neuromechanics_control/LearningState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LearningState>)))
  "Returns md5sum for a message object of type '<LearningState>"
  "ef2f83334187d4def2eb42802f8414ef")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LearningState)))
  "Returns md5sum for a message object of type 'LearningState"
  "ef2f83334187d4def2eb42802f8414ef")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LearningState>)))
  "Returns full string definition for message of type '<LearningState>"
  (cl:format cl:nil "Header header~%bool learning~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LearningState)))
  "Returns full string definition for message of type 'LearningState"
  (cl:format cl:nil "Header header~%bool learning~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LearningState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LearningState>))
  "Converts a ROS message object to a list"
  (cl:list 'LearningState
    (cl:cons ':header (header msg))
    (cl:cons ':learning (learning msg))
))
