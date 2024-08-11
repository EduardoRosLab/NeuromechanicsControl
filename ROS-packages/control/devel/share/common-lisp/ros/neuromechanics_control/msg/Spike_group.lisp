; Auto-generated. Do not edit!


(cl:in-package neuromechanics_control-msg)


;//! \htmlinclude Spike_group.msg.html

(cl:defclass <Spike_group> (roslisp-msg-protocol:ros-message)
  ((neuron_index
    :reader neuron_index
    :initarg :neuron_index
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (time
    :reader time
    :initarg :time
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Spike_group (<Spike_group>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Spike_group>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Spike_group)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name neuromechanics_control-msg:<Spike_group> is deprecated: use neuromechanics_control-msg:Spike_group instead.")))

(cl:ensure-generic-function 'neuron_index-val :lambda-list '(m))
(cl:defmethod neuron_index-val ((m <Spike_group>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neuromechanics_control-msg:neuron_index-val is deprecated.  Use neuromechanics_control-msg:neuron_index instead.")
  (neuron_index m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <Spike_group>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neuromechanics_control-msg:time-val is deprecated.  Use neuromechanics_control-msg:time instead.")
  (time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Spike_group>) ostream)
  "Serializes a message object of type '<Spike_group>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'neuron_index))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'neuron_index))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'time))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Spike_group>) istream)
  "Deserializes a message object of type '<Spike_group>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'neuron_index) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'neuron_index)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'time) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'time)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Spike_group>)))
  "Returns string type for a message object of type '<Spike_group>"
  "neuromechanics_control/Spike_group")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Spike_group)))
  "Returns string type for a message object of type 'Spike_group"
  "neuromechanics_control/Spike_group")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Spike_group>)))
  "Returns md5sum for a message object of type '<Spike_group>"
  "981939a1e507ff75c61993d24854ca72")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Spike_group)))
  "Returns md5sum for a message object of type 'Spike_group"
  "981939a1e507ff75c61993d24854ca72")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Spike_group>)))
  "Returns full string definition for message of type '<Spike_group>"
  (cl:format cl:nil "uint32[] neuron_index~%float64[] time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Spike_group)))
  "Returns full string definition for message of type 'Spike_group"
  (cl:format cl:nil "uint32[] neuron_index~%float64[] time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Spike_group>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'neuron_index) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'time) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Spike_group>))
  "Converts a ROS message object to a list"
  (cl:list 'Spike_group
    (cl:cons ':neuron_index (neuron_index msg))
    (cl:cons ':time (time msg))
))
