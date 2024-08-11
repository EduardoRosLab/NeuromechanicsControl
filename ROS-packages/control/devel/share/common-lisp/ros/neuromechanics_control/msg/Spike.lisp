; Auto-generated. Do not edit!


(cl:in-package neuromechanics_control-msg)


;//! \htmlinclude Spike.msg.html

(cl:defclass <Spike> (roslisp-msg-protocol:ros-message)
  ((neuron_index
    :reader neuron_index
    :initarg :neuron_index
    :type cl:integer
    :initform 0)
   (time
    :reader time
    :initarg :time
    :type cl:float
    :initform 0.0))
)

(cl:defclass Spike (<Spike>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Spike>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Spike)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name neuromechanics_control-msg:<Spike> is deprecated: use neuromechanics_control-msg:Spike instead.")))

(cl:ensure-generic-function 'neuron_index-val :lambda-list '(m))
(cl:defmethod neuron_index-val ((m <Spike>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neuromechanics_control-msg:neuron_index-val is deprecated.  Use neuromechanics_control-msg:neuron_index instead.")
  (neuron_index m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <Spike>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neuromechanics_control-msg:time-val is deprecated.  Use neuromechanics_control-msg:time instead.")
  (time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Spike>) ostream)
  "Serializes a message object of type '<Spike>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'neuron_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'neuron_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'neuron_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'neuron_index)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Spike>) istream)
  "Deserializes a message object of type '<Spike>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'neuron_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'neuron_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'neuron_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'neuron_index)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'time) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Spike>)))
  "Returns string type for a message object of type '<Spike>"
  "neuromechanics_control/Spike")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Spike)))
  "Returns string type for a message object of type 'Spike"
  "neuromechanics_control/Spike")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Spike>)))
  "Returns md5sum for a message object of type '<Spike>"
  "ffb331888bb7ffb461985148bcf50fd8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Spike)))
  "Returns md5sum for a message object of type 'Spike"
  "ffb331888bb7ffb461985148bcf50fd8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Spike>)))
  "Returns full string definition for message of type '<Spike>"
  (cl:format cl:nil "uint32 neuron_index~%float64 time~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Spike)))
  "Returns full string definition for message of type 'Spike"
  (cl:format cl:nil "uint32 neuron_index~%float64 time~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Spike>))
  (cl:+ 0
     4
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Spike>))
  "Converts a ROS message object to a list"
  (cl:list 'Spike
    (cl:cons ':neuron_index (neuron_index msg))
    (cl:cons ':time (time msg))
))
