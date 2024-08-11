;; Auto-generated. Do not edit!


(when (boundp 'neuromechanics_control::Spike_group)
  (if (not (find-package "NEUROMECHANICS_CONTROL"))
    (make-package "NEUROMECHANICS_CONTROL"))
  (shadow 'Spike_group (find-package "NEUROMECHANICS_CONTROL")))
(unless (find-package "NEUROMECHANICS_CONTROL::SPIKE_GROUP")
  (make-package "NEUROMECHANICS_CONTROL::SPIKE_GROUP"))

(in-package "ROS")
;;//! \htmlinclude Spike_group.msg.html


(defclass neuromechanics_control::Spike_group
  :super ros::object
  :slots (_neuron_index _time ))

(defmethod neuromechanics_control::Spike_group
  (:init
   (&key
    ((:neuron_index __neuron_index) (make-array 0 :initial-element 0 :element-type :integer))
    ((:time __time) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _neuron_index __neuron_index)
   (setq _time __time)
   self)
  (:neuron_index
   (&optional __neuron_index)
   (if __neuron_index (setq _neuron_index __neuron_index)) _neuron_index)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:serialization-length
   ()
   (+
    ;; uint32[] _neuron_index
    (* 4    (length _neuron_index)) 4
    ;; float64[] _time
    (* 8    (length _time)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32[] _neuron_index
     (write-long (length _neuron_index) s)
     (dotimes (i (length _neuron_index))
       (write-long (elt _neuron_index i) s)
       )
     ;; float64[] _time
     (write-long (length _time) s)
     (dotimes (i (length _time))
       (sys::poke (elt _time i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32[] _neuron_index
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _neuron_index (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _neuron_index i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; float64[] _time
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _time (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _time i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get neuromechanics_control::Spike_group :md5sum-) "981939a1e507ff75c61993d24854ca72")
(setf (get neuromechanics_control::Spike_group :datatype-) "neuromechanics_control/Spike_group")
(setf (get neuromechanics_control::Spike_group :definition-)
      "uint32[] neuron_index
float64[] time

")



(provide :neuromechanics_control/Spike_group "981939a1e507ff75c61993d24854ca72")


