;; Auto-generated. Do not edit!


(when (boundp 'neuromechanics_control::Spike)
  (if (not (find-package "NEUROMECHANICS_CONTROL"))
    (make-package "NEUROMECHANICS_CONTROL"))
  (shadow 'Spike (find-package "NEUROMECHANICS_CONTROL")))
(unless (find-package "NEUROMECHANICS_CONTROL::SPIKE")
  (make-package "NEUROMECHANICS_CONTROL::SPIKE"))

(in-package "ROS")
;;//! \htmlinclude Spike.msg.html


(defclass neuromechanics_control::Spike
  :super ros::object
  :slots (_neuron_index _time ))

(defmethod neuromechanics_control::Spike
  (:init
   (&key
    ((:neuron_index __neuron_index) 0)
    ((:time __time) 0.0)
    )
   (send-super :init)
   (setq _neuron_index (round __neuron_index))
   (setq _time (float __time))
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
    ;; uint32 _neuron_index
    4
    ;; float64 _time
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _neuron_index
       (write-long _neuron_index s)
     ;; float64 _time
       (sys::poke _time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _neuron_index
     (setq _neuron_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _time
     (setq _time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get neuromechanics_control::Spike :md5sum-) "ffb331888bb7ffb461985148bcf50fd8")
(setf (get neuromechanics_control::Spike :datatype-) "neuromechanics_control/Spike")
(setf (get neuromechanics_control::Spike :definition-)
      "uint32 neuron_index
float64 time
")



(provide :neuromechanics_control/Spike "ffb331888bb7ffb461985148bcf50fd8")


