;; Auto-generated. Do not edit!


(when (boundp 'naoqi_driver::AudioCustomMsg)
  (if (not (find-package "NAOQI_DRIVER"))
    (make-package "NAOQI_DRIVER"))
  (shadow 'AudioCustomMsg (find-package "NAOQI_DRIVER")))
(unless (find-package "NAOQI_DRIVER::AUDIOCUSTOMMSG")
  (make-package "NAOQI_DRIVER::AUDIOCUSTOMMSG"))

(in-package "ROS")
;;//! \htmlinclude AudioCustomMsg.msg.html


(defclass naoqi_driver::AudioCustomMsg
  :super ros::object
  :slots (_rearLeft _rearRight _frontLeft _frontRight ))

(defmethod naoqi_driver::AudioCustomMsg
  (:init
   (&key
    ((:rearLeft __rearLeft) (make-array 0 :initial-element 0 :element-type :integer))
    ((:rearRight __rearRight) (make-array 0 :initial-element 0 :element-type :integer))
    ((:frontLeft __frontLeft) (make-array 0 :initial-element 0 :element-type :integer))
    ((:frontRight __frontRight) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _rearLeft __rearLeft)
   (setq _rearRight __rearRight)
   (setq _frontLeft __frontLeft)
   (setq _frontRight __frontRight)
   self)
  (:rearLeft
   (&optional __rearLeft)
   (if __rearLeft (setq _rearLeft __rearLeft)) _rearLeft)
  (:rearRight
   (&optional __rearRight)
   (if __rearRight (setq _rearRight __rearRight)) _rearRight)
  (:frontLeft
   (&optional __frontLeft)
   (if __frontLeft (setq _frontLeft __frontLeft)) _frontLeft)
  (:frontRight
   (&optional __frontRight)
   (if __frontRight (setq _frontRight __frontRight)) _frontRight)
  (:serialization-length
   ()
   (+
    ;; int16[] _rearLeft
    (* 2    (length _rearLeft)) 4
    ;; int16[] _rearRight
    (* 2    (length _rearRight)) 4
    ;; int16[] _frontLeft
    (* 2    (length _frontLeft)) 4
    ;; int16[] _frontRight
    (* 2    (length _frontRight)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16[] _rearLeft
     (write-long (length _rearLeft) s)
     (dotimes (i (length _rearLeft))
       (write-word (elt _rearLeft i) s)
       )
     ;; int16[] _rearRight
     (write-long (length _rearRight) s)
     (dotimes (i (length _rearRight))
       (write-word (elt _rearRight i) s)
       )
     ;; int16[] _frontLeft
     (write-long (length _frontLeft) s)
     (dotimes (i (length _frontLeft))
       (write-word (elt _frontLeft i) s)
       )
     ;; int16[] _frontRight
     (write-long (length _frontRight) s)
     (dotimes (i (length _frontRight))
       (write-word (elt _frontRight i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16[] _rearLeft
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _rearLeft (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _rearLeft i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; int16[] _rearRight
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _rearRight (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _rearRight i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; int16[] _frontLeft
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _frontLeft (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _frontLeft i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; int16[] _frontRight
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _frontRight (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _frontRight i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;;
   self)
  )

(setf (get naoqi_driver::AudioCustomMsg :md5sum-) "bcac904b0030b6d70386338d161f4882")
(setf (get naoqi_driver::AudioCustomMsg :datatype-) "naoqi_driver/AudioCustomMsg")
(setf (get naoqi_driver::AudioCustomMsg :definition-)
      "# AudioCustomMsg.msg
int16[] rearLeft
int16[] rearRight
int16[] frontLeft
int16[] frontRight
")



(provide :naoqi_driver/AudioCustomMsg "bcac904b0030b6d70386338d161f4882")


