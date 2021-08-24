;; Auto-generated. Do not edit!


(when (boundp 'aruco::Angle)
  (if (not (find-package "ARUCO"))
    (make-package "ARUCO"))
  (shadow 'Angle (find-package "ARUCO")))
(unless (find-package "ARUCO::ANGLE")
  (make-package "ARUCO::ANGLE"))

(in-package "ROS")
;;//! \htmlinclude Angle.msg.html


(defclass aruco::Angle
  :super ros::object
  :slots (_angle_base _angle_aruco _angle_cam ))

(defmethod aruco::Angle
  (:init
   (&key
    ((:angle_base __angle_base) 0.0)
    ((:angle_aruco __angle_aruco) 0.0)
    ((:angle_cam __angle_cam) 0.0)
    )
   (send-super :init)
   (setq _angle_base (float __angle_base))
   (setq _angle_aruco (float __angle_aruco))
   (setq _angle_cam (float __angle_cam))
   self)
  (:angle_base
   (&optional __angle_base)
   (if __angle_base (setq _angle_base __angle_base)) _angle_base)
  (:angle_aruco
   (&optional __angle_aruco)
   (if __angle_aruco (setq _angle_aruco __angle_aruco)) _angle_aruco)
  (:angle_cam
   (&optional __angle_cam)
   (if __angle_cam (setq _angle_cam __angle_cam)) _angle_cam)
  (:serialization-length
   ()
   (+
    ;; float32 _angle_base
    4
    ;; float32 _angle_aruco
    4
    ;; float32 _angle_cam
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _angle_base
       (sys::poke _angle_base (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle_aruco
       (sys::poke _angle_aruco (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle_cam
       (sys::poke _angle_cam (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _angle_base
     (setq _angle_base (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle_aruco
     (setq _angle_aruco (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle_cam
     (setq _angle_cam (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get aruco::Angle :md5sum-) "d2b3e9a194f5925c37a312cac19f86be")
(setf (get aruco::Angle :datatype-) "aruco/Angle")
(setf (get aruco::Angle :definition-)
      "
float32 angle_base
float32 angle_aruco
float32 angle_cam
")



(provide :aruco/Angle "d2b3e9a194f5925c37a312cac19f86be")


