;; Auto-generated. Do not edit!


(when (boundp 'qingzhou_locate::RobotLocation)
  (if (not (find-package "QINGZHOU_LOCATE"))
    (make-package "QINGZHOU_LOCATE"))
  (shadow 'RobotLocation (find-package "QINGZHOU_LOCATE")))
(unless (find-package "QINGZHOU_LOCATE::ROBOTLOCATION")
  (make-package "QINGZHOU_LOCATE::ROBOTLOCATION"))
(unless (find-package "QINGZHOU_LOCATE::ROBOTLOCATIONREQUEST")
  (make-package "QINGZHOU_LOCATE::ROBOTLOCATIONREQUEST"))
(unless (find-package "QINGZHOU_LOCATE::ROBOTLOCATIONRESPONSE")
  (make-package "QINGZHOU_LOCATE::ROBOTLOCATIONRESPONSE"))

(in-package "ROS")





(defclass qingzhou_locate::RobotLocationRequest
  :super ros::object
  :slots ())

(defmethod qingzhou_locate::RobotLocationRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass qingzhou_locate::RobotLocationResponse
  :super ros::object
  :slots (_location ))

(defmethod qingzhou_locate::RobotLocationResponse
  (:init
   (&key
    ((:location __location) 0)
    )
   (send-super :init)
   (setq _location (round __location))
   self)
  (:location
   (&optional __location)
   (if __location (setq _location __location)) _location)
  (:serialization-length
   ()
   (+
    ;; int32 _location
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _location
       (write-long _location s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _location
     (setq _location (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass qingzhou_locate::RobotLocation
  :super ros::object
  :slots ())

(setf (get qingzhou_locate::RobotLocation :md5sum-) "5ee1757f0484d31829b2fbdfc3bd23cd")
(setf (get qingzhou_locate::RobotLocation :datatype-) "qingzhou_locate/RobotLocation")
(setf (get qingzhou_locate::RobotLocation :request) qingzhou_locate::RobotLocationRequest)
(setf (get qingzhou_locate::RobotLocation :response) qingzhou_locate::RobotLocationResponse)

(defmethod qingzhou_locate::RobotLocationRequest
  (:response () (instance qingzhou_locate::RobotLocationResponse :init)))

(setf (get qingzhou_locate::RobotLocationRequest :md5sum-) "5ee1757f0484d31829b2fbdfc3bd23cd")
(setf (get qingzhou_locate::RobotLocationRequest :datatype-) "qingzhou_locate/RobotLocationRequest")
(setf (get qingzhou_locate::RobotLocationRequest :definition-)
      "
---
int32 location
")

(setf (get qingzhou_locate::RobotLocationResponse :md5sum-) "5ee1757f0484d31829b2fbdfc3bd23cd")
(setf (get qingzhou_locate::RobotLocationResponse :datatype-) "qingzhou_locate/RobotLocationResponse")
(setf (get qingzhou_locate::RobotLocationResponse :definition-)
      "
---
int32 location
")



(provide :qingzhou_locate/RobotLocation "5ee1757f0484d31829b2fbdfc3bd23cd")


