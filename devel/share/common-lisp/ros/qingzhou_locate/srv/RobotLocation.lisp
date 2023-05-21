; Auto-generated. Do not edit!


(cl:in-package qingzhou_locate-srv)


;//! \htmlinclude RobotLocation-request.msg.html

(cl:defclass <RobotLocation-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass RobotLocation-request (<RobotLocation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotLocation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotLocation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qingzhou_locate-srv:<RobotLocation-request> is deprecated: use qingzhou_locate-srv:RobotLocation-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotLocation-request>) ostream)
  "Serializes a message object of type '<RobotLocation-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotLocation-request>) istream)
  "Deserializes a message object of type '<RobotLocation-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotLocation-request>)))
  "Returns string type for a service object of type '<RobotLocation-request>"
  "qingzhou_locate/RobotLocationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotLocation-request)))
  "Returns string type for a service object of type 'RobotLocation-request"
  "qingzhou_locate/RobotLocationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotLocation-request>)))
  "Returns md5sum for a message object of type '<RobotLocation-request>"
  "5ee1757f0484d31829b2fbdfc3bd23cd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotLocation-request)))
  "Returns md5sum for a message object of type 'RobotLocation-request"
  "5ee1757f0484d31829b2fbdfc3bd23cd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotLocation-request>)))
  "Returns full string definition for message of type '<RobotLocation-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotLocation-request)))
  "Returns full string definition for message of type 'RobotLocation-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotLocation-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotLocation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotLocation-request
))
;//! \htmlinclude RobotLocation-response.msg.html

(cl:defclass <RobotLocation-response> (roslisp-msg-protocol:ros-message)
  ((location
    :reader location
    :initarg :location
    :type cl:integer
    :initform 0))
)

(cl:defclass RobotLocation-response (<RobotLocation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotLocation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotLocation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qingzhou_locate-srv:<RobotLocation-response> is deprecated: use qingzhou_locate-srv:RobotLocation-response instead.")))

(cl:ensure-generic-function 'location-val :lambda-list '(m))
(cl:defmethod location-val ((m <RobotLocation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qingzhou_locate-srv:location-val is deprecated.  Use qingzhou_locate-srv:location instead.")
  (location m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotLocation-response>) ostream)
  "Serializes a message object of type '<RobotLocation-response>"
  (cl:let* ((signed (cl:slot-value msg 'location)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotLocation-response>) istream)
  "Deserializes a message object of type '<RobotLocation-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'location) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotLocation-response>)))
  "Returns string type for a service object of type '<RobotLocation-response>"
  "qingzhou_locate/RobotLocationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotLocation-response)))
  "Returns string type for a service object of type 'RobotLocation-response"
  "qingzhou_locate/RobotLocationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotLocation-response>)))
  "Returns md5sum for a message object of type '<RobotLocation-response>"
  "5ee1757f0484d31829b2fbdfc3bd23cd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotLocation-response)))
  "Returns md5sum for a message object of type 'RobotLocation-response"
  "5ee1757f0484d31829b2fbdfc3bd23cd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotLocation-response>)))
  "Returns full string definition for message of type '<RobotLocation-response>"
  (cl:format cl:nil "int32 location~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotLocation-response)))
  "Returns full string definition for message of type 'RobotLocation-response"
  (cl:format cl:nil "int32 location~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotLocation-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotLocation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotLocation-response
    (cl:cons ':location (location msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RobotLocation)))
  'RobotLocation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RobotLocation)))
  'RobotLocation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotLocation)))
  "Returns string type for a service object of type '<RobotLocation>"
  "qingzhou_locate/RobotLocation")