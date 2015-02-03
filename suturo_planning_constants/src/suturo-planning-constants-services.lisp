(in-package :constants)

(if (not (boundp '+suturo-planning-constants-services-defined+))
    (progn
      (defconstant +suturo-planning-constants-services-defined+ "" "If this variable is not void, every constant in this file is already defined")
      (defconstant +service-name-move-mastcam+ "/suturo/manipulation/move_mastcam" "The name of the service to move the mastcam")
      (defconstant +service-name-add-point-cloud+ "/suturo/add_point_cloud" "The name of the service to add a point cloud")
      (defconstant +service-name-move-robot+ "/suturo/manipulation/move" "The name of the service to move the robot")
      (defconstant +service-name-get-base-origin+ "/suturo/get_base_origin" "The name of the service to get the base origin")
      (defconstant +timeout-service+ 10 "The time to wait for a service")))
