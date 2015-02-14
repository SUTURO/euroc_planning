(in-package :constants)

(if (not (boundp '+suturo-planning-constants-services-defined+))
    (progn
      (defconstant +suturo-planning-constants-services-defined+ "" "If this variable is not void, every constant in this file is already defined")
      (defconstant +service-name-move-mastcam+ "/suturo/manipulation/move_mastcam" "The name of the service to move the mastcam")
      (defconstant +service-name-add-point-cloud+ "/suturo/add_point_cloud" "The name of the service to add a point cloud")
      (defconstant +service-name-move-robot+ "/suturo/manipulation/move" "The name of the service to move the robot")
      (defconstant +service-name-get-base-origin+ "/suturo/get_base_origin" "The name of the service to get the base origin")
      (defconstant +service-name-get-obstacle-regions+ "/suturo/environment/get_obstacle_regions" "The name of the service to get the obstacle regions")
      (defconstant +service-name-get-map+ "/suturo/environment/get_map" "The name of the service to get the map")
      (defconstant +service-name-create-poses-for-object-scanning+ "/suturo/environment/create_poses_for_object_scanning" "The name of the service to get the poses for object scanning")
      (defconstant +service-name-euroc-object-to-odom-combined+ "/suturo/euroc_object_to_odom_combined" "The name of the service to convert an EurocObject the a odom combined one")
      (defconstant +service-name-classify-objects+ "suturo/Classifier" "The name of the service to classify objects")
      (defconstant +service-name-add-collision-objects+ "/suturo/manipulation/add_collision_objects" "The name of the service to add collision objects to the current scene")

      (defconstant +topic-name-get-yaml+ "suturo/yaml_pars0r" "The name of the topic to receive the yaml file")

      (defconstant +timeout-service+ 10 "The time to wait for a service")
      (defconstant +base-origin-topic+ "/suturo/manipulation/get_base_origin")))
