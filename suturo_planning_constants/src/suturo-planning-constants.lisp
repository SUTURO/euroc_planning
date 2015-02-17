(in-package :constants)

(if (not (boundp '+suturo-planning-constants-defined+))
    (progn
      (defconstant +suturo-planning-constants-defined+ "" "If this variable is not void, every constant in this file is already defined")
      (defconstant +scan-obstacles-angle+ 1.2 "The angle the camera should look at the object. Is used to calculate the poses")
      (defconstant +scan-obstacles-distance-parameter-offset+ 0.6 "Used to calculate the distance beetween the camera and the object when the camera is looking at the object. Is used to calculate the poses")
      (defconstant +scan-obstacles-distance-parameter-factor+ 0.008 "Used to calculate the distance beetween the camera and the object when the camera is looking at the object. Is used to calculate the poses")
      (defconstant +scan-obstacles-number-of-poses+ 16 "The quantity of poses to be calculated")
      (defconstant +scan-obstacles-arm-max-distance+ 1.1 "The maximal distance the arm can or should reach. Objects out of this range will be ignored.")
      
))