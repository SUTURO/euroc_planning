(in-package :manipulation)

(if (not (boundp '+suturo-pm-manipulation-defined+))
  (progn
    (defconstant +suturo-pm-manipulation-defined+ "")  
    (defconstant +finger-length+ 0.04 "The length of the 'finger' of the robot")
    (defconstant +hand-length+ 0.183 "The length of the hand")
   
    (defconstant +pre-grasp-length+ 0.1 "Move the finger right in front of the object with this distance")
    (defconstant +pre-place-length+ 0.06 "This parameter will be added to the place pose to be right above the placing spot")
    (defconstant +post-place-length+ 0.1 "After placing an object move up the arm by this distance")
    (defconstant +safe-place+ 0.005)
    
    (defconstant +gripper-max-pose+ 0.0345 "The maximum distance the gripper can open")
))
