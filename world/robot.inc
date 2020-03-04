# Author: Vitor Machado Guilherme Barros

define pioneer_base position 
(
  color "red"                   
  drive "omni"                  
  gui_nose 1                    
  obstacle_return 1             
  blob_return 1                   
  fiducial_return 1             

  localization "gps"            
  localization_origin [0 0 0 0] 

  velocity_bounds [-0.5 0.5 0 0 0 0 -90.0 90.0 ]                    
  acceleration_bounds [-0.5 0.5 0 0 0 0 -90 90.0 ]
)

define robo_laser ranger
(
	sensor(
		range [ 0.0 3.0 ]
		fov 360
		samples 360
	)
	color "black"
	size [ 0.05 0.05 0.1 ]
)

define robot pioneer_base
(
  # actual size
  size [0.44 0.38 0.22] # sizes from MobileRobots' web site

  origin [-0.04 0 0 0]

  gui_nose 1

  mass 23.0 
  drive "omni"
  block( 
    points 8
    point[0] [-0.2 0.12]
    point[1] [-0.2 -0.12]
    point[2] [-0.12 -0.2555]
    point[3] [0.12 -0.2555]
    point[4] [0.2 -0.12]
    point[5] [0.2 0.12]
    point[6] [0.12 0.2555]
    point[7] [-0.12 0.2555]
    z [0 0.22]
  )
  robo_laser(pose [0 0 0 0 ])
)
