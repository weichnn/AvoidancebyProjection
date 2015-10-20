# AvoidancebyProjection

  This project is a simulation for a algorith, which use brushfire algorithm to solve the real-time problem. The algorithm consists of two steps: path planning and trajectory generation. Path planning is used to get a sequence of waypoints. Trajectory generation takes use of planning result to generate trajectory under differential constraint. Brushfire algorithm is to process simple result and get non-collision path for local space. I also use minimum-snap objection for trajectory to avoid abrupt and excessive control inputs.
  
  
  In simulation , I went through the free-running operation experiments to follow global path to generate local path that the quadrotor did. These experiments all had good results and help verify the algorithm that can deal with local planning issues. As for the time of the algorithm taking, which this paper introduces, is much less than standard RRT algorithm’s need in compalicate experiment.

###platform
ubuntu and ROS hydro
