# uncertain_turtlebot_navigation

A turtlebot navigation simulation in Gazebo using uncertainty maps.

The uncertainty maps used in this demo environment are constructed according to:

F. Verdoja, J. Lundell, and V. Kyrki, “Deep Network Uncertainty Maps for Indoor
Navigation,” arXiv:1809.04891 [cs, eess], Sep. 2018. 

Please refer to that paper for more details about the map building process.

## Use


*  First, you need to start a simulated turtlebot in an empty gazebo 
   environment:
   
   `roslaunch uncertain_turtlebot_navigation turtlebot-sim-empty.launch`
   
   If you don't want the gazebo gui to launch, add ` gui:=false`.

* Then, launch the navigation you prefer:
  
  * For normal slam navigation:
  
    `roslaunch uncertain_turtlebot_navigation slam_navigation.launch`

  * For uncertain navigation:
    
    `roslaunch uncertain_turtlebot_navigation uncertain_navigation.launch`
    
    The default uncertainty map in this case will be the one using a Laplace
    model, in alternative you can use the argument
    `uncertainty:={laplace|gaussian|mcdropout}` to select the uncertainty map to
    be used.

* Now you can either plan by setting navigation goals through rviz or by 
  executing a set of sample navigation goals by using:
  
  `roslaunch uncertain_turtlebot_navigation execute_navigation_goals.launch`

* Once you have run the navigation and stored the trajectories in the folder 
  */data/trajectories*, you can analyze the results, that is compute the number 
  of collisions that occured, by running the script `calculate_collisions` in 
  the *matlab* folder.
