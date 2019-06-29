# uncertain_turtlebot_navigation

A turtlebot navigation simulation in Gazebo using uncertainty maps.


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

* Now you can either plan setting navigation goals through rviz or by executing a
  set of sample navigation goals by using:
  
  `roslaunch uncertain_turtlebot_navigation execute_navigation_goals.launch`


