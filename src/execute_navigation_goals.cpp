#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef boost::shared_ptr<nav_msgs::Path const> PathPtr;
typedef std::vector<geometry_msgs::Point> PointVector;

PointVector generateGoals() {
  PointVector points;
  geometry_msgs::Point p1, p2;

  p1.x = 0.0;
  p1.y = 0.0;
  p2.x = 10.8;
  p2.y = 5.6;

  points.push_back(p1);
  points.push_back(p2);

  return points;
}

void writePath(PathPtr path_ptr) {
  for(std::vector<geometry_msgs::PoseStamped>::const_iterator it = path_ptr->poses.begin(); it != path_ptr->poses.end(); ++it) {
    geometry_msgs::Point p = it->pose.position;
    ROS_INFO("Path: [%f, %f, %f]", p.x, p.y, p.z);
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  PathPtr path_ptr;
  PointVector goal_points = generateGoals();
  
  goal.target_pose.header.frame_id = "map";
  
  for(PointVector::const_iterator pt = goal_points.begin(); pt != goal_points.end(); ++pt) {
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = pt->x;
    goal.target_pose.pose.position.y = pt->y;
    goal.target_pose.pose.orientation.w = 1.0;
  
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    path_ptr = ros::topic::waitForMessage<nav_msgs::Path>("/move_base/NavfnROS/plan", ros::Duration(10));
    if (path_ptr == NULL)
      ROS_WARN("No path messages received");
    else
      writePath(path_ptr);
  
    ac.waitForResult();
    
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Goal reached");
    else
      ROS_WARN("Goal aborted for some reason");

  }

  return 0;
}
