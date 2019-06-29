#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <actionlib/client/simple_action_client.h>

using std::string;
using std::vector;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef boost::shared_ptr<nav_msgs::Path const> PathPtr;
typedef vector<geometry_msgs::Point> PointVector;

PointVector readXYGoals(string file_path, string delimiter = " ")
{
  PointVector points;
  geometry_msgs::Point p;
  string line;
  string::size_type sz; // alias of size_t
  std::ifstream file(file_path);

  ROS_ASSERT_MSG(file.is_open(), "File %s could not be found or opened.", file_path.c_str());
  while (getline(file, line))
  {
    p.x = std::stof(line, &sz);
    p.y = std::stof(line.substr(sz));
    //ROS_INFO("Goal: [%f, %f]", p.x, p.y);
    points.push_back(p);
  }
  file.close();

  return points;
}

PointVector randomGoalSeq(PointVector goals, unsigned int n, unsigned int seed)
{
  PointVector points;
  int p, last_p = 0;
  int goals_sz = goals.size();

  std::srand(seed);
  for (int i = 0; i < n; i++)
  {
    p = std::rand() % (goals_sz - 1);

    if (p >= last_p)
      p++;

    points.push_back(goals[p]);
    last_p = p;
  }

  return points;
}

void writePath(PathPtr path_ptr, string file_path)
{
  std::ofstream file;
  file.open(file_path);

  for (vector<geometry_msgs::PoseStamped>::const_iterator it = path_ptr->poses.begin(); it != path_ptr->poses.end(); ++it)
  {
    geometry_msgs::Point p = it->pose.position;
    file << std::to_string(p.x) << " " << std::to_string(p.y) << "\n";
    //ROS_INFO("Path: [%f, %f]", p.x, p.y);
  }

  file.close();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "execute_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0)))
    ROS_INFO("Waiting for the move_base action server to come up");

  unsigned int n_iterations = 5;
  unsigned int seed = 15;
  string dir_path = "./";
  string file_name = "trajectory";
  string file_path = dir_path + file_name;
  string goals_file_path = "/home/fverdoja/turtle_ws/src/uncertain_turtlebot_navigation/maps/goals";
  PathPtr path_ptr;
  PointVector goal_points = readXYGoals(goals_file_path);
  PointVector goal_seq = randomGoalSeq(goal_points, n_iterations, seed);

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";

  int i = 0;
  for (PointVector::const_iterator pt = goal_seq.begin(); pt != goal_seq.end(); ++pt)
  {
    string file_path_i = file_path + "_" + std::to_string(i);
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = pt->x;
    goal.target_pose.pose.position.y = pt->y;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal n. %d", i);
    ac.sendGoal(goal);

    path_ptr = ros::topic::waitForMessage<nav_msgs::Path>("/move_base/NavfnROS/plan", ros::Duration(10));
    if (path_ptr == NULL)
      ROS_WARN("No path messages received");
    else
    {
      ROS_INFO("Path received. Writing to %s", file_path_i.c_str());
      writePath(path_ptr, file_path_i);
    }

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Goal n. %d reached", i);
    else
      ROS_WARN("Goal n. %d aborted for some reason", i);

    i++;
  }

  return 0;
}
