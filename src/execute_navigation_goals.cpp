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

/** \brief Read X and Y coordinates from a file and return them as a vector of points 
 *
 * Parse each line in a file for X and Y coordinate pairs. Each line in the file is expected to be composed of two
 * floats separated by a delimiter (by default, a space). An example line would look like this "1.0 3.2".
 * 
 * \param file_path the path to the file to be parsed for coordinates
 * \param delimiter (optional) the delimeter used in the file to separate the two coordinates (default, space)
 * \return a vector containing a point for each line in the file
 */
PointVector readXYCoordinates(string file_path, string delimiter = " ")
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

    points.push_back(p);
  }
  file.close();

  return points;
}

/** \brief Generate a random sequence of points sampled from a set of possible points 
 *
 * Given a vector of points, this function generates a random sequence of `n` samples from those points. The sequence is
 * generated making sure that the same point is never inserted in the sequence in two consecutive positions. Although 
 * there is no quarantee that all points in the set are going to be sampled at least once, a warning is issued if any 
 * point in the set has not been sampled.
 * 
 * \param points the set of points from which the sequence is sampled
 * \param n the number of points the sequence should be composed of
 * \param seed (optional) the seed to be used in the random generator
 * \return a vector containing a random sequence of `n` points
 */
PointVector randomPointSeq(PointVector points, unsigned int n, unsigned int seed = std::time(NULL))
{
  PointVector point_seq;
  int p, last_p = 0;
  size_t points_sz = points.size();
  vector<int> tally(points_sz, 0);

  std::srand(seed);
  for (int i = 0; i < n; i++)
  {
    p = std::rand() % (points_sz - 1);

    if (p >= last_p)
      p++;

    point_seq.push_back(points[p]);
    tally[p]++;
    last_p = p;
  }

  ROS_WARN_COND(
      find(tally.begin(), tally.end(), 0) != tally.end(),
      "Not all points from the set are part of the sequence");

  return point_seq;
}

/** \brief Write X and Y coordinates from a navigation path to a file 
 *
 * Write a navigation path in a file as pairs of X and Y coordinates. Each line in the file will be composed of two
 * floats separated by a delimiter (by default, a space). An example line would look like this "1.0 3.2".
 * 
 * \param path_ptr a pointer to the navigation path to be written
 * \param file_path the path to the file where to write the coordinates
 * \param delimiter (optional) the delimeter to be used in the file to separate the two coordinates (default, space)
 */
void writePath(PathPtr path_ptr, string file_path, string delimiter = " ")
{
  std::ofstream file;
  file.open(file_path);

  for (
      vector<geometry_msgs::PoseStamped>::const_iterator it = path_ptr->poses.begin();
      it != path_ptr->poses.end(); ++it)
  {
    geometry_msgs::Point p = it->pose.position;
    file << std::to_string(p.x) << delimiter.c_str() << std::to_string(p.y) << "\n";
  }

  file.close();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "execute_navigation_goals");

  // tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0)))
    ROS_INFO("Waiting for the move_base action server to come up");

  unsigned int n_iterations = 2;
  unsigned int seed = 15;
  string dir_path = "./";
  string file_name = "trajectory";
  string file_path = dir_path + file_name;
  string goals_file_path = "/home/fverdoja/turtle_ws/src/uncertain_turtlebot_navigation/maps/goals";
  PathPtr path_ptr;
  PointVector goal_points = readXYCoordinates(goals_file_path);
  PointVector goal_point_seq = randomPointSeq(goal_points, n_iterations, seed);

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.pose.orientation.w = 1.0; // all goals will have the same orientation

  int i = 0;
  for (PointVector::const_iterator pt = goal_point_seq.begin(); pt != goal_point_seq.end(); ++pt)
  {
    string file_path_i = file_path + "_" + std::to_string(i);

    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position = *pt;

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
