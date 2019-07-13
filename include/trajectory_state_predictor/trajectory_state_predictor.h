#include <pips_trajectory_msgs/trajectory_points.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace cv
{
  class Mat;
}

class TrajectoryStatePredictor
{
public:
  TrajectoryStatePredictor();
  bool getRelativePose(double start_time, double offset, cv::Mat& t);
  bool getRelativePose(ros::Time now, ros::Duration duration, geometry_msgs::PoseStamped& p);
  bool getRelativePose(ros::Time now, ros::Duration duration, cv::Mat& t);
  
private:
  void trajectoryCB(const pips_trajectory_msgs::trajectory_points::ConstPtr& trajectory);
  
  
  pips_trajectory_msgs::trajectory_points::ConstPtr current_trajectory_;
  ros::Subscriber traj_sub_;

};
