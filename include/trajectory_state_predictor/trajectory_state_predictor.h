#include <pips_trajectory_msgs/trajectory_points.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

class TrajectoryStatePredictor
{
public:
  TrajectoryStatePredictor();
  //bool getRelativePoses(double start_time, double offset1, double offset2, cv::Mat& t);
  bool getRelativePose(ros::Time now, ros::Duration duration, geometry_msgs::PoseStamped& p);
  
private:
  void trajectoryCB(const pips_trajectory_msgs::trajectory_points::ConstPtr& trajectory);
  
  
  pips_trajectory_msgs::trajectory_points::ConstPtr current_trajectory_;
  ros::Subscriber traj_sub_;

};
