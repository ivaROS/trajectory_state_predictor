#include <trajectory_state_predictor/trajectory_state_predictor.h>

#include <tf/transform_datatypes.h>
#include <pips_trajectory_msgs/trajectory_point.h>
#include <pips_trajectory_msgs/trajectory_points.h>
//#include <tf2/convert.h>  //needed to give access to templates



void getRelativePose(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2, geometry_msgs::PoseStamped& p)
{
  tf::Transform t1,t2,t3;
  tf::poseMsgToTF(p1.pose,t1);
  tf::poseMsgToTF(p2.pose,t2);
  t3 = t1.inverseTimes(t2);
  
  //could also use poseStampedTFToMsg
  
  tf::poseTFToMsg(t3, p.pose);
  p.header = p2.header;
}


bool getDesiredState(pips_trajectory_msgs::trajectory_points trajectory, ros::Time stamp, geometry_msgs::PoseStamped& posestamped)
{
  pips_trajectory_msgs::trajectory_point pre_point, post_point;
  double pre_time_fraction = 1;
  double t;
  int num_points;
  int post_index;
  int curr_index_=0;
  
  std_msgs::Header header;
  header.stamp = stamp;
  
  {
    num_points = trajectory.points.size();
    header.frame_id = trajectory.header.frame_id;
    
    ros::Duration elapsed_time = stamp - trajectory.header.stamp;
    t = elapsed_time.toSec();
    
    //This updates curr_index to refer to the last trajectory point before the desired time.
    for(; curr_index_ < num_points -1 && trajectory.points[curr_index_+1].time < elapsed_time; curr_index_++);
    
    if(curr_index_ == num_points)
    {
      ROS_ERROR("Desired time goes beyond the end of the trajectory!");
      return false;
    }
    //This handles the case where we've reached the end of the trajectory time
    post_index = std::min(curr_index_+1, num_points-1);
    
    pre_point = trajectory.points[curr_index_];
    post_point = trajectory.points[post_index];
    
    ros::Duration pre_time = elapsed_time - pre_point.time;
    ros::Duration period = post_point.time - pre_point.time;
    
    
    if(curr_index_ < num_points-1)
    {
      pre_time_fraction = pre_time.toSec()/period.toSec();
    }
    
    if(curr_index_ == num_points-1)
    {
      curr_index_ = -1;
    }
  }
  
  double x = pre_point.x*(1-pre_time_fraction) + post_point.x*pre_time_fraction;
  double y = pre_point.y*(1-pre_time_fraction) + post_point.y*pre_time_fraction;;
  double theta = pre_point.theta*(1-pre_time_fraction) + post_point.theta*pre_time_fraction;
  
  geometry_msgs::Pose pose;
  
  // Position
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.0;
  
  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(theta);
  
  pose.orientation = quat;
  
  ROS_DEBUG_STREAM("Index: " << curr_index_ << "; # points: " << num_points); 
  ROS_DEBUG_STREAM("Preindex: " << curr_index_ << "; postindex: " << post_index);  
  ROS_DEBUG_STREAM("Desired@ " << t << "s: (" << x << "," << y << ") and " << quat.w <<"," << quat.z);
  
  posestamped.pose = pose;
  posestamped.header = header;
}



TrajectoryStatePredictor::TrajectoryStatePredictor()
{
  ros::NodeHandle nh;
  traj_sub_ = nh.subscribe("trajectory", 1, &TrajectoryStatePredictor::trajectoryCB, this);
}

bool TrajectoryStatePredictor::getRelativePose(ros::Time now, ros::Duration duration, geometry_msgs::PoseStamped& p)
{
  pips_trajectory_msgs::trajectory_points::ConstPtr trajectoryPtr = current_trajectory_;
  if(!trajectoryPtr)
  {
    ROS_ERROR("No trajectory has been received yet!");
    return false;
  }
  else
  {
    const pips_trajectory_msgs::trajectory_points& trajectory = *trajectoryPtr;
    geometry_msgs::PoseStamped p1,p2;
    if(getDesiredState(trajectory, now, p1) && getDesiredState(trajectory, now+duration, p2))
    {
      ::getRelativePose(p1,p2,p);
      return true;
    }
  }
  return false;
}

// bool getRelativePoses(double start_time, double offset1, double offset2, cv::Mat& t)
// {
//   
// }

void TrajectoryStatePredictor::trajectoryCB(const pips_trajectory_msgs::trajectory_points::ConstPtr& trajectory)
{
  ROS_DEBUG_STREAM("Received trjaectory with timestamp " << trajectory->header.stamp);
  current_trajectory_ = trajectory;
}


