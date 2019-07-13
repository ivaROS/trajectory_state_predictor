#include <trajectory_state_predictor/trajectory_state_predictor.h>
#include <opencv2/core/mat.hpp>
#include <sstream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_state_predictor_tester");

    TrajectoryStatePredictor pred;

    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    ros::Rate r(5);
    
    
    
    ros::Publisher pub = ros::NodeHandle().advertise<geometry_msgs::PoseStamped>("predicted_states",10);
    
    geometry_msgs::PoseStamped p;
    while(ros::ok())
    {
        if(pred.getRelativePose(ros::Time::now(), ros::Duration(-1), p));
        {
            pub.publish(p);
            
            cv::Mat m(4,4,CV_64F);
            pred.getRelativePose(ros::Time::now().toSec(), ros::Duration(-1).toSec(), m);
            //pred.getRelativePose(ros::Time::now(), ros::Duration(-1), m);
            
            
//             std::string matAsString (m.begin<unsigned char>(), m.end<unsigned char>());
//             std::stringstream ss;
//             ss << m;
//             ROS_INFO_STREAM("Mat: " << ss.str());
            ROS_INFO_STREAM(m.at<double>(0,0) << ", "  << m.at<double>(0,1) << ", "  << m.at<double>(0,2) << ", "  << m.at<double>(0,3));
            ROS_INFO_STREAM(m.at<double>(1,0) << ", "  << m.at<double>(1,1) << ", " << m.at<double>(1,2) << ", "  << m.at<double>(1,3));
            ROS_INFO_STREAM(m.at<double>(2,0) << ", "  << m.at<double>(2,1) << ", " << m.at<double>(2,2) << ", "  << m.at<double>(2,3));
            ROS_INFO_STREAM(m.at<double>(3,0) << ", "  << m.at<double>(3,1) << ", " << m.at<double>(3,2) << ", "  << m.at<double>(3,3));
            
        }
        r.sleep();
    }
}
