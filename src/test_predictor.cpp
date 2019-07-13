#include <trajectory_state_predictor/trajectory_state_predictor.h>

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
        }
        r.sleep();
    }
}
