#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <c2_ros/c2_agent.h>
#include <c2_ros/planner.h>
#include <asco_utils/utils.h>


namespace C2 {

class MBHV_LawnMow: public Planner
{
private:
	ros::Subscriber odom_est_sub;
	c2_ros::MissionLeg ml;
	geometry_msgs::Pose2D curPos;
	double curBearing;
	bool isMLCompleted;

public:
	MBHV_LawnMow(std::string name, int loopRate, ros::NodeHandle nh):
		Planner(name,loopRate,nh),
		curBearing(0),
		isMLCompleted(false){
		registerCapableBHV(c2_ros::C2_BHV::LAWNMOW);

		//subscribe to vehicle state
		std::string odm_name;
		if (!nh_.getParam("/c2_params/odometry_topic_name",odm_name)) odm_name = "/odometry/filtered";
		odom_est_sub = nh_.subscribe(odm_name,1, &C2::MBHV_LawnMow::odom_est,this);
	}

	~MBHV_LawnMow(){

	}

	void odom_est(const nav_msgs::Odometry::ConstPtr& odom_pos_est)
	{
		curPos.x = odom_pos_est->pose.pose.position.x;
		curPos.y = odom_pos_est->pose.pose.position.y;
		curPos.theta = tf::getYaw(odom_pos_est->pose.pose.orientation);
		curBearing = asco::Utils::yaw2bearing(curPos.theta);
	}

	void onGoalReceived(){
		isMLCompleted = false;
		ml = getMissionLeg();
	}

	void tick(){
		if(isMPointCompleted())
		{
			if(isMLCompleted)
			{
				setMLCompleted(true);
				isMLCompleted = false;
			}
			else
			{

				//only one set of points to send
				isMLCompleted = true;
			}
		}
	}

	void onStop(){
		isMLCompleted = false;
	}

	void onMPointFailure(){
		setMLCompleted(false);
		isMLCompleted = false;
	}

	void generateLMPath()
	{

	}

};

}

int main (int argc, char ** argv)
{
	ros::init(argc, argv, C2::C2Agent(C2::C2Agent::MBHV_LAWNMOWER).toString());
	ros::NodeHandle nh;
	C2::MBHV_LawnMow c(C2::C2Agent(C2::C2Agent::MBHV_LAWNMOWER).toString(),1,nh);
	c.spin();

	return 0;
}
