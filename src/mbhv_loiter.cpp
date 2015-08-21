#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include <c2_ros/c2_agent.h>
#include <c2_ros/planner.h>
#include <asco_utils/utils.h>


namespace C2 {
//perpetual until abort

class MBHV_Loiter: public Planner
{
private:
	ros::Subscriber odom_est_sub;
	c2_ros::MissionLeg ml;
	geometry_msgs::Pose2D curPos;
	double curBearing;

public:
	MBHV_Loiter(std::string name, int loopRate, ros::NodeHandle nh):
		Planner(name,loopRate,nh),
		curBearing(0){
		registerCapableBHV(c2_ros::C2_BHV::LOITER);

		//subscribe to vehicle state
		std::string odm_name;
		if (!nh_.getParam("/c2_params/odometry_topic_name",odm_name)) odm_name = "/odometry/filtered";
		odom_est_sub = nh_.subscribe(odm_name,1, &C2::MBHV_Loiter::odom_est,this);
	}

	~MBHV_Loiter(){

	}

	void odom_est(const nav_msgs::Odometry::ConstPtr& odom_pos_est)
	{
		curPos.x = odom_pos_est->pose.pose.position.x;
		curPos.y = odom_pos_est->pose.pose.position.y;
		curPos.theta = tf::getYaw(odom_pos_est->pose.pose.orientation);
		curBearing = asco::Utils::yaw2bearing(curPos.theta);
	}

	void onGoalReceived(){
		ml = getMissionLeg();
	}

	void tick(){
		if(isMPointCompleted())
		{
			//check the distance between curMissionLeg pos and curPos
			if(asco::Utils::getDist2D(curPos,ml.m_state.pose) > 15)
			{
				c2_ros::State3D p;
				p.pose = ml.m_state.pose;
				p.twist = ml.m_state.twist;
				p.m_pt_radius = ml.m_pt_radius;

				sendMPoint(p);
			}
		}
	}

	void onStop(){

	}

	void onMPointFailure(){
		setMLCompleted(false);
	}

};

}

int main (int argc, char ** argv)
{
	ros::init(argc, argv, C2::C2Agent(C2::C2Agent::MBHV_LOITER).toString());
	ros::NodeHandle nh;
	C2::MBHV_Loiter c(C2::C2Agent(C2::C2Agent::MBHV_LOITER).toString(),1,nh);
	c.spin();

	return 0;
}
