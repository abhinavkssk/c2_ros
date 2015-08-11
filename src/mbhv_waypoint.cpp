#include <ros/ros.h>
#include <c2_ros/c2_agent.h>
#include <c2_ros/planner.h>
#include <iostream>


namespace C2 {

class MBHV_WayPoint: public Planner
{
private:
	c2_ros::MissionLeg ml;
	bool isMLCompleted;

public:
	MBHV_WayPoint(std::string name, int loopRate, ros::NodeHandle nh):
		Planner(name,loopRate,nh),
		isMLCompleted(false)
{

}

	~MBHV_WayPoint(){

	}

	void onGoalReceived(){
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
				ROS_INFO("sending MPoints");
				geometry_msgs::PoseStamped p;
				p.pose.position.x = ml.lat;
				p.pose.position.y = ml.lon;

				sendMPoint(p);

				//only one point to send
				isMLCompleted = true;
			}
		}
		else
		{
			ROS_INFO("Pilot working hard...");
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
	ros::init(argc, argv, C2::C2Agent(C2::C2Agent::MBHV_WAYPOINTER).toString());
	ros::NodeHandle nh;
	C2::MBHV_WayPoint c(C2::C2Agent(C2::C2Agent::MBHV_WAYPOINTER).toString(),1,nh);
	c.spin();

	return 0;
}
