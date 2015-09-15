#include <ros/ros.h>
#include <c2_ros/c2_agent.h>
#include <c2_ros/planner.h>
#include <iostream>


namespace C2 {

class MBHV_WayPoint: public Planner
{
private:
	c2_ros_msgs::MissionLeg ml;
	bool isMLCompleted;

public:
	MBHV_WayPoint(std::string name, int loopRate, ros::NodeHandle nh):
		Planner(name,loopRate,nh),
		isMLCompleted(false)
{
		registerCapableBHV(c2_ros_msgs::C2_BHV::WAY_POINT);
}

	~MBHV_WayPoint(){

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
				ROS_INFO("sending MPoints");
				c2_ros_msgs::State3D p;
				p.pose = ml.m_state.pose;
				p.twist = ml.m_state.twist;
				p.m_pt_radius = ml.m_pt_radius;

				sendMPoint(p);

				//only one point to send
				isMLCompleted = true;
			}
		}
		else
		{
			//ROS_INFO("[%s]:Pilot working hard...",action_name_.c_str());
		}
	}

	void onStop(){
		isMLCompleted = false;
	}

	void onMPointFailure(){
		setMLCompleted(false);
		isMLCompleted = false;
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
