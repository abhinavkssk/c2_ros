#include <ros/ros.h>
#include <c2_ros/c2_agent.h>
#include <c2_ros/planner.h>


namespace C2 {

class MBHV_Abort: public Planner
{
private:
	c2_ros_msgs::MissionLeg ml;
	bool isMLCompleted;

public:
	MBHV_Abort(std::string name, int loopRate, ros::NodeHandle nh):
		Planner(name,loopRate,nh),
		isMLCompleted(false)
{
		registerCapableBHV(c2_ros_msgs::C2_BHV::ABORT);
}

	~MBHV_Abort(){

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
				if(ml.m_state.twist.linear.x != 0.0)
				{
					ROS_INFO("[%s]:sending abort position to pilot",agentName.c_str());
					c2_ros_msgs::State3D p;
					p.pose = ml.m_state.pose;
					p.twist = ml.m_state.twist;
					p.m_pt_radius = ml.m_pt_radius;

					sendMPoint(p);
				}

				//only one point to send
				isMLCompleted = true;
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
	ros::init(argc, argv, C2::C2Agent(C2::C2Agent::MBHV_ABORTER).toString());
	ros::NodeHandle nh;
	C2::MBHV_Abort c(C2::C2Agent(C2::C2Agent::MBHV_ABORTER).toString(),1,nh);
	c.spin();

	return 0;
}
