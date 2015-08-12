#include <ros/ros.h>
#include <c2_ros/pilot.h>
#include <c2_ros/c2_agent.h>
#include <vector>

namespace C2{

class Pilot_Simulated: public Pilot
{
private:
	std::vector<c2_ros::MissionPoint> poseToRun;
	c2_ros::MissionPoint curMP;
	int poseCnt;
	bool isCompleted;
	bool isReinitialized;
	bool isCurMPReached;

public:
	Pilot_Simulated(int loopRate, ros::NodeHandle nh):
		Pilot(loopRate,nh),
		poseCnt(0),
		isCompleted(true),
		isReinitialized(true),
		isCurMPReached(true)
{
		ROS_INFO("Done, spin now ! ");
}

	~Pilot_Simulated()
	{

	}

	void tick()
	{
		if(!isCompleted)
		{
			if(isReinitialized)
			{
				ROS_INFO("reinitialized the pilot");
				isReinitialized = false;
				isCurMPReached = true;
				poseCnt = 0;
			}

			if(isCurMPReached){
				if(poseCnt < poseToRun.size())
				{
					curMP = poseToRun.at(poseCnt);
					ROS_INFO("Navigating to x=%f, y=%f",curMP.m_pt.x,curMP.m_pt.y);
					poseCnt++;
					isCurMPReached = false;
				}
				else
				{
					isCompleted = true;
					setMPCompleted(true);
					return;
				}
			}

			navigateTo(curMP);
		}
	}

	void navigateTo(c2_ros::MissionPoint mp)
	{
		//simulate succeed
		ROS_INFO("reached x=%f, y=%f",mp.m_pt.x,mp.m_pt.y);
		isCurMPReached = true;


		//simulate failure
		//isCompleted = true;
		//setMPCompleted(false);
	}

	void onStop()
	{
		poseToRun.clear();
		poseCnt = 0;
		isCompleted = true;
		isCurMPReached = true;
	}

	void newMissionPointAvailable(std::vector<c2_ros::MissionPoint> poses, bool isOverwrite)
	{
		ROS_INFO("Mission point received by [%s]",action_name_.c_str());
		if(isCompleted)
		{
			poseToRun = std::vector<c2_ros::MissionPoint>(poses);
			isCompleted = false;
			isCurMPReached = true;
			poseCnt = 0;
		}
		else
		{
			if(isOverwrite)
			{
				poseToRun = std::vector<c2_ros::MissionPoint>(poses);
				isReinitialized = true;
				poseCnt = 0;
			}
			else
			{
				std::vector<c2_ros::MissionPoint>::iterator it;
				for(it = poses.begin(); it != poses.end(); it++){
					poseToRun.push_back(*it);
				}
			}
		}
	}
};
}

int main (int argc, char ** argv)
{
	ros::init(argc, argv, C2::C2Agent(C2::C2Agent::PILOT).toString());
	ros::NodeHandle nh;
	C2::Pilot_Simulated c(10,nh);
	c.spin();

	return 0;
}
