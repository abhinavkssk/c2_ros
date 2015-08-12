#ifndef PILOT_H_
#define PILOT_H_

#include <ros/ros.h>
#include <vector>

#include <actionlib/server/simple_action_server.h>
#include <c2_ros/MissionPointAction.h>

namespace C2{

class Pilot{

private:
	ros::Rate loop_rate;

	//server for Planners
	actionlib::SimpleActionServer<c2_ros::MissionPointAction> as_;
	void goal_callback();
	void preempt_callback();

protected:
	ros::NodeHandle nh_;
	std::string agentName;
	virtual void tick() = 0;
	virtual void onStop() = 0;
	virtual void newMissionPointAvailable(std::vector<c2_ros::MissionPoint> mpoints, bool isOverwrite) = 0;
	void setMPCompleted(bool isSucceeded);
	void sendMPProgress(int percentage_completed);

public:
	Pilot(int loopRate, ros::NodeHandle nh);
	virtual ~Pilot() = default;
	void spin();

};
}

#endif
