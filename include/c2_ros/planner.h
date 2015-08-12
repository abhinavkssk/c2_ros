#ifndef PLANNER_H_
#define PLANNER_H_

#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <c2_ros/MissionLegAction.h>
#include <c2_ros/MissionPointAction.h>
#include <c2_ros/MissionLeg.h>

namespace C2 {


class Planner{

private:
	c2_ros::MissionLeg m_leg;
	bool mpointCompleted;
	bool toTick;
	int mp_progressPercentage;
	ros::Rate loop_rate;

	actionlib::SimpleActionServer<c2_ros::MissionLegAction> as_;
	void goal_callback();
	void preempt_callback();

	actionlib::SimpleActionClient<c2_ros::MissionPointAction> mpoint_client;
	void mpoint_result_callback(const actionlib::SimpleClientGoalState& state,
			const c2_ros::MissionPointResultConstPtr& result);
	void mpoint_active_callback();
	void mpoint_feedback_callback(const c2_ros::MissionPointFeedbackConstPtr& feedback);

protected:
	ros::NodeHandle nh_;

	//NOTE: Do not block these methods for too long, use boost::bind and boost::thread for your
	//own leisure if you are working with heavy stuffs !!
	virtual void tick() = 0;
	virtual void onGoalReceived() = 0;
	virtual void onStop() = 0;
	virtual void onMPointFailure() = 0;

	//server for Captain
	std::string action_name_;
	c2_ros::MissionLeg getMissionLeg();
	void setMLCompleted(bool isSucceeded);
	void sendMLProgress(int percentage_completed);
	bool isMPointCompleted();
	int getMPProgress();


	//client for pilot
	void sendMPoint(const c2_ros::MissionPoint& mpoint, bool isOverwrite = true);
	void sendMPoint(std::vector<c2_ros::MissionPoint> mpoints, bool isOverwrite = true);

public:
	Planner(std::string name, int loopRate, ros::NodeHandle nh);
	virtual ~Planner() = default;
	void spin();

};

}

#endif
