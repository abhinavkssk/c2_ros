//Pilot base class

#include <ros/ros.h>
#include <c2_ros/pilot.h>
#include <c2_ros/c2_agent.h>
#include <c2_ros/MissionPointGoal.h>

using C2::Pilot;

Pilot::Pilot(int loopRate, ros::NodeHandle nh):
												nh_(nh),
												agentName(C2::C2Agent(C2::C2Agent::PILOT).toString()),
												loop_rate(loopRate),
												as_(nh,C2::C2Agent(C2::C2Agent::PILOT).toString(),false)
{
	//register the goal and feeback callbacks
	as_.registerGoalCallback(boost::bind(&Pilot::goal_callback, this));
	as_.registerPreemptCallback(boost::bind(&Pilot::preempt_callback, this));
	as_.start();
}

void Pilot::goal_callback()
{
	auto goal = as_.acceptNewGoal();
	newMissionPointAvailable(goal->mission_traj,goal->isOverwrite);
}

void Pilot::preempt_callback()
{
	ROS_INFO("Preempt requested in [%s]",agentName.c_str());
	onStop();

	//reply planner's request
	as_.setPreempted();
}

void Pilot::setMPCompleted(bool isSucceeded)
{
	c2_ros::MissionPointResult result;
	if(isSucceeded)
	{
		ROS_INFO("[%s]'s mission points succeeded",agentName.c_str());
		result.isSucceeded = true;
		as_.setSucceeded(result);

	}
	else
	{
		ROS_INFO("[%s]'s mission points failed",agentName.c_str());
		result.isSucceeded = false;
		as_.setAborted(result);
	}

}

void Pilot::sendMPProgress(int percentage_completed)
{
	c2_ros::MissionPointFeedback feedback;
	feedback.progress = percentage_completed;
	as_.publishFeedback(feedback);
}

//this function must be called !!
void Pilot::spin(){
	//iterate
	while (ros::ok())
	{
		tick();
		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::shutdown();
}
