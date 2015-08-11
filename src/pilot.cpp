//Pilot base class

#include <ros/ros.h>
#include <c2_ros/pilot.h>
#include <c2_ros/c2_agent.h>
#include <c2_ros/MissionPointGoal.h>

using C2::Pilot;

Pilot::Pilot(int loopRate, ros::NodeHandle nh):
												nh_(nh),
												action_name_(C2::C2Agent(C2::C2Agent::PILOT).toString()),
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
	newMissionPointAvailable(goal->mission_pt,goal->isOverwrite);
}

void Pilot::preempt_callback()
{
	ROS_INFO("Preempt requested in [%s]",action_name_.c_str());
	onStop();

	//reply planner's request
	as_.setPreempted();
}

void Pilot::setMPCompleted(bool isSucceeded)
{
	c2_ros::MissionPointResult result;
	if(isSucceeded)
	{
		ROS_INFO("[%s]'s mission points succeeded",action_name_.c_str());
		result.isSucceeded = true;
		as_.setSucceeded(result);

	}
	else
	{
		ROS_INFO("[%s]'s mission points failed",action_name_.c_str());
		result.isSucceeded = false;
		as_.setAborted(result);
	}

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
