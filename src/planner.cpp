//base class for all the mission behaviors

#include <ros/ros.h>
#include <c2_ros/planner.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <c2_ros/MissionLegAction.h>
#include <c2_ros/MissionPointAction.h>
#include <c2_ros_msgs/MissionLeg.h>
#include <c2_ros/c2_agent.h>
#include <c2_ros_msgs/BHVProposer.h>

using C2::Planner;

Planner::Planner(std::string name, int loopRate, ros::NodeHandle nh):
								nh_(nh),
								as_(nh,name,false), // call Planner::spin() manually
								agentName(name),
								mpoint_client(C2::C2Agent(C2::C2Agent::PILOT).toString(),true), //call Planner::spin() manually
								loop_rate(loopRate),
								mpointCompleted(false),
								toTick(false),
								mp_progressPercentage(0)
{
	//register the goal and feeback callbacks
	as_.registerGoalCallback(boost::bind(&Planner::goal_callback, this));
	as_.registerPreemptCallback(boost::bind(&Planner::preempt_callback, this));
	as_.start();


	//connect to pilot server
	if(!mpoint_client.waitForServer(ros::Duration(10,0)))
		ROS_WARN("[%s]:Pilot server can not be connected !",agentName.c_str());
	else
		ROS_INFO("[%s]:Pilot server established, continue ...",agentName.c_str());

	//sub and pub bhv
	bhv_request_sub = nh_.subscribe("/captain/bhv_request",100, &Planner::bhv_request_callback,this);
	bhv_propose_pub = nh_.advertise<c2_ros_msgs::BHVProposer>("/captain/bhv_propose",100);
}

void Planner::bhv_request_callback(const c2_ros_msgs::C2_BHV::ConstPtr& bhv_request)
{
	std::vector<c2_ros_msgs::C2_BHV::_bhv_type>::iterator it;

	it = find (capable_bhv.begin(), capable_bhv.end(), bhv_request->bhv);
	if (it != capable_bhv.end())
	{
		//publish to let Captain know
		c2_ros_msgs::BHVProposer p;
		p.name = agentName;
		bhv_propose_pub.publish(p);
	}
}

//this function must be called !!
void Planner::spin(){
	//iterate
	while (ros::ok())
	{
		if(toTick)
			tick();
		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::shutdown();
}

void Planner::sendMPoint(const c2_ros_msgs::State3D& state3D, bool isOverwrite)
{
	c2_ros::MissionPointGoal goal;
	goal.mission_traj.trajectory.push_back(state3D);
	goal.isOverwrite = isOverwrite;
	mpoint_client.sendGoal(goal,
			boost::bind(&Planner::mpoint_result_callback, this, _1,_2),
			boost::bind(&Planner::mpoint_active_callback,this),
			boost::bind(&Planner::mpoint_feedback_callback,this,_1));
	//reset
	mpointCompleted = false;
	mp_progressPercentage = 0;
}

void Planner::sendMPoint(const c2_ros_msgs::Trajectory& traj, bool isOverwrite)
{
	c2_ros::MissionPointGoal goal;
	goal.isOverwrite = isOverwrite;
	c2_ros_msgs::Trajectory::_trajectory_type::const_iterator it;
	for (it=traj.trajectory.begin(); it != traj.trajectory.end();it++){
		goal.mission_traj.trajectory.push_back(*it);
	}

	mpoint_client.sendGoal(goal,
			boost::bind(&Planner::mpoint_result_callback, this, _1,_2),
			boost::bind(&Planner::mpoint_active_callback,this),
			boost::bind(&Planner::mpoint_feedback_callback,this,_1));

	//reset
	mpointCompleted = false;
	mp_progressPercentage = 0;
}

void Planner::setMLCompleted(bool isSucceeded)
{
	c2_ros::MissionLegResult result;
	if(isSucceeded)
	{
		ROS_INFO("[%s]'s goal succeeded",agentName.c_str());
		result.isSucceeded = true;
		as_.setSucceeded(result);

	}
	else
	{
		ROS_INFO("[%s]'s goal failed",agentName.c_str());
		result.isSucceeded = false;
		as_.setAborted(result);
	}


	//stop tick
	toTick = false;
}

bool Planner::isMPointCompleted(){return mpointCompleted;}

c2_ros_msgs::MissionLeg Planner::getMissionLeg(){return m_leg;}

int Planner::getMPProgress(){return mp_progressPercentage;}

void Planner::goal_callback()
{
	ROS_INFO("[%s]:Mission Leg received",agentName.c_str());

	//reset all the attributes
	mpointCompleted = true;
	toTick = true;
	mp_progressPercentage = 0;

	//accept the goal
	m_leg = as_.acceptNewGoal()->m_leg;

	//allow the inheritant class to do some preparation
	onGoalReceived();
}

void Planner::preempt_callback()
{
	//ensure the pilot is preempted too!
	mpoint_client.cancelAllGoals();

	//call the inherited class' onStop()
	onStop();

	//reset the attributes
	toTick = false;
	m_leg = c2_ros_msgs::MissionLeg();

	//reply the Captain
	as_.setPreempted();

}

void Planner::mpoint_result_callback(const actionlib::SimpleClientGoalState& state,
		const c2_ros::MissionPointResultConstPtr& result)
{
	if(state == actionlib::SimpleClientGoalState::ABORTED)
	{
		ROS_INFO("[%s]:Mission point failure reported by pilot",agentName.c_str());
		onMPointFailure();
	}
	else if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("[%s]:Mission point Succeeded reported by pilot",agentName.c_str());
		mp_progressPercentage = 100;
		mpointCompleted = true;
	}else if(state == actionlib::SimpleClientGoalState::PREEMPTED)
		ROS_INFO("[%s]:Mission point preempted reported by pilot",agentName.c_str());
}

void Planner::mpoint_active_callback()
{
	mpointCompleted = false;
}

void Planner::mpoint_feedback_callback(const c2_ros::MissionPointFeedbackConstPtr& feedback)
{
	mp_progressPercentage = feedback->progress;
}

void Planner::sendMLProgress(int percentage_completed)
{
	c2_ros::MissionLegFeedback feedback;
	feedback.progress = percentage_completed;
	as_.publishFeedback(feedback);
}

void Planner::registerCapableBHV(c2_ros_msgs::C2_BHV::_bhv_type bhv)
{
	capable_bhv.push_back(bhv);
}
