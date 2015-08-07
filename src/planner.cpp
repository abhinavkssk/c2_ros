//base class for all the mission behaviors

#include <c2_ros/planner.h>

using C2::Planner;

Planner::Planner(std::string name, int loopRate):
						as_(nh_,name,false), // call Planner::spin() manually
						action_name_(name),
						mpoint_client("Pilot",false), //call Planner::spin() manually
						loop_rate(loopRate),
						mpointCompleted(false),
						toTick(false),
						progressPercentage(0)
{

	//register the goal and feeback callbacks
	as_.registerGoalCallback(boost::bind(&Planner::goal_callback, this));
	as_.registerPreemptCallback(boost::bind(&Planner::preempt_callback, this));


	//connect to pilot server
	if(!mpoint_client.waitForServer(ros::Duration(5,0)))
	{
		ROS_WARN("Pilot server can not be connected, in [%s]",action_name_.c_str());
		c2_ros::MissionLegResult result;
		result.isSucceeded = false;
		as_.setAborted(result);
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

void Planner::sendMPoint(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
	c2_ros::MissionPointGoal goal;
	goal.mission_pt[0].header = pose->header;
	goal.mission_pt[0].pose = pose->pose;

	mpoint_client.sendGoal(goal,
			boost::bind(&Planner::mpoint_result_callback, this, _1,_2),
			boost::bind(&Planner::mpoint_active_callback,this),
			boost::bind(&Planner::mpoint_feedback_callback,this,_1));

	//reset
	mpointCompleted = false;
	progressPercentage = 0;
}

void Planner::sendMPoint(std::vector<geometry_msgs::PoseStamped> poses)
{
	c2_ros::MissionPointGoal goal;
	std::vector<geometry_msgs::PoseStamped>::iterator it;
	int i=0;
	for (it=poses.begin(); it != poses.end();it++){
		goal.mission_pt[i] = *it;
		i++;
	}

	mpoint_client.sendGoal(goal,
			boost::bind(&Planner::mpoint_result_callback, this, _1,_2),
			boost::bind(&Planner::mpoint_active_callback,this),
			boost::bind(&Planner::mpoint_feedback_callback,this,_1));

	//reset
	mpointCompleted = false;
	progressPercentage = 0;
}

void Planner::setMLCompleted(bool isSucceeded)
{
	c2_ros::MissionLegResult result;
	if(isSucceeded)
	{
		ROS_INFO("[%s]'s goal succeeded",action_name_.c_str());
		result.isSucceeded = true;
		as_.setSucceeded(result);

	}
	else
	{
		ROS_INFO("[%s]'s goal failed",action_name_.c_str());
		result.isSucceeded = false;
		as_.setAborted(result);
	}


	//stop tick
	toTick = false;
}

bool Planner::isMPointCompleted(){return mpointCompleted;}

c2_ros::MissionLeg Planner::getMissionLeg(){return m_leg;}

int Planner::getMPProgress(){return progressPercentage;}

void Planner::goal_callback()
{
	//reset all the attributes
	mpointCompleted = true;
	toTick = true;
	progressPercentage = 0;

	//accept the goal
	m_leg = as_.acceptNewGoal()->m_leg;

	//allow the inheritant class to do some preparation
	init();
}

void Planner::preempt_callback()
{
	//ensure the pilot is preempted too!
	mpoint_client.cancelAllGoals();

	//call the inherited class' onStop()
	onStop();

	//reset the attributes
	toTick = false;
	m_leg = c2_ros::MissionLeg();

}

void Planner::mpoint_result_callback(const actionlib::SimpleClientGoalState& state,
		const c2_ros::MissionPointResultConstPtr& result)
{
	if(state == actionlib::SimpleClientGoalState::ABORTED)
	{
		ROS_INFO("Mission point failure reported by pilot");
		onMPointFailure();
	}
	else if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Mission point Succeeded reported by pilot");
		progressPercentage = 100;
		mpointCompleted = true;
	}else if(state == actionlib::SimpleClientGoalState::PREEMPTED)
		ROS_INFO("Mission point preempted reported by pilot");
}

void Planner::mpoint_active_callback()
{
	mpointCompleted = false;
}

void Planner::mpoint_feedback_callback(const c2_ros::MissionPointFeedbackConstPtr& feedback)
{
	progressPercentage = feedback->progress;
}
