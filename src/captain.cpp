#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>

#include "c2_ros/mission.h"
#include "c2_ros/C2_BHV.h"
#include "c2_ros/BHVProposer.h"
#include "c2_ros/C2_CMD.h"
#include "c2_ros/c2_state.h"
#include "c2_ros/MissionLeg.h"
#include <c2_ros/c2_agent.h>
#include <c2_ros/planner_map.h>
#include <asco_utils/utils.h>

#include <actionlib/client/simple_action_client.h>
#include <c2_ros/MissionLegAction.h>
#include <geodesy/utm.h>

c2_ros::C2_BHV bhv;

using C2::C2_STATE;

#define DEFAULT_SPEED 1
#define DEFAULT_MPT_RADIUS 5

namespace C2 {

class Captain
{

#define mPath "/Missions/mission.txt"

protected:
	ros::NodeHandle nh_;

	ros::ServiceServer srv_cmd;
	ros::Subscriber odom_est_sub;
	ros::Subscriber bhv_propose_sub;
	ros::Publisher bhv_request_pub;

	C2::Planner_Map *pm;

private:
	std::string agentName;
	std::string activePlanner;
	C2::Mission curMission;
	geometry_msgs::Pose2D curPos;
	geometry_msgs::Pose2D m_startPos;
	geometry_msgs::Pose2D home_Pos;
	const c2_ros::MissionLeg* curMissionLeg;
	C2_STATE myState;
	c2_ros::C2_CMD receivedCmd;
	int m_leg_cnt;
	bool isCurMLCompleted;
	double default_speed;
	double default_mpt_radius;

	bool loadMissionFile(){

		//clear the previous mission
		curMission.clear();

		//get the home dir
		struct passwd *pw = getpwuid(getuid());
		std::string homedir (pw->pw_dir);

		//get mission path from param server
		std::string m_filename;
		if (!nh_.getParam("/captain_node/captain_params/mission_file_location",m_filename)) m_filename = homedir + mPath;


		//construct the path and check if the dir exist
		std::ifstream infile(m_filename.c_str());
		if (!infile.good()){
			ROS_WARN("Mission file not exist:[%s]",m_filename.c_str());
			return false;
		}

		//file exist, proceeed
		//the processing is according to APM 2.0 GUI (QGC WPL 110)
		std::string line;
		double lat,lon;
		while (std::getline(infile, line))
		{
			std::istringstream iss(line);
			std::string token;
			int cnt = 0;
			while (std::getline(iss, token, '\t')) cnt++;
			if (cnt < 11) continue; // must be 12 columns

			cnt = 1;
			c2_ros::MissionLeg ml;
			std::istringstream is(line);
			while (std::getline(is, token, '\t')){
				if(cnt == 4){//command type
					//convert to integer and assign it to the corresponding bhv constant
					int bhv_num = std::stoi(token);
					if(bhv_num == 16)
						ml.m_bhv.bhv = bhv.WAY_POINT;
					//TODO fill in the rest !
				}
				else if (cnt == 6){ // mission_pt_radius
					ml.m_pt_radius = std::stod(token);
					if(ml.m_pt_radius <= 0.5) ml.m_pt_radius = default_mpt_radius;
				}
				else if (cnt == 8){ //heading
					ml.m_pt.theta = std::stod(token);
				}
				else if (cnt == 9){ // lat
					lat = std::stod(token);
				}
				else if (cnt == 10){ //lon
					lon = std::stod(token);
				}
				else if (cnt == 11){ //altitude or depth
					ml.altdepth = std::stod(token);
				}
				cnt++;
			}

			//convert lat-lon to UTM
			geodesy::UTMPoint utmp;
			geodesy::fromMsg(geodesy::toMsg(lat,lon),utmp);
			ml.m_pt.x = utmp.easting;
			ml.m_pt.y = utmp.northing;


			//TODO APM mission planner not allow desired speed, hack away !!!
			ml.desired_speed = default_speed;

			//add the mission leg into the mission
			curMission.add(ml);
		}

		if (curMission.getMissionLegCount() < 1){
			ROS_WARN("no mission leg in the mission file");
			return false;
		}

		ROS_INFO("Mission file [%s] has %d legs",m_filename.c_str(),curMission.getMissionLegCount());
		return true;
	}

	bool setMyState(C2_STATE state){
		if(onStateEntry(myState,state)){
			ROS_INFO("[%s]: [%s]->[%s]",agentName.c_str(),C2::C2_StateName[(int)myState],C2::C2_StateName[(int)state]);
			myState = state;
			//broadcast the captain's state

			return true;
		}else return false;

		return true;
	}
	bool onStateEntry(C2_STATE oldState, C2_STATE newState){
		if(oldState == newState){
			ROS_WARN("captain already in [%s] state",C2::C2_StateName[(int)oldState]);
			return false;
		}
		//TODO onEntry logic checking and possibly house keeping

		if(oldState == C2_STATE::STANDBY){

			//STANDBY -> RUN
			if(newState == C2_STATE::RUN){
				//load mission file
				if(!loadMissionFile()){
					setMyState(C2_STATE::ERROR);
					return false;
				}else{
					//reset m_leg_cnt
					m_leg_cnt = 0;
					isCurMLCompleted = true;

					//record the mission start position
					m_startPos = curPos;

					// reset active planner's name
					activePlanner.clear();

				}
			}

		}else if(oldState == C2_STATE::RUN){

		}else if(oldState == C2_STATE::ABORT){

		}else if(oldState == C2_STATE::ERROR){
			ROS_WARN("captain shouldn't be in error state, rectify the problem before continuing");
			return false;
		}


		return true;
	}

	void requestForProposal(c2_ros::C2_BHV bhv)
	{
		c2_ros::C2_BHV b;
		b.bhv = bhv.bhv;
		bhv_request_pub.publish(b);
	}

	void run(){
		if(isCurMLCompleted){
			if(m_leg_cnt < curMission.getMissionLegCount()){
				curMissionLeg = curMission.get(m_leg_cnt);
				if(curMissionLeg != nullptr)
				{
					if(activePlanner.empty())
					{
						requestForProposal(curMissionLeg->m_bhv);
						return;
					}

					m_leg_cnt++;

					//debug: print out all the mission legs
					ROS_INFO("bhv:[%d],x:[%f],y:[%f],heading:[%f],mpoint_rad:[%f]",curMissionLeg->m_bhv.bhv,curMissionLeg->m_pt.x,curMissionLeg->m_pt.y,curMissionLeg->m_pt.theta,curMissionLeg->m_pt_radius );
					sendGoal(*curMissionLeg);
					isCurMLCompleted = false;
				}
				else
				{
					ROS_WARN("current mission return null ! ");
					setMyState(C2_STATE::ERROR);
				}

			}else if(m_leg_cnt == curMission.getMissionLegCount()){
				ROS_INFO("Mission Completed");
				setMyState(C2_STATE::STANDBY);
			}else{
				ROS_WARN("Error in mission leg count");
				setMyState(C2_STATE::ERROR);
			}
		}
	}
public:
	bool request_cmd_callback(c2_ros::C2_CMD::Request& request,
			c2_ros::C2_CMD::Response& response)
	{
		receivedCmd.request.command = request.command;
		bool result = false;
		if(request.command == c2_ros::C2_CMD::Request::RUN_MISSION)
		{
			result = setMyState(C2_STATE::RUN);
			response.result = result;
			return result;
		}else if (request.command == c2_ros::C2_CMD::Request::ABORT ||
				request.command == c2_ros::C2_CMD::Request::ABORT_TO_HOME ||
				request.command == c2_ros::C2_CMD::Request::ABORT_TO_START_POS)
		{
			//cancel the activeplanner's activities
			if(!activePlanner.empty())
				pm->getClient(activePlanner)->cancelAllGoals();

			//reassign the activePlanner to aborter
			activePlanner = C2::C2Agent(C2::C2Agent::MBHV_ABORTER).toString();
			result = setMyState(C2_STATE::ABORT);
			response.result = result;
			return result;
		}
		response.result = result;
		return result;
	}

	void goal_result_callback(const actionlib::SimpleClientGoalState& state,
			const c2_ros::MissionLegResultConstPtr& result)
	{
		if(state == actionlib::SimpleClientGoalState::ABORTED)
		{
			ROS_INFO("Mission leg failure reported by planner, abort the mission...");
			setMyState(C2_STATE::ABORT);
		}
		else if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Mission leg Succeeded reported by planner, proceed to the next mission leg...");
			isCurMLCompleted = true;
			activePlanner.clear();
		}else if(state == actionlib::SimpleClientGoalState::PREEMPTED)
			ROS_INFO("Mission leg preempted reported by planner");
	}

	void goal_active_callback()
	{
		ROS_INFO("MissionLeg number [%d] activated",m_leg_cnt);
	}

	void goal_feedback_callback(const c2_ros::MissionLegFeedbackConstPtr& feedback)
	{

	}

	void abort_result_callback(const actionlib::SimpleClientGoalState& state,
			const c2_ros::MissionLegResultConstPtr& result)
	{

	}

	void abort_active_callback()
	{

	}

	bool contactServers(){

		//contact all the MBHV servers
		for (int i=0 ; i < pm->count(); i++)
		{
			if(!pm->getClient(i)->waitForServer(ros::Duration(10,0)))
			{
				ROS_WARN("Timeout contacting [%s] server",pm->getClientName(i).c_str());
				return false;
			}
		}
		return true;
	}

	void sendGoal(const c2_ros::MissionLeg& ml){

		if(activePlanner.empty())
		{
			ROS_WARN("No active planner exist ! ");
			return;
		}

		c2_ros::MissionLegGoal goal;
		goal.m_leg = c2_ros::MissionLeg(ml);

		if(ml.m_bhv.bhv == bhv.ABORT)
		{
			pm->getClient(activePlanner)->sendGoal(goal,
					boost::bind(&Captain::abort_result_callback, this, _1,_2),
					boost::bind(&Captain::abort_active_callback,this),
					actionlib::SimpleActionClient<c2_ros::MissionLegAction>::SimpleFeedbackCallback());
		}
		else
		{
			pm->getClient(activePlanner)->sendGoal(goal,
					boost::bind(&Captain::goal_result_callback, this, _1,_2),
					boost::bind(&Captain::goal_active_callback,this),
					boost::bind(&Captain::goal_feedback_callback,this,_1));
		}
	}

	void vehicleOdom(const nav_msgs::Odometry::ConstPtr& odom_pos_est)
	{
		curPos.x = odom_pos_est->pose.pose.position.x;
		curPos.y = odom_pos_est->pose.pose.position.y;
		curPos.theta = tf::getYaw(odom_pos_est->pose.pose.orientation);
	}

	void bhv_propose_callback(const c2_ros::BHVProposer::ConstPtr& bhv_proposer)
	{
		//TODO future exansion to allow more than one active planner ?
		//alway take the last one that reply, need to fix this when there is a need in the future
		//market-based ??
		activePlanner = std::string(bhv_proposer->name);
	}

	Captain(std::string name, ros::NodeHandle nh):
		agentName(name),
		myState(C2_STATE::INIT),
		nh_(nh),
		m_leg_cnt(0),
		curMissionLeg(nullptr),
		isCurMLCompleted(true){

		//retrieve the default value for speed and radius
		if (!nh_.getParam("/c2_params/default_desired_speed",default_speed)) default_speed = DEFAULT_SPEED;
		if (!nh_.getParam("/c2_params/default_m_pt_radius",default_mpt_radius)) default_mpt_radius = DEFAULT_MPT_RADIUS;
		if (!nh_.getParam("/c2_params/homeX",home_Pos.x)) home_Pos.x = 0;
		if (!nh_.getParam("/c2_params/homeY",home_Pos.y)) home_Pos.y = 0;
		double lat,lon;
		if (nh_.getParam("/c2_params/homeX_lat",lat) &&
				nh_.getParam("/c2_params/homeY_lon",lon))
		{
			geodesy::UTMPoint utmp;
			geodesy::fromMsg(geodesy::toMsg(lat,lon),utmp);
			home_Pos.x = utmp.easting;
			home_Pos.y = utmp.northing;
		}

		//advertise service
		srv_cmd = nh_.advertiseService(C2::C2Agent(C2::C2Agent::CAPTAIN).toString(),&C2::Captain::request_cmd_callback,this);


		//subscribe to vehicle state
		std::string odm_name;
		if (!nh_.getParam("/c2_params/odometry_topic_name",odm_name)) odm_name = "/odometry/filtered";
		odom_est_sub = nh_.subscribe(odm_name,1, &C2::Captain::vehicleOdom,this);

		//initialize and declare all the MBHV planners.
		pm = new Planner_Map();

		//make sure all the server exist, or else report error
		if(!contactServers())
			setMyState(C2_STATE::ERROR);

		//pub and sub to behavior request and proposal
		bhv_propose_sub = nh_.subscribe("/captain/bhv_propose",100, &Captain::bhv_propose_callback,this);
		bhv_request_pub = nh_.advertise<c2_ros::C2_BHV>("/captain/bhv_request",100);
	}

	~Captain(){

	}

	void tick(){
		switch(myState){
		case C2_STATE::INIT:
			setMyState(C2_STATE::STANDBY);
			break;
		case C2_STATE::RUN:
			run();
			break;
		case C2_STATE::STANDBY:
			break;
		case C2_STATE::ABORT:
			break;
		case C2_STATE::ERROR:
			ROS_WARN("Captain in error state ! ");
			break;
		default :
			ROS_WARN("Captain in default state??!!");
		}
	}


};

}

int main (int argc, char ** argv)
{	
	ros::init(argc, argv, C2::C2Agent(C2::C2Agent::CAPTAIN).toString());
	ros::NodeHandle n;
	ros::Rate loop_rate(1); //default to 1Hz

	C2::Captain c(C2::C2Agent(C2::C2Agent::CAPTAIN).toString(),n);

	//iterate
	while (ros::ok())
	{
		c.tick();
		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;
}
