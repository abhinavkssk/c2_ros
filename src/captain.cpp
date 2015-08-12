#include <ros/ros.h>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <fstream>
#include <sstream>
#include <string>

#include "c2_ros/mission.h"
#include "c2_ros/C2_BHV.h"
#include "c2_ros/C2_CMD.h"
#include "c2_ros/c2_state.h"
#include "c2_ros/MissionLeg.h"
#include <c2_ros/c2_agent.h>

#include <actionlib/client/simple_action_client.h>
#include <c2_ros/MissionLegAction.h>
#include <geodesy/utm.h>

c2_ros::C2_BHV bhv;

using C2::C2_STATE;

typedef actionlib::SimpleActionClient<c2_ros::MissionLegAction> ML_Client;

class Captain
{

#define mPath "/Missions/mission.txt"

protected:
	ros::NodeHandle nh_;

	ros::ServiceServer srv_cmd;

	ML_Client c_waypoint, c_abort, c_lawnmow, c_adaptivesampling;

private:
	C2::Mission curMission;
	const c2_ros::MissionLeg* curMissionLeg;
	C2_STATE myState;
	c2_ros::C2_CMD receivedCmd;
	int m_leg_cnt;
	bool isCurMLCompleted;

	bool loadMissionFile(){

		//clear the previous mission
		curMission.clear();

		//get the home dir
		struct passwd *pw = getpwuid(getuid());
		std::string homedir (pw->pw_dir);

		//construct the path and check if the dir exist
		std::string filePath = homedir + "/Missions/mission.txt";
		std::ifstream infile(filePath.c_str());
		if (!infile.good()){
			ROS_WARN("Mission file not exist:[%s]",filePath.c_str());
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

			//add the mission leg into the mission
			curMission.add(ml);
		}

		if (curMission.getMissionLegCount() < 1){
			ROS_WARN("no mission leg in the mission file");
			return false;
		}

		ROS_INFO("Mission file [%s] has %d legs",filePath.c_str(),curMission.getMissionLegCount());
		return true;
	}

	bool setMyState(C2_STATE state){
		if(onStateEntry(myState,state)){
			ROS_INFO("state transition [%s]->[%s]",C2::C2_StateName[(int)myState],C2::C2_StateName[(int)state]);
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

	void run(){
		if(isCurMLCompleted){
			if(m_leg_cnt < curMission.getMissionLegCount()){
				curMissionLeg = curMission.get(m_leg_cnt);
				if(curMissionLeg != nullptr)
				{
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
		if(request.command == c2_ros::C2_CMD::Request::RUN_MISSION){
			result = setMyState(C2_STATE::RUN);
			response.result = result;
			return result;
		}else if (request.command == c2_ros::C2_CMD::Request::ABORT ||
				request.command == c2_ros::C2_CMD::Request::ABORT_TO_HOME ||
				request.command == c2_ros::C2_CMD::Request::ABORT_TO_START_POS){
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

	bool contactServers(){
		if(!c_waypoint.waitForServer(ros::Duration(10,0))){
			ROS_WARN("Timeout contacting c_waypoint server");
			return false;
		}else
			ROS_INFO("c_waypoint server contacted, continue ...");
		if(!c_abort.waitForServer(ros::Duration(10,0))){
			ROS_WARN("Timeout contacting c_abort server");
			return false;
		}else
			ROS_INFO("c_abort server contacted, continue ...");
		if(!c_lawnmow.waitForServer(ros::Duration(10,0))){
			ROS_WARN("Timeout contacting c_lawnmow server");
			return false;
		}else
			ROS_INFO("c_lawnmow server contacted, continue ...");
		if(!c_adaptivesampling.waitForServer(ros::Duration(10,0))){
			ROS_WARN("Timeout contacting c_adaptivesampling server");
			return false;
		}else
			ROS_INFO("c_adaptivesampling server contacted, continue ...");

		return true;
	}

	void sendGoal(const c2_ros::MissionLeg& ml){
		c2_ros::MissionLegGoal goal;
		goal.m_leg = c2_ros::MissionLeg(ml);

		if(ml.m_bhv.bhv == bhv.WAY_POINT){
			c_waypoint.sendGoal(goal,
					boost::bind(&Captain::goal_result_callback, this, _1,_2),
					boost::bind(&Captain::goal_active_callback,this),
					boost::bind(&Captain::goal_feedback_callback,this,_1));
		}else if (ml.m_bhv.bhv == bhv.LAWNMOW){
			c_lawnmow.sendGoal(goal,
					boost::bind(&Captain::goal_result_callback, this, _1,_2),
					boost::bind(&Captain::goal_active_callback,this),
					boost::bind(&Captain::goal_feedback_callback,this,_1));
		}else if (ml.m_bhv.bhv == bhv.ADAPTIVE_SAMPLING){
			c_adaptivesampling.sendGoal(goal,
					boost::bind(&Captain::goal_result_callback, this, _1,_2),
					boost::bind(&Captain::goal_active_callback,this),
					boost::bind(&Captain::goal_feedback_callback,this,_1));
		}
	}

	Captain(std::string name, ros::NodeHandle nh):
		myState(C2_STATE::INIT),
		nh_(nh),
		m_leg_cnt(0),
		curMissionLeg(nullptr),
		isCurMLCompleted(true),
		//declare the clients
		c_waypoint(C2::C2Agent(C2::C2Agent::MBHV_WAYPOINTER).toString(),true),
		c_abort(C2::C2Agent(C2::C2Agent::MBHV_ABORTER).toString(),true),
		c_lawnmow(C2::C2Agent(C2::C2Agent::MBHV_LAWNMOWER).toString(),true),
		c_adaptivesampling(C2::C2Agent(C2::C2Agent::MBHV_ADAPTIVESAMPLER).toString(),true) {

		//advertise service
		srv_cmd = nh_.advertiseService(C2::C2Agent(C2::C2Agent::CAPTAIN).toString(),&Captain::request_cmd_callback,this);

		//make sure all the server exist, or else report error
		if(!contactServers())
			setMyState(C2_STATE::ERROR);


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

int main (int argc, char ** argv)
{	
	ros::init(argc, argv, C2::C2Agent(C2::C2Agent::CAPTAIN).toString());
	ros::NodeHandle n;
	ros::Rate loop_rate(1); //default to 1Hz

	Captain c(C2::C2Agent(C2::C2Agent::CAPTAIN).toString(),n);

	//iterate
	while (ros::ok())
	{
		c.tick();
		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;
}
