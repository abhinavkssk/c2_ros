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

c2_ros::C2_BHV bhv;

using C2::C2_STATE;

class Captain
{

#define mPath "/Missions/mission.txt"

protected:
	ros::NodeHandle nh_;

	ros::ServiceServer srv_cmd;


private:
	C2::Mission curMission;
	C2_STATE myState;
	c2_ros::C2_CMD receivedCmd;

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
					ml.mission_pt_radius = std::stod(token);
				}
				else if (cnt == 8){ //heading
					ml.heading = std::stod(token);
				}
				else if (cnt == 9){ // lat
					ml.lat = std::stod(token);
				}
				else if (cnt == 10){ //lon
					ml.lon = std::stod(token);
				}
				cnt++;
			}

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

	Captain(std::string name, ros::NodeHandle nh): myState(C2_STATE::INIT),nh_(nh) {
		//advertise service
		srv_cmd = nh_.advertiseService("captain",&Captain::request_cmd_callback,this);

	}

	~Captain(){

	}

	void tick(){
		switch(myState){
		case C2_STATE::INIT:
			setMyState(C2_STATE::STANDBY);
			break;
		case C2_STATE::RUN:
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
	ros::init(argc, argv, "captain_node");
	ros::NodeHandle n;
	ros::Rate loop_rate(1); //default to 1Hz

	Captain c(ros::this_node::getName(),n);

	//iterate
	while (ros::ok())
	{
		c.tick();
		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;
}
