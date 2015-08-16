#ifndef PLANNER_MAP_H_
#define PLANNER_MAP_H_

#include <actionlib/client/simple_action_client.h>
#include <c2_ros/MissionLegAction.h>
#include <c2_ros/c2_agent.h>

typedef actionlib::SimpleActionClient<c2_ros::MissionLegAction> ML_client;

namespace C2 {

class Planner_Map {

private:
	std::vector<std::string> agent_names;
	std::vector<boost::shared_ptr<ML_client> > agents;

public:
	Planner_Map();
	int count();
	boost::shared_ptr<ML_client> getClient(std::string agent_name);
	boost::shared_ptr<ML_client> getClient(int indx);
	std::string getClientName(int indx);
};


}


#endif
