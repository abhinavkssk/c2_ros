#include <actionlib/client/simple_action_client.h>
#include <c2_ros/MissionLegAction.h>
#include <c2_ros/c2_agent.h>
#include <c2_ros/planner_map.h>
#include <asco_utils/utils.h>

using C2::Planner_Map;
using asco::Utils;

Planner_Map::Planner_Map()
{
	//register all the MBHV agents
	for (int i=0 ; i < C2::C2Agent::AGENT_COUNT; i++)
	{
		std::string name = C2::C2Agent(C2::C2Agent::AgentEnum(i)).toString();
		if (Utils::chomp(name,"_") == "MBHV"){
			boost::shared_ptr<ML_client> a;
			a.reset(new ML_client(C2::C2Agent(C2::C2Agent::AgentEnum(i)).toString(),true));
			agents.push_back(a);
			agent_names.push_back(C2::C2Agent(C2::C2Agent::AgentEnum(i)).toString());
			ROS_INFO("[Planner_Map]:[%s] found! ",C2::C2Agent(C2::C2Agent::AgentEnum(i)).toString().c_str());
		}
	}
}

int Planner_Map::count()
{
	return agents.size();
}

boost::shared_ptr<ML_client> Planner_Map::getClient(std::string agent_name)
{
	int cnt = 0;
	for (std::vector<std::string>::iterator it = agent_names.begin(); it != agent_names.end(); it++){
		if(*it != agent_name) {
			cnt++;
			//ROS_INFO("agent name=%s",(*it).c_str());
		}
		else return agents[cnt];
	}

	ROS_WARN("Agent not found in Planner_Map !");
	return nullptr;
}

boost::shared_ptr<ML_client> Planner_Map::getClient(int indx)
{
 return agents[indx];
}

std::string Planner_Map::getClientName(int indx)
{
	return agent_names[indx];
}
