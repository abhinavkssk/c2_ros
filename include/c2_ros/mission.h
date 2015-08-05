#ifndef MISSION_H_
#define MISSION_H_

#include "c2_ros/MissionLeg.h"

namespace C2 {

class Mission {

private:
	std::vector<c2_ros::MissionLeg> m_legs;

public:
	Mission();
	~Mission(void);
	void add(c2_ros::MissionLeg ml);
	int getMissionLegCount();
	void clear();
	c2_ros::MissionLeg get(int num);
};

}

#endif
