#ifndef MISSION_H_
#define MISSION_H_

#include "c2_ros_msgs/MissionLeg.h"

namespace C2 {

class Mission {

private:
	std::vector<c2_ros_msgs::MissionLeg> m_legs;

public:
	Mission();
	~Mission(void);
	void add(c2_ros_msgs::MissionLeg ml);
	int getMissionLegCount();
	void clear();
	const c2_ros_msgs::MissionLeg *get(int num);
};

}

#endif
