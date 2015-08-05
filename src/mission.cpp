#include "c2_ros/mission.h"

using C2::Mission;

Mission::Mission(){}
Mission::~Mission(void){}

void Mission::add(c2_ros::MissionLeg ml)
{
	m_legs.push_back(ml);
}

int Mission::getMissionLegCount()
{
	return m_legs.size();
}

void Mission::clear()
{
	m_legs.clear();
}

c2_ros::MissionLeg Mission::get(int num)
{
	if (num >=0 && num < m_legs.size())
		return m_legs[num];
	else
		return nullptr;
}

