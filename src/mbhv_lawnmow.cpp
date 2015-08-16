#include <ros/ros.h>
#include <c2_ros/c2_agent.h>
#include <c2_ros/planner.h>


namespace C2 {

class MBHV_LawnMow: public Planner
{
private:

public:
	MBHV_LawnMow(std::string name, int loopRate, ros::NodeHandle nh):Planner(name,loopRate,nh){
		registerCapableBHV(c2_ros::C2_BHV::LAWNMOW);
	}

	~MBHV_LawnMow(){

	}

	void onGoalReceived(){

	}

	void tick(){

	}

	void onStop(){

	}

	void onMPointFailure(){

	}

};

}

int main (int argc, char ** argv)
{
	ros::init(argc, argv, C2::C2Agent(C2::C2Agent::MBHV_LAWNMOWER).toString());
	ros::NodeHandle nh;
	C2::MBHV_LawnMow c(C2::C2Agent(C2::C2Agent::MBHV_LAWNMOWER).toString(),1,nh);
	c.spin();

	return 0;
}
