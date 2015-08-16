#include <ros/ros.h>
#include <c2_ros/c2_agent.h>
#include <c2_ros/planner.h>


namespace C2 {

class MBHV_Loiter: public Planner
{
private:

public:
	MBHV_Loiter(std::string name, int loopRate, ros::NodeHandle nh):Planner(name,loopRate,nh){
		registerCapableBHV(c2_ros::C2_BHV::LOITER);
	}

	~MBHV_Loiter(){

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
	ros::init(argc, argv, C2::C2Agent(C2::C2Agent::MBHV_LOITER).toString());
	ros::NodeHandle nh;
	C2::MBHV_Loiter c(C2::C2Agent(C2::C2Agent::MBHV_LOITER).toString(),1,nh);
	c.spin();

	return 0;
}
