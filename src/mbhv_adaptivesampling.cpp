#include <ros/ros.h>
#include <c2_ros/c2_agent.h>
#include <c2_ros/planner.h>


namespace C2 {

class MBHV_AdaptiveSampling: public Planner
{
private:

public:
	MBHV_AdaptiveSampling(std::string name, int loopRate, ros::NodeHandle nh):Planner(name,loopRate,nh){

	}

	~MBHV_AdaptiveSampling(){

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
	ros::init(argc, argv, C2::C2Agent(C2::C2Agent::MBHV_ADAPTIVESAMPLER).toString());
	ros::NodeHandle nh;
	C2::MBHV_AdaptiveSampling c(C2::C2Agent(C2::C2Agent::MBHV_ADAPTIVESAMPLER).toString(),1,nh);
	c.spin();

	return 0;
}