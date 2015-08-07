#include <ros/ros.h>
#include <c2_ros/planner.h>


namespace C2 {

class MBHV_WayPoint: public Planner
{
private:

public:
	MBHV_WayPoint(std::string name, int loopRate):Planner(name,loopRate){

	}

	~MBHV_WayPoint(){

	}

	void init(){

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
	ros::init(argc, argv, "Waypoint");

	C2::MBHV_WayPoint c("Waypoint",1);
	c.spin();

	return 0;
}
