#include <ros/ros.h>
#include <c2_ros/c2_agent.h>
#include <c2_ros/planner.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <c2_ros/State3D.h>
#include <c2_ros/Trajectory.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

namespace C2 {

class MBHV_AdaptiveSampling: public Planner
{
private:
	c2_ros::MissionLeg ml;
	bool isBHVStopped;

	ros::Subscriber adp_path_sub;
	ros::Publisher adp_start_pub;

	//rviz
	ros::Publisher marker_pub;

public:
	MBHV_AdaptiveSampling(std::string name, int loopRate, ros::NodeHandle nh):
		Planner(name,loopRate,nh),
		isBHVStopped(false){
		registerCapableBHV(c2_ros::C2_BHV::ADAPTIVE_SAMPLING);

		adp_path_sub = nh_.subscribe("/adp_path",10, &C2::MBHV_AdaptiveSampling::ADPPath_callback,this);
		adp_start_pub = nh_.advertise<std_msgs::Bool>("/adp_start",100);

		marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);

	}

	~MBHV_AdaptiveSampling(){

	}

	void onGoalReceived(){
		ml = getMissionLeg();
		std_msgs::Bool isStart;
		isStart.data = true;
		adp_start_pub.publish(isStart);

		isBHVStopped = false;
	}

	void tick(){
		//can implement a timer to time the matlab node,
	}

	void onStop(){
		//stop the matlab adp node
		std_msgs::Bool isStart;
		isStart.data = false;
		adp_start_pub.publish(isStart);

		isBHVStopped = true;
	}

	void onMPointFailure(){
		//stop the matlab adp node
		std_msgs::Bool isStart;
		isStart.data = false;
		adp_start_pub.publish(isStart);

		//notify Captain
		setMLCompleted(false);

		isBHVStopped = true;
	}

	void ADPPath_callback(const nav_msgs::Path adpPath)
	{
		//this check is needed in case matlab node is misbehaving
		if(!isBHVStopped){
			//rviz
			//post to rviz
			visualization_msgs::Marker line_strip;
			line_strip.id = 1000;
			line_strip.type = visualization_msgs::Marker::LINE_STRIP;
			line_strip.scale.x = 1;
			// Line strip is blue
			line_strip.color.b = 1.0;
			line_strip.color.a = 1.0;




			//translate it into C2 framework's Trajectory
			c2_ros::Trajectory traj;
			nav_msgs::Path::_poses_type::const_iterator it;
			for (it=adpPath.poses.begin(); it != adpPath.poses.end();it++){
				c2_ros::State3D p;
				p.pose = it->pose;
				p.twist.linear.x = 2;
				p.m_pt_radius = 0.5;
				traj.trajectory.push_back(p);

				//rviz
				geometry_msgs::Point point;
				point.x = it->pose.position.x;
				point.y = it->pose.position.y;
				line_strip.points.push_back(point);
			}

			//overwrite the previous point
			sendMPoint(traj,true);

			//post to rviz
			marker_pub.publish(line_strip);
		}
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
