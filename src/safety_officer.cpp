#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include "c2_ros/c2_agent.h"
#include "c2_ros/c2_state.h"
#include "c2_ros/C2_CMD.h"
#include "c2_ros_msgs/C2_STATE.h"

using C2::C2_STATE;

namespace C2{

class SafetyOfficer
{
protected:
	ros::NodeHandle nh_;
	ros::Subscriber odom_est_sub;
	ros::Subscriber captain_state_sub;
	ros::ServiceClient captain_srv;
	geometry_msgs::Pose2D curPos;

private:
	std::string agentName;
	C2_STATE myState;
	C2_STATE captainState;
	bool activeGeoFence;
	int npol;
	float *xp, *yp;

	// check if point is inside AOI
	int inAoi(float x, float y) {
		if (npol == 0) return 1;
		int i, j, c = 0;
		for (i = 0, j = npol-1; i < npol; j = i++) {
			if ((((yp[i] <= y) && (y < yp[j])) ||
					((yp[j] <= y) && (y < yp[i]))) &&
					(x < (xp[j] - xp[i]) * (y - yp[i]) / (yp[j] - yp[i]) + xp[i]))
				c = !c;
		}
		return c;
	}

	// check if vehicle is headed into AOI
	int headedIntoAoi(float x, float y, float yaw) {
		if (npol == 0) return 1;
		float sx = 0, sy = 0;
		for (int i = 0; i < npol; i++) {
			sx += xp[i];
			sy += yp[i];
		}
		sx = sx/npol-x;
		sy = sy/npol-y;
		float b = atan2f(sy,sx);
		b -= yaw;
		while (b > M_PI) b -= 2*M_PI;
		while (b < -M_PI) b += 2*M_PI;
		return (fabsf(b) < M_PI/2);
	}

public:

	SafetyOfficer(std::string name, ros::NodeHandle nh):
		agentName(name),
		myState(C2_STATE::INIT),
		captainState(C2_STATE::INIT),
		activeGeoFence(false),
		nh_(nh){

		//subscribe to vehicle state
		std::string odm_name;
		if (!nh_.getParam("/global_params/odometry_topic_name",odm_name)) odm_name = "/odometry/filtered";
		odom_est_sub = nh_.subscribe(odm_name,1, &C2::SafetyOfficer::odom_est,this);

		captain_state_sub = nh_.subscribe("/captain/state",1, &C2::SafetyOfficer::captain_state,this);
		captain_srv = nh_.serviceClient<c2_ros::C2_CMD>(C2::C2Agent(C2::C2Agent::CAPTAIN).toString());

		//retrieve the geofence vertexes
		if (!nh_.getParam("/c2_params/opArea_vertices",npol))
		{
			//geo-fence must be defined, or SO shouldn't be on
			ROS_ERROR("[%s] requires that OpArea:Vertices to be defined ! existing....",agentName.c_str());
			exit(0);
		}
		else
		{
			if (npol >= 0) {
				xp = (float*)malloc(sizeof(float)*npol);
				yp = (float*)malloc(sizeof(float)*npol);
				for (int i = 0; i < npol; i++) {
					char t[32];
					sprintf(t,"/c2_params/opArea_vertex_%d",i+1);
					std::string s;
					nh_.getParam(t,s);
					if (!s.empty()) {
						sscanf(s.c_str(),"%f/%f",&xp[i],&yp[i]);
						ROS_INFO("[%s] GeoFence Vertex: %f %f",agentName.c_str(),xp[i],yp[i]);
					} else {
						ROS_WARN("%s not specified",t);
						xp[i] = 0;
						yp[i] = 0;
					}
				}
			}
		}

	}

	~SafetyOfficer(){

	}

	void captain_state(const c2_ros_msgs::C2_STATEConstPtr& c_state)
	{
		captainState = (C2_STATE)c_state->state;
		if(captainState == C2_STATE::RUN)
			activeGeoFence = true;
		ROS_INFO("[%s] received captain's state = %s",agentName.c_str(),C2::C2_StateName[(int)captainState]);
	}

	void odom_est(const nav_msgs::Odometry::ConstPtr& odom_pos_est)
	{
		curPos.x = odom_pos_est->pose.pose.position.x;
		curPos.y = odom_pos_est->pose.pose.position.y;
		curPos.theta = tf::getYaw(odom_pos_est->pose.pose.orientation);
	}

	void tick(){
		if(!inAoi(curPos.x,curPos.y) && !headedIntoAoi(curPos.x,curPos.y,curPos.theta) && activeGeoFence)
		{
			ROS_WARN("[%s] out of geo-fence! inform [%s] to abort mission",agentName.c_str(),C2::C2Agent(C2::C2Agent::CAPTAIN).toString().c_str());
			c2_ros::C2_CMD srv;
			srv.request.command = c2_ros::C2_CMD::Request::ABORT_TO_HOME;
			if (captain_srv.call(srv) && srv.response.result)
			{
				ROS_INFO("[%s] inform succeed!",agentName.c_str());
			}
			else
			{
				ROS_ERROR("[%s] Failed to inform [%s]",agentName.c_str(),C2::C2Agent(C2::C2Agent::CAPTAIN).toString().c_str());
			}

			activeGeoFence = false;
		}
	}

};
}


int main (int argc, char ** argv)
{
	ros::init(argc, argv, C2::C2Agent(C2::C2Agent::SAFETY_OFFICER).toString());
	ros::NodeHandle n;
	ros::Rate loop_rate(0.5); //default to 1Hz

	C2::SafetyOfficer so(C2::C2Agent(C2::C2Agent::SAFETY_OFFICER).toString(),n);

	//iterate
	while (ros::ok())
	{
		so.tick();
		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;
}
