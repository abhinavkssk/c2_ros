#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <c2_ros/pilot.h>
#include <c2_ros/c2_agent.h>
#include <c2_ros/ActuatorControl.h>
#include <c2_ros/Trajectory.h>
#include <c2_ros/State3D.h>
#include <asco_utils/utils.h>
#include <vector>
#include <math.h>

namespace C2{

class Pilot_Simulated: public Pilot
{
private:
	ros::Subscriber odom_est_sub;
	ros::Publisher ac_pub;
	c2_ros::Trajectory poseToRun;
	c2_ros::State3D curMP;
	geometry_msgs::Pose2D curPos;


	float sideDist = 20;
	float secAngle = 10;

	int poseCnt;
	bool isCompleted;
	bool isReinitialized;
	bool isCurMPReached;
	double curBearing,bearingSP, speedSP;

public:
	void odom_x_est(const nav_msgs::Odometry::ConstPtr& odom_pos_est)
	{
		curPos.x = odom_pos_est->pose.pose.position.x;
		curPos.y = odom_pos_est->pose.pose.position.y;
		curPos.theta = tf::getYaw(odom_pos_est->pose.pose.orientation);
		curBearing = asco::Utils::yaw2bearing(curPos.theta);
	}

	Pilot_Simulated(int loopRate, ros::NodeHandle nh):
		Pilot(loopRate,nh),
		poseCnt(0),
		isCompleted(true),
		isReinitialized(true),
		isCurMPReached(true),
		bearingSP(0),
		speedSP(0),
		curBearing(0)
	{
		//subscribe to vehicle state
		std::string odm_name;
		if (!nh_.getParam("/c2_params/odometry_topic_name",odm_name)) odm_name = "/odometry/filtered";
		odom_est_sub = nh_.subscribe(odm_name,1, &C2::Pilot_Simulated::odom_x_est,this);

		ac_pub = nh_.advertise<c2_ros::ActuatorControl>("/c2_ros/actuator_control",1);
	}

	~Pilot_Simulated()
	{

	}



	void tick()
	{
		if(!isCompleted)
		{
			if(isReinitialized)
			{
				ROS_INFO("reinitialized the pilot");
				isReinitialized = false;
				isCurMPReached = true;
				poseCnt = 0;
			}
			if(isCurMPReached){
				if(poseCnt < poseToRun.trajectory.size())
				{
					curMP = poseToRun.trajectory.at(poseCnt);
					ROS_INFO("Navigating to x=%f, y=%f",curMP.pose.position.x,curMP.pose.position.y);

					//check vehicle's side angles
					isCurMPReached = checkDistAngle();
					if(isCurMPReached) return;

					poseCnt++;
				}
				else
				{
					isCompleted = true;
					stopVehicle();
					setMPCompleted(true);
					return;
				}
			}
			if(navigateTo(curMP))
			{
				isCurMPReached = true;
			}
		}
	}

	void stopVehicle()
	{
		c2_ros::ActuatorControl ac;
		ac.desired_speed = 0;
		ac_pub.publish(ac);
	}

	bool navigateTo(c2_ros::State3D mp)
	{
		double dist = asco::Utils::getDist2D(curPos, mp.pose);
		if(dist > mp.m_pt_radius){

			//calculate bearing
			bearingSP = asco::Utils::calAngle(curPos,mp.pose);

			//record speed
			speedSP = mp.twist.linear.x;

			//determine if the wp is on the side of the AUV (make sure to change to AUV's body Frame).
			float ang1 = (curBearing-90)/180*M_PI;
			if(ang1<0) ang1+=2*M_PI;
			float ang2 = ang1+(secAngle/180*M_PI);
			if(ang2>2*M_PI) ang2-=2*M_PI;
			float ang3 = (curBearing-270)/180*M_PI;
			if(ang3<0) ang3+=2*M_PI;
			float ang4 = ang3+(secAngle/180*M_PI);
			if(ang4>2*M_PI) ang4-=2*M_PI;
			if(((bearingSP>=ang1&&bearingSP<=ang2)||(bearingSP>=ang3&&bearingSP<=ang4))&&dist<sideDist)
			{
				ROS_INFO("wayPoint on the side, proceed to next one");
				return true;
			}

			//send actuator control
			c2_ros::ActuatorControl ac;
			ac.desired_speed = speedSP;
			ac.desired_bearing = bearingSP;
			ac_pub.publish(ac);
			ROS_INFO("[%s]: desired_bearing = %f, curBearing = %f, dist=%f",agentName.c_str(),bearingSP,curBearing,dist);

			return false;
		}

		ROS_INFO("reached x=%f, y=%f",mp.pose.position.x,mp.pose.position.y);
		return true;
	}

	bool checkDistAngle()
	{
		float angle = asco::Utils::calAngle(curPos, curMP.pose);
		float dist = asco::Utils::getDist2D(curPos, curMP.pose);
		float bearingDiff = fabs(curBearing - angle);

		ROS_INFO("angleToNextPos=%f bearing=%f angleDiff=%f",angle,curBearing,bearingDiff);

		if(bearingDiff > 45 && bearingDiff < 315) {
			//advance at most two points
			int cnt = 0;
			while(dist < sideDist && cnt < 2){
				if(poseCnt < poseToRun.trajectory.size()) {
					curMP = poseToRun.trajectory.at(poseCnt);
					poseCnt++;
					cnt++;
					ROS_INFO("too close for comfort,waypoint incremented by 1");
				}else if (poseCnt == poseToRun.trajectory.size()){
					return true;
				}
			}
		}
		return false;
	}

	void onStop()
	{
		poseToRun.trajectory.clear();
		poseCnt = 0;
		isCompleted = true;
		isCurMPReached = true;
		stopVehicle();
	}

	void newMissionPointAvailable(c2_ros::Trajectory traj, bool isOverwrite)
	{
		ROS_INFO("Mission point received by [%s]",agentName.c_str());
		if(isCompleted)
		{
			poseToRun.trajectory = c2_ros::Trajectory::_trajectory_type(traj.trajectory);
			isCompleted = false;
			isCurMPReached = true;
			poseCnt = 0;
		}
		else
		{
			if(isOverwrite)
			{
				poseToRun.trajectory = c2_ros::Trajectory::_trajectory_type(traj.trajectory);
				isReinitialized = true;
				poseCnt = 0;
			}
			else
			{
				c2_ros::Trajectory::_trajectory_type::iterator it;
				for(it = traj.trajectory.begin(); it != traj.trajectory.end(); it++){
					poseToRun.trajectory.push_back(*it);
				}
			}
		}
	}
};
}

int main (int argc, char ** argv)
{
	ros::init(argc, argv, C2::C2Agent(C2::C2Agent::PILOT).toString());
	ros::NodeHandle nh;
	C2::Pilot_Simulated c(10,nh);
	c.spin();

	return 0;
}
