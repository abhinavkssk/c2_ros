#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <c2_ros/pilot.h>
#include <c2_ros/c2_agent.h>
#include <c2_ros/ActuatorControl.h>
#include <vector>
#include <math.h>

namespace C2{

class Pilot_Simulated: public Pilot
{
private:
	ros::Subscriber odom_est_sub;
	ros::Publisher ac_pub;
	std::vector<c2_ros::MissionPoint> poseToRun;
	c2_ros::MissionPoint curMP;
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
		curBearing = yaw2bearing(curPos.theta);
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
				if(poseCnt < poseToRun.size())
				{
					curMP = poseToRun.at(poseCnt);
					ROS_INFO("Navigating to x=%f, y=%f",curMP.m_pt.x,curMP.m_pt.y);
					poseCnt++;

					//check vehicle's side angles
					isCurMPReached = checkDistAngle();
					if(isCurMPReached) return;
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

	bool navigateTo(c2_ros::MissionPoint mp)
	{
		double dist = getDist2D(curPos, mp.m_pt);
		if(dist > mp.m_pt_radius){

			//calculate bearing
			bearingSP = calAngle(curPos,mp.m_pt);

			//record speed
			speedSP = mp.desired_speed;

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

		ROS_INFO("reached x=%f, y=%f",mp.m_pt.x,mp.m_pt.y);
		return true;
	}

	bool checkDistAngle()
	{
		float angle = calAngle(curPos, curMP.m_pt);
		float dist = getDist2D(curPos, curMP.m_pt);
		float bearingDiff = fabs(curBearing - angle);

		ROS_INFO("angleToNextPos=%f bearing=%f angleDiff=%f",angle,curBearing,bearingDiff);

		if(bearingDiff > 45 && bearingDiff < 315) {
			//advance at most two points
			int cnt = 0;
			while(dist < sideDist && cnt < 2){
				if(poseCnt < poseToRun.size()) {
					curMP = poseToRun.at(poseCnt);
					poseCnt++;
					cnt++;
					ROS_INFO("too close for comfort,waypoint incremented by 1");
				}else if (poseCnt == poseToRun.size()){
					return true;
				}
			}
		}
		return false;
	}

	double yaw2bearing(double yaw)
	{
		double b = 90 - (yaw/M_PI)*180;
		if(b < 0) b += 360;
		if(b > 360) b -= 360;
		return b;
	}

	//angle between two points in deg
	double calAngle(geometry_msgs::Pose2D pos1, geometry_msgs::Pose2D pos2)
	{
		double dx = pos2.x-pos1.x;
		double dy = pos2.y-pos1.y;
		double angle = 0;

		if(dy==0.0&&dx==0.0)
			angle = 0.0;
		else
			angle = -(atan2(dy,dx)-(M_PI/2));

		if(angle<0) angle+=2*M_PI;
		return angle/M_PI*180;
	}

	double getDist2D(geometry_msgs::Pose2D pos1, geometry_msgs::Pose2D pos2)
	{
		return sqrt(pow((pos1.x - pos2.x),2)+pow((pos1.y - pos2.y),2));
	}

	void onStop()
	{
		poseToRun.clear();
		poseCnt = 0;
		isCompleted = true;
		isCurMPReached = true;
		stopVehicle();
	}

	void newMissionPointAvailable(std::vector<c2_ros::MissionPoint> poses, bool isOverwrite)
	{
		ROS_INFO("Mission point received by [%s]",agentName.c_str());
		if(isCompleted)
		{
			poseToRun = std::vector<c2_ros::MissionPoint>(poses);
			isCompleted = false;
			isCurMPReached = true;
			poseCnt = 0;
		}
		else
		{
			if(isOverwrite)
			{
				poseToRun = std::vector<c2_ros::MissionPoint>(poses);
				isReinitialized = true;
				poseCnt = 0;
			}
			else
			{
				std::vector<c2_ros::MissionPoint>::iterator it;
				for(it = poses.begin(); it != poses.end(); it++){
					poseToRun.push_back(*it);
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
