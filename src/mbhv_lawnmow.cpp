#include <ros/ros.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <c2_ros/c2_agent.h>
#include <c2_ros/planner.h>
#include <asco_utils/utils.h>


namespace C2 {

class MBHV_LawnMow: public Planner
{
private:
	ros::Subscriber odom_est_sub;
	c2_ros_msgs::MissionLeg ml;
	geometry_msgs::Pose2D curPos;
	double curBearing;
	double xLength, yLength, moweWidth, moweBearing;
	bool isMLCompleted;

public:
	MBHV_LawnMow(std::string name, int loopRate, ros::NodeHandle nh):
		Planner(name,loopRate,nh),
		curBearing(0),
		isMLCompleted(false),
		xLength(20.0),
		yLength(20.0),
		moweWidth(10.0),
		moweBearing(0.0){
		registerCapableBHV(c2_ros_msgs::C2_BHV::LAWNMOW);

		//subscribe to vehicle state
		std::string odm_name;
		if (!nh_.getParam("/global_params/odometry_topic_name",odm_name)) odm_name = "/odometry/filtered";
		odom_est_sub = nh_.subscribe(odm_name,1, &C2::MBHV_LawnMow::odom_est,this);

		getParams();
	}

	~MBHV_LawnMow(){

	}

	void odom_est(const nav_msgs::Odometry::ConstPtr& odom_pos_est)
	{
		curPos.x = odom_pos_est->pose.pose.position.x;
		curPos.y = odom_pos_est->pose.pose.position.y;
		curPos.theta = tf::getYaw(odom_pos_est->pose.pose.orientation);
		curBearing = asco::Utils::yaw2bearing(curPos.theta);
	}

	void onGoalReceived(){
		isMLCompleted = false;
		ml = getMissionLeg();
		getParams();
	}

	void getParams(){
		//TODO: since the mission planning hasn't have a way to specify the parameters required for the LM planner, just read them from the parameter server for now.
		//obtain all the parameters for the LM planning.
		if (!nh_.getParam("/c2_params/lm_xlength",xLength))
		{
			ROS_WARN("Please Specify the lawnmowe parameter for [%s] in /c2_params",agentName.c_str());
		}
		nh_.getParam("/c2_params/lm_ylength",yLength);
		nh_.getParam("/c2_params/lm_mowewidth",moweWidth);
		nh_.getParam("/c2_params/lm_mowebearing",moweBearing);
	}

	void tick(){
		if(isMPointCompleted())
		{
			if(isMLCompleted)
			{
				setMLCompleted(true);
				isMLCompleted = false;
			}
			else
			{
				//only one set of points to send
				ROS_INFO("[%s] lm params:x=%f y=%f xlength=%f ylength=%f mwidth=%f mbearing=%f",agentName.c_str(),
						ml.m_state.pose.position.x,ml.m_state.pose.position.y,xLength,yLength,moweWidth,moweBearing);
				c2_ros_msgs::Trajectory traj = generateLMPath();


				if(traj.trajectory.size() > 0){

					isMLCompleted = true;

					//check which end point is the closest to the current position;
					double dist1 = asco::Utils::getDist2D(curPos,traj.trajectory.begin()->pose);
					double dist2 = asco::Utils::getDist2D(curPos,traj.trajectory.end()->pose);

					if(dist2 < dist1){
						//flip the order
						ROS_DEBUG("path order reversed...");
						std::reverse(traj.trajectory.begin(),traj.trajectory.end());
					}

					//compute the orientation
					computeOrientation(traj);

					//logging
					for(int i=0;i<traj.trajectory.size();i++)
					{
						c2_ros_msgs::State3D pt = traj.trajectory.at(i);
						ROS_INFO("[%s] lm | %f %f %f %f %f %f",agentName.c_str(),pt.pose.position.x,pt.pose.position.y,tf::getYaw(pt.pose.orientation)/M_PI*180,pt.twist.linear.x,pt.twist.linear.y,pt.twist.angular.z);
					}
					sendMPoint(traj);
				}
				else
				{
					ROS_ERROR("[%s] failed to plan a lm path!", agentName.c_str());
					setMLCompleted(false);
				}
			}
		}
	}

	void onStop(){
		isMLCompleted = false;
	}

	void onMPointFailure(){
		setMLCompleted(false);
		isMLCompleted = false;
	}

	c2_ros_msgs::Trajectory generateLMPath(){

		c2_ros_msgs::Trajectory p_path;
		c2_ros_msgs::Trajectory intercept1;
		c2_ros_msgs::Trajectory intercept2;

		//4 points for the 4 corners of the mow area.
		//for the moment, dive at the same altitude specfied in the mission file
		float halfXLength = xLength/2.0;
		float halfYLength = yLength/2.0;
		c2_ros_msgs::State3D TL,TR,BL,BR;

		//copy radius and linear speed from the misison leg
		TL.m_pt_radius = ml.m_pt_radius;
		TR.m_pt_radius = ml.m_pt_radius;
		BL.m_pt_radius = ml.m_pt_radius;
		BR.m_pt_radius = ml.m_pt_radius;

		//copy the default orientation
		TL.pose.orientation = ml.m_state.pose.orientation;
		TR.pose.orientation = ml.m_state.pose.orientation;
		BL.pose.orientation = ml.m_state.pose.orientation;
		BR.pose.orientation = ml.m_state.pose.orientation;

		//copy the default twist
		TL.twist = ml.m_state.twist;
		TR.twist = ml.m_state.twist;
		BL.twist = ml.m_state.twist;
		BR.twist = ml.m_state.twist;

		TL.pose.position.x = ml.m_state.pose.position.x - halfXLength;
		TL.pose.position.y = ml.m_state.pose.position.y + halfYLength;
		TL.pose.position.z = ml.m_state.pose.position.z;
		TR.pose.position.x = ml.m_state.pose.position.x + halfXLength;
		TR.pose.position.y = TL.pose.position.y;
		TR.pose.position.z = ml.m_state.pose.position.z;
		BL.pose.position.x = TL.pose.position.x;
		BL.pose.position.y = ml.m_state.pose.position.y - halfYLength;
		BL.pose.position.z = ml.m_state.pose.position.z;
		BR.pose.position.x = TR.pose.position.x;
		BR.pose.position.y = BL.pose.position.y;
		BR.pose.position.z = ml.m_state.pose.position.z;

		//determine if to mowe topdown or leftright.
		if(xLength>=yLength) //topdown
		{
			float dy = TR.pose.position.y;
			//record first two intersection point
			intercept1.trajectory.push_back(TL); // left
			intercept2.trajectory.push_back(TR); // right
			dy-=moweWidth;
			//start the loop
			while(dy>BR.pose.position.y)
			{
				c2_ros_msgs::State3D tmp = TL;
				tmp.pose.position.y = dy;
				intercept1.trajectory.push_back(tmp); // left
				tmp = TR;
				tmp.pose.position.y = dy;
				intercept2.trajectory.push_back(tmp); // right
				dy-=moweWidth;
			}
			//add the last two points
			intercept1.trajectory.push_back(BL); // left
			intercept2.trajectory.push_back(BR); // right

			//populate them into path.
			int cnt = intercept1.trajectory.size();
			bool isLeft = false;
			//just have to add the vetical points to form the whole path.EXCEPT the first point
			//isLeft=false =>                  **
			//                                                         **
			//                                                                 **
			//isLeft=true  => **               **
			//                                **
			//                                **

			//add the first point
			p_path.trajectory.push_back(intercept1.trajectory.at(0));
			for(int i=0;i<cnt-1;i++)
			{
				if(isLeft)
				{
					p_path.trajectory.push_back(intercept1.trajectory.at(i));
					p_path.trajectory.push_back(intercept1.trajectory.at(i+1));
					ROS_DEBUG("x=%f y=%f",intercept1.trajectory.at(i).pose.position.x,intercept1.trajectory.at(i).pose.position.y);
					isLeft = false;
				}
				else
				{

					p_path.trajectory.push_back(intercept2.trajectory.at(i));
					p_path.trajectory.push_back(intercept2.trajectory.at(i+1));
					ROS_DEBUG("x=%f y=%f",intercept2.trajectory.at(i).pose.position.x,intercept2.trajectory.at(i).pose.position.y);
					isLeft = true;
				}
			}
		}
		else//leftright
		{
			float dx = TL.pose.position.x;
			//record first two intersection point
			intercept1.trajectory.push_back(TL); // top
			intercept2.trajectory.push_back(BL); // bottom
			dx+=moweWidth;
			//start the loop
			while(dx<TR.pose.position.x)
			{
				c2_ros_msgs::State3D tmp = TL;
				tmp.pose.position.x = dx;
				intercept1.trajectory.push_back(tmp); // top
				tmp.pose.position.y = BL.pose.position.y;
				intercept2.trajectory.push_back(tmp);
				dx+=moweWidth;
			}
			//add last two points
			intercept1.trajectory.push_back(TR); // top
			intercept2.trajectory.push_back(BR); // bottom


			//populate them into path.
			int cnt = intercept1.trajectory.size();
			bool isTop = true;
			//just have to add the horizontal points to form the whole path.EXCEPT the first point
			//isTop=true   =>  **
			//isTop=false  =>    **
			//
			//

			//add the first point
			p_path.trajectory.push_back(intercept2.trajectory.at(0));
			for(int i=0;i<cnt-1;i++)
			{
				if(isTop)
				{
					p_path.trajectory.push_back(intercept1.trajectory.at(i));
					ROS_DEBUG("x=%f y=%f",intercept1.trajectory.at(i).pose.position.x,intercept1.trajectory.at(i).pose.position.y);
					isTop = false;
				}
				else
				{
					p_path.trajectory.push_back(intercept2.trajectory.at(i));
					ROS_DEBUG("x=%f y=%f",intercept2.trajectory.at(i).pose.position.x,intercept2.trajectory.at(i).pose.position.y);
					isTop = true;
				}
			}
		}

		//rotation and transformation if needed
		// printf("moweBearing=%f\n",moweBearing)
		if(moweBearing!=0)
		{
			float mb_rad = moweBearing*M_PI/180.0;

			int wpcnt = p_path.trajectory.size();
			c2_ros_msgs::State3D ps;
			float x=0.0;
			float y=0.0;
			for(int i=0;i<wpcnt;i++)
			{
				//transform to origin
				ps = p_path.trajectory.at(i);
				float x1 = ps.pose.position.x-ml.m_state.pose.position.x;
				float y1 = ps.pose.position.y-ml.m_state.pose.position.y;

				//rotation
				x = x1*cos(mb_rad)+y1*sin(mb_rad);
				y = -x1*sin(mb_rad)+y1*cos(mb_rad);

				//transform back to center point
				x1 = x + ml.m_state.pose.position.x;
				y1 = y + ml.m_state.pose.position.y;

				//put it back to the path
				ps.pose.position.x = x1;
				ps.pose.position.y = y1;
				//std::cout<<x1<<" "<<y1<<std::endl;
				p_path.trajectory[i] = ps;
			}
		}
		return p_path;

	}

	void computeOrientation(c2_ros_msgs::Trajectory& traj)
	{
		//compute the orientation of path segments and update its State3D orientation field
		for(int i=0; i<traj.trajectory.size()-1; i++)
		{
			double yaw = atan2((traj.trajectory[i+1].pose.position.y - traj.trajectory[i].pose.position.y),(traj.trajectory[i+1].pose.position.x - traj.trajectory[i].pose.position.x));
			geometry_msgs::Quaternion y_quat = tf::createQuaternionMsgFromYaw(yaw);
			traj.trajectory[i].pose.orientation = y_quat;
		}
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
