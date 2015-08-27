#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include <c2_ros/pilot.h>
#include <c2_ros/c2_agent.h>
#include <c2_ros/Trajectory.h>
#include <c2_ros/State3D.h>
#include <asco_utils/utils.h>
#include <vector>
#include <math.h>

#include <dynamic_reconfigure/server.h>
#include <c2_ros/DDPInterfaceConfig.h>
#include <c2_ros/controlConfig.h>

#include "gcop/ddp.h"
#include "gcop/lqcost.h"


#include "gcop/se2.h"
#include "gcop/body2d.h"
#include <gcop_comm/CtrlTraj.h>
#include <gcop_comm/State.h>
#include <mavros/ActuatorControl.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>


#include <stdio.h>


#define MAX_SPEED 5
using namespace std;
using namespace Eigen;
using namespace gcop;
typedef Ddp< pair<Matrix3d, Vector3d>, 6, 2> Body2dDdp;
typedef Matrix< double , 6 , 1> Vector6d;

  Body2dState x0,xf;

  VectorXd Q(6), Qf(6),R(2);

	int iters=100;
	int N = 64;
	double MU=0.01;
	float eeps=1e-3;
	gcop_comm::CtrlTraj trajectory;
	ros::Publisher trajpub;
	ros::Subscriber odom_est_sub;
	double  thrust_offset=0;
		
	Body2dForce<2> force;
	Body2d<2> sys(&force);
	vector<double> ts(N+1);
	Vector2d u(0,0);
	vector<Vector2d> us(N, u);
	double tfinal = 5;
	//geometry_msgs::Pose2D curPos;

void paramreqcallback(c2_ros::DDPInterfaceConfig &config, uint32_t level) 
{
	ROS_INFO("Received Reconfigure Request ");
	
	SE2 &se2 = SE2::Instance();  



	VectorXd qv0(6) ;// initial state
	VectorXd qvf(6);
	qvf(0) = config.thetaN;
	qvf(1) = config.xN;
	qvf(2) = config.yN;
	qvf(3) = config.omegaN;
	qvf(4) = config.vxN;
	qvf(5) = config.vyN;

	qv0(0) = config.theta0;
	qv0(1) = config.x0;
	qv0(2) = config.y0;
	qv0(3) = config.omega0;
	qv0(4) = config.vx0;
	qv0(5) = config.vy0;



    se2.q2g(x0.first, qv0.head(3));
    x0.second = qv0.tail(3);


    se2.q2g(xf.first, qvf.head(3));
    xf.second = qvf.tail(3);



	Q(0) = config.Q1;
	Q(1) = config.Q2;
	Q(2) = config.Q3;
	Q(3) = config.Q4;
	Q(4) = config.Q5;
	Q(5) = config.Q6;

	Qf(0) = config.Qf1;
	Qf(1) = config.Qf2;
	Qf(2) = config.Qf3;
	Qf(3) = config.Qf4;
	Qf(4) = config.Qf5;
	Qf(5) = config.Qf6;

	R(0) = config.R1;
	R(1) = config.R2;

  iters = config.Nit;
	MU=config.mu;
	eeps=pow(10,config.eps);
	thrust_offset=config.thrust_offset;
}

void pubtraj(vector<pair<Matrix3d, Vector3d> > xss,vector<Vector2d> uss,Body2dState xfs,vector<double> tss) //N is the number of segments
{
  vector<Vector6d> qxs(N+1);
  VectorXd qxf(6);
	SE2 &se2 = SE2::Instance();  
	Vector3d vxs;
	for (int count = 0;count<N+1;count++)
	{
		se2.g2q(vxs,xss[count].first); 

		qxs[count].head(3)=vxs;
		qxs[count].tail(3)=xss[count].second;
		for(int count1 = 0;count1 < 6;count1++)
		{
			trajectory.statemsg[count].statevector.resize(6);

			trajectory.statemsg[count].statevector[count1] = qxs[count](count1);
		}
	}
	for (int count = 0;count<N;count++)
	{
		for(int count1 = 0;count1 < 2;count1++)
		{
			trajectory.ctrl[count].ctrlvec.resize(2);

			trajectory.ctrl[count].ctrlvec[count1] = uss[count](count1);
		}
	}
	trajectory.time = tss;
	//final goal:

	Vector3d vq;
	se2.g2q(vq,xfs.first);
	qxf.head(3)=vq;
	qxf.tail(3)=xfs.second;

	for(int count = 0;count<6;count++)
	{
		trajectory.finalgoal.statevector[count] = qxf(count);
	}
trajpub.publish(trajectory);
}

void poseTwistToState(Body2dState &x,const geometry_msgs::Pose pose,const geometry_msgs::Twist twist)
{
		
		SE2 &se2 = SE2::Instance(); 
		VectorXd qv0(6);
		qv0(0) = tf::getYaw(pose.orientation);
		qv0(1) =  pose.position.x;
		qv0(2) =pose.position.y;
	
		qv0(3) = twist.angular.z;
		qv0(4) = twist.linear.x;
		qv0(5) = twist.linear.y;

		se2.q2g(x.first, qv0.head(3));
	    	x.second = qv0.tail(3);


}

geometry_msgs::Pose2D StatetoPose2D(const Body2dState x)
{
	Vector3d xpose;
	geometry_msgs::Pose2D pose;
	const Matrix3d x_gmat=x.first;
	SE2::Instance().g2q(xpose, x_gmat);
	pose.x=xpose(0);
	pose.y=xpose(1);
	pose.theta=xpose(2);
	return pose;
}


	void odom_x_est(const nav_msgs::Odometry::ConstPtr& odom_pos_est)
{
	poseTwistToState(x0,odom_pos_est->pose.pose,odom_pos_est->twist.twist);
		//curPos.x = odom_pos_est->pose.pose.position.x;
		//curPos.y = odom_pos_est->pose.pose.position.y;
		//curPos.theta = tf::getYaw(odom_pos_est->pose.pose.orientation);

}

	double FtoLevel(double Force)
	{
		double level=0;
		if(Force>0)
		{

			level=-0.000375120937947*Force*Force+0.037259288160874*Force;
			level=level+thrust_offset;
		}

		if(Force<0)
		{
			Force=-Force;
			level=-0.000375120937947*Force*Force+0.037259288160874*Force;
			level=-level-0.15;
			level=level-thrust_offset;

		}		
		return level;
	}
		

namespace C2{

class Pilot_Ddp: public Pilot
{

private:
	c2_ros::Trajectory poseToRun;
	c2_ros::State3D curMP;
	
	c2_ros::Trajectory subTraj;
	c2_ros::State3D subPose;
	int subPosecnt=0;
	
	
	int poseCnt;
	bool isCompleted;
	bool isReinitialized;
	bool isCurMPReached=false;
	bool isCurSubMPReached=false;
	bool isSubTrajRequired=false;
	double curBearing,bearingSP, speedSP;
	ros::Publisher actuator_controls_pub;
	float sideDist = 20;
	float secAngle = 10;

public:


	Pilot_Ddp(int loopRate, ros::NodeHandle nh):
		Pilot(loopRate,nh),
		poseCnt(0),
		isCompleted(true),
		isReinitialized(true),
		isCurMPReached(true)
{
	 actuator_controls_pub = nh.advertise<mavros::ActuatorControl>("/mavros/actuator_control", 1000);
		
	std::string odm_name;
		if (!nh_.getParam("/global_params/odometry_topic_name",odm_name)) odm_name = "/insekf/pose";
		odom_est_sub = nh.subscribe(odm_name,1000, odom_x_est);




	trajpub = nh.advertise<gcop_comm::CtrlTraj>("/usvnode/ctrltraj",1);

	


  	


 	force.D(0)= 0.01;
 	force.D(1)= 0.02;
  	force.D(2)= 5;

	MatrixXd m(3,2);
	m << 0.265,-0.265,1,1,0,0;
	force.B=m;
	
  		Vector2d d;
	d << 1.16,0.72;
	sys.d=d;

	Vector3d Ii;

	//Inertia vect is (I,m,m)
	Ii << 3.572,23,23;
	sys.I=Ii;
 	sys.U.lb<<-100,-100;
	sys.U.ub<<100,100;
  	sys.U.bnd = true;
  	
  	
  	for (int k = 0; k <=N; ++k)
    	ts[k] = k*tfinal/N;
  	

	trajectory.N = N;
	trajectory.statemsg.resize(N+1);
	trajectory.ctrl.resize(N);
	trajectory.time = ts;
	trajectory.finalgoal.statevector.resize(6);

  	for (int i = 0; i < N/2; ++i) 
  	{
    		us[i] = Vector2d(0,0);
  		us[N/2+i] = Vector2d(0,0);
  	}



}

	~Pilot_Ddp()
	{

	}

	void getSubTraj(geometry_msgs::Pose2D pose,c2_ros::State3D MP,int parts)
	{
	
		for (int k=1;k<=parts;k++)
		{
			double i=(k*1.0)/(parts*1.0);
			c2_ros::State3D subState;
			subState.header=MP.header;
			subState.m_pt_radius=MP.m_pt_radius;
			subState.pose.orientation=MP.pose.orientation;
			subState.pose.position.x = pose.x * (1-i) + MP.pose.position.x * i;
			subState.pose.position.y = pose.y * (1-i) + MP.pose.position.y * i ;
			subState.twist=MP.twist;
			subTraj.trajectory.push_back(subState);
		}
	
	}


	void resetSubStates()
	{
		subTraj.trajectory.clear();
		subPosecnt=0;
		isCurSubMPReached=false;
		isSubTrajRequired=false;
	
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
					isCurMPReached = false;
					curMP = poseToRun.trajectory.at(poseCnt);
					ROS_INFO("Navigating to x=%f, y=%f",curMP.pose.position.x,curMP.pose.position.y);
					double curDist=asco::Utils::getDist2D(StatetoPose2D(x0), curMP.pose);
					if(curDist> MAX_SPEED*tfinal)
					{
						isSubTrajRequired=true;
						getSubTraj(StatetoPose2D(x0), curMP,(int)ceil(curDist/(MAX_SPEED*tfinal*1.0)));
						isCurSubMPReached=true;
					}
					
					else isSubTrajRequired=false;
					//check vehicle's side angles
				//	isCurMPReached = checkDistAngle();
				//	if(isCurMPReached) return;

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
			
			if(!isSubTrajRequired)
			{
				if(navigateTo(curMP))
				{
					isCurMPReached = true;
				
				}
			}
			else
			{
				if(isCurSubMPReached){
				if(subPosecnt < subTraj.trajectory.size())
				{
					isCurSubMPReached = false;
					subPose = subTraj.trajectory.at(subPosecnt);
					ROS_INFO("Navigating in Sub Trajectory to to x=%f, y=%f",subPose.pose.position.x,subPose.pose.position.y);

					subPosecnt++;
				}
				else
				{
					ROS_INFO("Sub Trajectory Completed");
					resetSubStates();
					return;
				}
			}
				
			
				if(navigateTo(subPose))
				{
					isCurSubMPReached = true;
				
				}
			
			
			
			
			}
			
			
			
		}
	}

	void stopVehicle()
	{
		 mavros::ActuatorControl actuator_control_msg;

        
        	//controls[1] goes to right thruster, controls[3] goes to left thruster.
	        actuator_control_msg.header.stamp = ros::Time::now();
	        actuator_control_msg.group_mix = 0;
	        actuator_control_msg.controls[0] = 0.0;
	        actuator_control_msg.controls[1] = 0.0;//FtoLevel(us[0][0]);
	        actuator_control_msg.controls[2] = 0.0;
	        actuator_control_msg.controls[3] = 0.0;//FtoLevel(us[0][1]);
	        actuator_control_msg.controls[4] = 0.0;
	        actuator_control_msg.controls[5] = 0.0;
	        actuator_control_msg.controls[6] = 0.0;
	        actuator_control_msg.controls[7] = 0.0;
	        actuator_controls_pub.publish(actuator_control_msg);
	}

	bool navigateTo(c2_ros::State3D mp)
	{
		//simulate succeed
			double dist = asco::Utils::getDist2D(StatetoPose2D(x0), mp.pose);
			
		if(dist > mp.m_pt_radius){

		poseTwistToState(xf,mp.pose,mp.twist);
			LqCost<Body2dState, 6, 2> cost(sys, tfinal, xf);
			cost.Q = Q.asDiagonal();
			cost.Qf = Qf.asDiagonal();
			cost.R = R.asDiagonal();
		
			vector<pair<Matrix3d, Vector3d> > xs(N+1);
		
		
			xs[0]= x0;

		

			std::rotate(us.begin(), us.begin() + 1, us.end());
			Body2dDdp ddp(sys, cost, ts, xs, us);
  		ddp.mu = MU;

			ddp.eps=eeps;
  		ddp.debug = false; 
			///struct timeval timer;
  		for (int i = 0; i < iters; ++i) 
  		{
		   // timer_start(timer);
		    ddp.Iterate();
		  //  long te = timer_us(timer);
		   // cout << "Iteration #" << i << " took: " << te << " us." << endl;    

			}	
		
			pubtraj(xs,us,xf,ts);
		

	
      mavros::ActuatorControl actuator_control_msg;

      //controls[1] goes to right thruster, controls[3] goes to left thruster.
      actuator_control_msg.header.stamp = ros::Time::now();
      actuator_control_msg.group_mix = 0;
      actuator_control_msg.controls[0] = 0.0;
      actuator_control_msg.controls[1] = FtoLevel(us[0][0]);
      actuator_control_msg.controls[2] = 0.0;
      actuator_control_msg.controls[3] = FtoLevel(us[0][1]);
      actuator_control_msg.controls[4] = 0.0;
      actuator_control_msg.controls[5] = 0.0;
      actuator_control_msg.controls[6] = 0.0;
      actuator_control_msg.controls[7] = 0.0;
      actuator_controls_pub.publish(actuator_control_msg);

			/*Vector3d xfprint,x0print,velf,vel0;
			const Matrix3d xfprmat=xf.first;
    	SE2::Instance().g2q(xfprint, xfprmat);

			Vector2d dist_vec;
			dist_vec<<mp.pose.position.x-xfprint(1),mp.pose.position.y-xfprint(2);
			if(dist_vec.norm()<0.50)
			{
			ROS_INFO("reached x=%f, y=%f",mp.pose.position.x,mp.pose.position.y);
			isCurMPReached = true;

			}*/



			return false;
		}

		ROS_INFO("reached x=%f, y=%f",mp.pose.position.x,mp.pose.position.y);
		return true;
	}

	bool checkDistAngle()
	{
		float angle = asco::Utils::calAngle(StatetoPose2D(x0), curMP.pose);
		float dist = asco::Utils::getDist2D(StatetoPose2D(x0), curMP.pose);
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
	        dynamic_reconfigure::Server<c2_ros::DDPInterfaceConfig> server;
        dynamic_reconfigure::Server<c2_ros::DDPInterfaceConfig>::CallbackType f;

        f = boost::bind(&paramreqcallback, _1, _2);
        server.setCallback(f);

	C2::Pilot_Ddp c(10,nh);
	c.spin();

	return 0;
}
