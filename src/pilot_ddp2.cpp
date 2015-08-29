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

class Pilot_Ddp2: public Pilot
{

private:
	c2_ros::Trajectory poseToRun;
	c2_ros::State3D curWaypt;

	int poseCnt;
	bool isCompleted;
	bool isCurWayPtReached=false;
	ros::Publisher actuator_controls_pub;

public:

	Pilot_Ddp2(int loopRate, ros::NodeHandle nh):
		Pilot(loopRate,nh),
		poseCnt(0),
		isCompleted(true),
		isCurWayPtReached(true)
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

	~Pilot_Ddp2(){}

	void tick()
	{
		if(!isCompleted)
		{
			if(isCurWayPtReached){
				if(poseCnt < poseToRun.trajectory.size())
				{
					isCurWayPtReached = false;
					curWaypt = poseToRun.trajectory.at(poseCnt);
					ROS_INFO("[%s]: Navigating to x=%f, y=%f",agentName.c_str(),curWaypt.pose.position.x,curWaypt.pose.position.y);

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

			//navigate to waypoint
			if(navigateTo(curWaypt))
			{
				isCurWayPtReached = true;
			}
		}
		else
			//make sure 0 signal is sent all the time when the pilot has no waypoint to navigate to
			stopVehicle();
	}

	inline void stopVehicle()
	{
		mavros::ActuatorControl actuator_control_msg;
		//controls[1] goes to right thruster, controls[3] goes to left thruster.
		actuator_control_msg.header.stamp = ros::Time::now();
		actuator_control_msg.group_mix = 0;
		actuator_control_msg.controls[0] = 0.0;
		actuator_control_msg.controls[1] = 0.0;
		actuator_control_msg.controls[2] = 0.0;
		actuator_control_msg.controls[3] = 0.0;
		actuator_control_msg.controls[4] = 0.0;
		actuator_control_msg.controls[5] = 0.0;
		actuator_control_msg.controls[6] = 0.0;
		actuator_control_msg.controls[7] = 0.0;
		actuator_controls_pub.publish(actuator_control_msg);
	}

	bool navigateTo(c2_ros::State3D wp)
	{
		//calculate the distance from the current position to destination pose
		double dist = asco::Utils::getDist2D(StatetoPose2D(x0), wp.pose);

		if(dist > wp.m_pt_radius){

			poseTwistToState(xf,wp.pose,wp.twist);
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

			//send control signal to the thruster
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

			return false;
		}

		ROS_INFO("[%s]:reached x=%f, y=%f",agentName.c_str(),wp.pose.position.x,wp.pose.position.y);
		return true;
	}

	void onStop()
	{
		poseToRun.trajectory.clear();
		poseCnt = 0;
		isCompleted = true;
		isCurWayPtReached = true;
	}

	c2_ros::Trajectory prepareTraj(c2_ros::Trajectory mTraj)
	{
		//TODO: To make the algo run a bit faster, prepare this and return vector of Body2dState ??

		//check the distance of the waypoints in the trajectory
		//if dist > dist_threshold, subdivide them int subtraj

		geometry_msgs::Pose2D p = StatetoPose2D(x0);
		c2_ros::Trajectory return_traj;
		c2_ros::Trajectory::_trajectory_type::iterator it;
		for(it = mTraj.trajectory.begin(); it != mTraj.trajectory.end(); it++){
			double dist=asco::Utils::getDist2D(p, (*it).pose);
			if(dist > MAX_SPEED*tfinal)
			{
				//generate sub trajectories
				//calculate number of traj parts
				ROS_INFO("[%s]:dist [%f] > [%f], generate sub-trajectory",agentName.c_str(),dist,MAX_SPEED*tfinal);
				int parts = (int)ceil(dist/(double)(MAX_SPEED*tfinal));
				for (int k=1;k<=parts;k++)
				{
					double i=(double)k/(double)parts;
					c2_ros::State3D subState;
					subState.header=(*it).header;
					subState.m_pt_radius=(*it).m_pt_radius;
					subState.pose.orientation=(*it).pose.orientation;
					subState.pose.position.x = p.x * (1-i) + (*it).pose.position.x * i;
					subState.pose.position.y = p.y * (1-i) + (*it).pose.position.y * i ;
					subState.twist=(*it).twist;
					return_traj.trajectory.push_back(subState);
					ROS_INFO("[%s]:sub[%d] - x=%f y=%f",agentName.c_str(),k,p.x,p.y);
				}
			}
			else
			{
				return_traj.trajectory.push_back((*it));
			}

			//renew p
			p.x = (*it).pose.position.x;
			p.y = (*it).pose.position.y;
		}
		return return_traj;
	}

	void newMissionPointAvailable(c2_ros::Trajectory traj, bool isOverwrite)
	{
		ROS_INFO("Mission point received by [%s]",agentName.c_str());
		if(isOverwrite)
		{
			//start from beginning
			isCurWayPtReached = true;
			poseCnt = 0;
			poseToRun.trajectory = prepareTraj(traj).trajectory;
		}
		else
		{
			//prepare the appending traj and append to the current trajectory
			c2_ros::Trajectory traj_to_apped = prepareTraj(traj);
			c2_ros::Trajectory::_trajectory_type::iterator it;
			for(it = traj_to_apped.trajectory.begin(); it != traj_to_apped.trajectory.end(); it++)
			{
				poseToRun.trajectory.push_back(*it);
			}
		}

		isCompleted = false;
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

	C2::Pilot_Ddp2 c(10,nh);
	c.spin();

	return 0;
}
