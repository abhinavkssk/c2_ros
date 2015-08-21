#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geodesy/utm.h>
#include <tf/transform_broadcaster.h>
#include <c2_ros/ActuatorControl.h>
#include <vector>
#include <math.h>

#include <visualization_msgs/Marker.h>

#define PIdiv2 M_PI/2
#define PItim2 M_PI*2

namespace C2{

class VSim{

private:
	//ros related
	ros::NodeHandle nh_;
	ros::Publisher odo_pub;
	ros::Subscriber ac_sun;

	//param
	double turnRate = 20;
	double diveMinSpeed = 1;
	double diveDownRate = 0.2;
	double diveUpRate = 0.5;
	double thrustScale = 1;

	////simulator parameters
	double currentSpeed = 0.1;
	double currentYaw = 0;
	std::vector<float> cur {0,0,0};

	//variable
	double bearing,thrust,turn,depthSP,time;
	double xPos,yPos,depth,xOld,yOld;

public:
	VSim(ros::NodeHandle n):bearing(0),thrust(0),turn(0),depthSP(0),depth(0),time(ros::Time::now().toSec()),nh_(n)
{
		std::string odm_name;
		if (!nh_.getParam("/c2_params/odometry_topic_name",odm_name)) odm_name = "/odometry/filtered";
		odo_pub = nh_.advertise<nav_msgs::Odometry>(odm_name,1);

		ac_sun = nh_.subscribe("/c2_ros/actuator_control",1,&C2::VSim::actuatorControl,this);

		retrieveParams();
		xOld = xPos;
		yOld = yPos;

}
	~VSim()
	{

	}

	void actuatorControl(const c2_ros::ActuatorControl::ConstPtr& ac)
	{
		ROS_DEBUG("Actuator Control received");
		setThrust(speed2thrust(ac->desired_speed));
		turnTo(ac->desired_bearing);
		diveTo(ac->desired_depth);
	}

	void retrieveParams()
	{
		if (!nh_.getParam("/c2_params/startX",xPos)) xPos = 0;
		if (!nh_.getParam("/c2_params/startY",yPos)) yPos = 0;
		double lat,lon;
		if (nh_.getParam("/c2_params/startX_lat",lat) &&
				nh_.getParam("/c2_params/startY_lon",lon))
		{
			geodesy::UTMPoint utmp;
			geodesy::fromMsg(geodesy::toMsg(lat,lon),utmp);
			xPos = utmp.easting;
			yPos = utmp.northing;
		}

		if (!nh_.getParam("/c2_params/turnRate",turnRate)) turnRate = 10;
		if (!nh_.getParam("/c2_params/thrustScale",thrustScale)) thrustScale = 1;
		if (!nh_.getParam("/c2_params/currentSpeed",currentSpeed)) currentSpeed = 0;
		if (!nh_.getParam("/c2_params/currentYaw",currentYaw)) currentYaw = 0;

	}

	///// seaCurrent
	void getWorldCurrent(float x, float y, std::vector<float> &curvec)
	{
		curvec[0] = (float)currentSpeed*cos(currentYaw);
		curvec[1] = (float)currentSpeed*sin(currentYaw);
		curvec[2] = 0.0;
	}

	//// getter methods

	double thrust2speed(double thrust) {
		if (thrust < 0.1 && thrust > -0.1) return 0;
		double v = (3.2*thrust-0.95)*thrustScale;
		if (thrust < 0) v /= 2;
		return v;
	}

	double speed2thrust(double speed) {
		if(speed == 0) return 0;
		return ((speed/thrustScale)+0.95)/3.2;
	}

	double getbearing()
	{
		return bearing;
	}

	float getHeading()
	{
		double speed = thrust2speed(thrust);
		float heading = atan2((speed-cur[1]), (speed-cur[0]));
		heading = -(heading-PIdiv2);
		while(heading<0) heading += PItim2;
		return heading*180/M_PI;
	}

	double getSpeed() {
		return thrust2speed(thrust);
	}

	//// helper methods

	double bearing2yaw(double bearing) {
		return toRadians(90-bearing);
	}

	int sign(double x) {
		if (x > 0) return 1;
		if (x < 0) return -1;
		return 0;
	}

	//////////////////////////// control functions

	//setting the vehicle's thrust
	void setThrust(double thrust)
	{
		if(thrust >= -1 && thrust <= 1) this->thrust = thrust;
	}

	void diveTo(double depth)
	{
		if(depth > 0) depthSP = depth;
	}

	void turnTo(double bearing)
	{
		turnTo(bearing,0);
	}

	void turnTo(double bearing, int dir)
	{
		while (bearing-this->bearing < 0) bearing += 360;
		while (bearing-this->bearing >= 360) bearing -= 360;
		double cw = bearing-this->bearing;
		double ccw = cw-360;
		if (dir > 0) turn = cw;
		else if (dir < 0) turn = ccw;
		else if (cw < -ccw) turn = cw;
		else turn = ccw;
	}

	void updateTo(double time)
	{
		if (time <= this->time) return;
		double speed = thrust2speed(thrust);
		double dt = (time-this->time);
		double t_lapsed = dt;
		//ROS_INFO("lapse=%f",dt);
		this->time = time;
		//get current
		getWorldCurrent((float)xPos,(float)yPos, cur);
		//ROS_INFO("cur x=%f,y=%f",cur[0],cur[1]);
		// depth control
		double dsp = depthSP;
		if (speed < diveMinSpeed) dsp = 0;
		if (depth < dsp) {
			double dt1 = (dsp-depth)/diveDownRate;
			if (dt1 > dt) dt1 = dt;
			depth += diveDownRate*dt1;
		} else if (depth > dsp) {
			double dt1 = (depth-dsp)/diveUpRate;
			if (dt1 > dt) dt1 = dt;
			depth -= diveUpRate*dt1;
		}
		// yaw control
		double dh = 0;
		if (turn != 0) {
			double dt1 = fabs(turn)/turnRate;
			if (dt1 > dt) dt1 = dt;
			dh = sign(turn)*turnRate*dt1;
			double a1 = bearing2yaw(bearing);
			double a2 = bearing2yaw(bearing+dh);
			double da = -sign(turn)*toRadians(turnRate);
			xPos += (speed-cur[0])/da*(sin(a2)-sin(a1));
			yPos += (speed-cur[1])/da*(cos(a1)-cos(a2));
			bearing += dh;
			turn -= dh;
			dt -= dt1;
		}
		// forward motion
		if (dt > 0) {
			double yaw = bearing2yaw(bearing);
			xPos += (speed*cos(yaw)-cur[0])*dt;
			yPos += (speed*sin(yaw)-cur[1])*dt;
			//ROS_INFO("speed=%f,yaw=%f,cur:x=%f,y=%f,dt=%f,xpos=%f,ypos=%f",speed,yaw,cur[0],cur[1],dt,xPos,yPos);
		}

		//make sure the bearing is within range
		while (bearing < 0) bearing += 360;
		while (bearing >= 360) bearing -= 360;

		nav_msgs::Odometry odom;
		odom.header.stamp = ros::Time(this->time);
		odom.header.frame_id = "odom";

		//set the position
		odom.pose.pose.position.x = xPos;
		odom.pose.pose.position.y = yPos;
		odom.pose.pose.position.z = 0.0;
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(bearing2yaw(bearing));;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = (xPos-xOld)/t_lapsed;
		odom.twist.twist.linear.y = (yPos-yOld)/t_lapsed;
		odom.twist.twist.angular.z = dh;

		//publish the message
		odo_pub.publish(odom);

		xOld = xPos;
		yOld = yPos;

		//ROS_INFO("bearing in vec=%f",bearing);

	}

	double toRadians(double deg)
	{
		return deg/180.0*M_PI;
	}

};

}

int main (int argc, char ** argv)
{
	ros::init(argc, argv, "VEHICLE_SIMULATOR");
	ros::NodeHandle n;
	ros::Rate loop_rate(10); //default to 10Hz

	C2::VSim c(n);

	//iterate
	while (ros::ok())
	{
		c.updateTo(ros::Time::now().toSec());
		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;
}

