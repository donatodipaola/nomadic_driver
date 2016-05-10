/*********************************************************************************
 *
 *  Copyright (c) 2014, Donato Di Paola
 *
 *  Software License Agreement (MIT License)
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 *
 *********************************************************************************/

#include "nomadic_driver_node.h"

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel){
	scout_vm((int)(cmd_vel->linear.x /(INCH_TO_METER)*10), (int)(cmd_vel->angular.z/(DEGREE_TO_RADIAN)*10));
}

int main(int argc, char** argv) {

	////////////////////////////////////////////////////////////
	// ROS	
	ros::init(argc, argv, "nomadic_driver_node");

	ros::NodeHandle n;

	// Load parameters
	std::string port;
	std::string model;
	std::string topic_odom;
	std::string topic_cmd_vel;
	std::string tf_odom;
	std::string tf_base_link;

	n.param<std::string>("port", port, "/dev/ttyUSB0");
	n.param<std::string>("model", model, "Scout2");
	n.param<std::string>("topic_odom", topic_odom, "/odom");
	n.param<std::string>("topic_cmd_vel", topic_cmd_vel, "/cmd_vel");
	n.param<std::string>("tf_odom", tf_odom, "/odom");
	n.param<std::string>("tf_base_link", tf_base_link, "/base_link");


	// Pub/Sub
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>(topic_odom, 50);
	//ros::Publisher robot_state_pub = n.advertise<lse_sensor_msgs::Range>("/sonar", 50);
	ros::Subscriber cmd_vel_sub = n.subscribe(topic_cmd_vel, 1, cmdVelCallback);

	// tf 
	tf::TransformBroadcaster odom_broadcaster;


	int robot_model;
	if(model.compare("N200")==0) robot_model = MODEL_N200;
	else if(model.compare("N150")==0) robot_model = MODEL_N150;
	else if(model.compare("Scout")==0) robot_model = MODEL_SCOUT;
	else if(model.compare("Scout2")==0) robot_model = MODEL_SCOUT2;
	else{
		ROS_FATAL("nomadic_driver_node | Unknown robot model: %s!", model.c_str());
		ROS_FATAL("nomadic_driver_node | Options are N200, N150, Scout and Scout2");
		return 1;
	}


	////////////////////////////////////////////////////////////
	// Connect to the robot
	ROS_INFO("nomadic_driver_node | Connecting to the robot ...");

	if(connect_robot(1, robot_model, port.c_str(), SCOUT_BAUD_RATE)) {
		ROS_FATAL("nomadic_driver_node | Cannot connect to the robot");
		return 1;
	}

	// Reset
	dp(0,0);
	da(0,0);
	zr();
	// Set command timeout
	conf_tm(100);


	//TODO tune the pose covariance matrix
	//TODO read matrix from file
	const boost::array<double, 36ul> covariance_matrix_pose = {
		0.01, 0, 0, 0, 0, 0,
		0, 0.01, 0, 0, 0, 0,
		0, 0, 0.01, 0, 0, 0,
		0, 0, 0, 0.01, 0, 0,
		0, 0, 0, 0, 0.01, 0,
		0, 0, 0, 0, 0, 0.03};

	//TODO tune the twist covariance matrix
	//TODO read matrix from file
	const boost::array<double, 36ul> covariance_matrix_twist = {
		0.01, 0, 0, 0, 0, 0,
		0, 0.01, 0, 0, 0, 0,
		0, 0, 0.01, 0, 0, 0,
		0, 0, 0, 0.01, 0, 0,
		0, 0, 0, 0, 0.01, 0,
		0, 0, 0, 0, 0, 0.03};


	ros::Time current_time;
	ros::Rate rate(10.0);


	while(ros::ok()){

		current_time = ros::Time::now();

		// Update the robot readings
		gs();

		float odom_x = State[STATE_CONF_X]*INCH_TO_METER/10.0;
		float odom_y = State[STATE_CONF_Y]*INCH_TO_METER/10.0;
		float odom_yaw =  State[STATE_CONF_STEER]*DEGREE_TO_RADIAN/10.0;

		float odom_velx = State[STATE_VEL_TRANS]*INCH_TO_METER/10.0;
		float odom_velz = State[STATE_VEL_STEER]*DEGREE_TO_RADIAN/10.0;

		// tf
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_yaw);
		geometry_msgs::TransformStamped odom_trans;

		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = tf_odom;
		odom_trans.child_frame_id = tf_base_link;

		odom_trans.transform.translation.x = odom_x;
		odom_trans.transform.translation.y = odom_y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		odom_broadcaster.sendTransform(odom_trans);

		// Odometry
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = tf_odom;
		odom.child_frame_id = tf_base_link;

		odom.pose.pose.position.x = odom_x;
		odom.pose.pose.position.y = odom_y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		odom.pose.covariance = covariance_matrix_pose;

		odom.twist.twist.linear.x = odom_velx;
		odom.twist.twist.linear.y = 0.0;
		odom.twist.twist.angular.z = odom_velz;
		odom.twist.covariance = covariance_matrix_twist;

		odom_pub.publish(odom);


		// Robot state
		NRobotState myRobotState;		
		char status = State[STATE_MOTOR_STATUS];

		myRobotState.left_wheel_moving = status & 0x01;
		myRobotState.right_wheel_moving = (status>>1) & 0x01;

		int battery_state;
		bool b0 = (status>>2) & 0x01;
		bool b1 = (status>>3) & 0x01;
		if(!b0 && !b1) battery_state = BATTERY_LOW;
		else if(b0 && !b1) battery_state = BATTERY_MED;
		else if(!b0 && b1) battery_state = BATTERY_HIGH;

		myRobotState.battery_state = battery_state;

		myRobotState.is_plugged = (status>>4) & 0x01;
		myRobotState.is_charging = (status>>5) & 0x01;
		myRobotState.emergency_stop = (status>>6) & 0x01;

		//robotStatePub.publish(myRobotState);

		ros::spinOnce();
		rate.sleep();
	}

	// Stop the robot
	st();

	// Disconnect the robot
	disconnect_robot(1);
	
	return 0;
}
