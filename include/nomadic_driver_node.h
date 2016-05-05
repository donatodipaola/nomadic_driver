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

#ifndef NOMADIC_DRIVER_NODE_H_
#define NOMADIC_DRIVER_NODE_H_

// ROS libs
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>			
#include <geometry_msgs/Twist.h>		

// Nomadic Libs
#include "../lib/ndirect/Nclient.h"

//  General Parameters
#define INCH_TO_METER		0.0254
#define DEGREE_TO_RADIAN	0.0174532925

#define SCOUT_BAUD_RATE		38400
#define BRIDGE_BAUD_RATE	19200

#define BATTERY_LOW 	0
#define BATTERY_MED 	1
#define BATTERY_HIGH 	2

long State[NUM_STATE];

// Nomadic Robot State Data
struct NRobotState{
	bool left_wheel_moving;
	bool right_wheel_moving;

	int battery_state;

	bool is_plugged;
	bool is_charging;
	bool emergency_stop;
};


#endif /* NOMADIC_DRIVER_NODE_H_ */
