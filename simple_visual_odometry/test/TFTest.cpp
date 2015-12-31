/*
 * TFTest.cpp
 *
 *  Created on: 27 dic 2015
 *      Author: dave
 */

#ifndef TEST_TFTEST_CPP_
#define TEST_TFTEST_CPP_

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "ConfigManager.h"

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "tf_test");

	ros::NodeHandle n;

	tf::Transform T_WC;
	tf::TransformBroadcaster tfBroadcaster;
	tf::TransformListener tfListener;

	tf::Transform T_CR;
	tf::Transform T_RC;



	ConfigManager config;

	ROS_INFO("TF test started");

	tf::Quaternion q_RC(0.5, -0.5, 0.5, 0.5);
	tf::Quaternion q_CR(0.5, -0.5, 0.5, -0.5);
	tf::Vector3 t0(0, 0, 0);

	T_CR.setRotation(q_CR);
	T_CR.setOrigin(t0);

	T_RC.setRotation(q_RC);
	T_RC.setOrigin(t0);


	double roll, pitch, yaw;
	config.T_WR.getBasis().getRPY(roll, pitch, yaw);

	tf::Quaternion q;
	q.setRPY(roll, pitch, yaw);

	std::cout << q.getX() << " " << q.getY() << " " << q.getZ() << " " << q.getW() << std::endl;

	//Init pose
	T_WC = config.T_WR * T_RC;

	while (ros::ok())
	{
		tfBroadcaster.sendTransform(
								tf::StampedTransform(config.T_WR, ros::Time::now(), "world",
											"T_WR"));

		tfBroadcaster.sendTransform(
						tf::StampedTransform(T_WC, ros::Time::now(), "world",
									"T_WC"));

		tfBroadcaster.sendTransform(
								tf::StampedTransform(T_WC*T_CR, ros::Time::now(), "world",
											"T_WR_back"));

		sleep(1);
		ros::spinOnce();
	}

}

#endif /* TEST_TFTEST_CPP_ */
