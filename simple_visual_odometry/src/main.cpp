/*
 * simple_visual_odometry,
 *
 *
 * Copyright (C) 2014 Davide Tateo
 * Versione 1.0
 *
 * This file is part of simple_visual_odometry.
 *
 * simple_visual_odometry is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * simple_visual_odometry is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with simple_visual_odometry.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "logic/VisualOdometryLogic.h"


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "simple_visual_odometry");

	if (argc < 2)
	{
		ROS_FATAL("argument needed: camera_source");
		return -1;
	}

	ros::NodeHandle n;
	string imageTopic = argv[1];

	VisualOdometryLogic logic(imageTopic, n);

	ROS_INFO("Visual Odometry node started");

	ros::spin();
}
