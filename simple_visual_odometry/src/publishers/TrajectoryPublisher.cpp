/*
 * simple_visual_odometry,
 *
 *
 * Copyright (C) 2015 Davide Tateo
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

#include "publishers/TrajectoryPublisher.h"

//#include <tf2_eigen/tf2_eigen.h>
#include "tf2_eigen.h" //FIXME levare quando funzionerà tf2_eigen

TrajectoryPublisher::TrajectoryPublisher(const std::string& frame_id)
{
	ros::NodeHandle n;

	trajectory_pub = n.advertise<nav_msgs::Path>("/visualization/path/" + frame_id, 1);
}


void TrajectoryPublisher::publishPath(const Eigen::Affine3d& T, const ros::Time& stamp)
{
	path.header.stamp = ros::Time::now();
	path.header.frame_id = "world";


	tf2::Stamped<Eigen::Affine3d> stamped(T, stamp, "world");

	geometry_msgs::PoseStamped  msgPose;

	tf2::convert(stamped, msgPose);

	path.poses.push_back(msgPose);

	trajectory_pub.publish(path);
}
