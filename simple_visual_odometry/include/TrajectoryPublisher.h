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

#ifndef INCLUDE_TRAJECTORYPUBLISHER_H_
#define INCLUDE_TRAJECTORYPUBLISHER_H_

#include <nav_msgs/Path.h>
#include <Eigen/Geometry>
#include <ros/ros.h>

class TrajectoryPublisher
{
public:
	TrajectoryPublisher(const std::string& frame_id);
	void publishPath(const Eigen::Affine3d& T, const ros::Time& stamp);

private:
	nav_msgs::Path path;
	ros::Publisher trajectory_pub;

};



#endif /* INCLUDE_TRAJECTORYPUBLISHER_H_ */
