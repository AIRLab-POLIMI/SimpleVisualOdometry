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

#ifndef INCLUDE_FEATURESPUBLISHER_H_
#define INCLUDE_FEATURESPUBLISHER_H_

#include <ros/ros.h>
#include "core/Features.h"
#include <Eigen/Geometry>

class FeaturesPublisher
{
public:
	FeaturesPublisher();
	void publishFeatureMarkers(const Features3D& features, const Eigen::Affine3d& F);

private:
	ros::Publisher markers_pub;

};


#endif /* INCLUDE_FEATURESPUBLISHER_H_ */
