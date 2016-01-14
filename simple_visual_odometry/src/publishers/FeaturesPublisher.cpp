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

#include "publishers/FeaturesPublisher.h"
#include <visualization_msgs/Marker.h>

FeaturesPublisher::FeaturesPublisher()
{
	ros::NodeHandle n;

	markers_pub = n.advertise<visualization_msgs::Marker>(
	      "/visualization/features", 1);

}

void FeaturesPublisher::publishFeatureMarkers(const Features3D& features, const Eigen::Affine3d& F)
{
  static unsigned int id = 0;
  visualization_msgs::Marker msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  msg.type = visualization_msgs::Marker::CUBE_LIST;
  msg.frame_locked = false;
  msg.ns = "vo";
  msg.id = id++;
  msg.action = visualization_msgs::Marker::ADD;

  msg.color.r = 1.0;
  msg.color.g = 0.0;
  msg.color.b = 0.0;
  msg.color.a = 1.0;

  msg.scale.x = 0.1;
  msg.scale.y = 0.1;
  msg.scale.z = 0.1;

  msg.pose.position.x = F.translation().x();
  msg.pose.position.y = F.translation().y();
  msg.pose.position.z = F.translation().z();

  Eigen::Quaterniond q(F.rotation());
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();

  msg.points.resize(features.size());

  for (int k = 0; k < features.size(); ++k)
  {
	cv::Point3f f = features[k];

    msg.points[k].x = f.x;
    msg.points[k].y = f.y;
    msg.points[k].z = f.z;
  }

  markers_pub.publish(msg);
}
