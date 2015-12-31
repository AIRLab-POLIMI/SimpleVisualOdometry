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

#include "FeaturesPublisher.h"
#include <visualization_msgs/Marker.h>

FeaturesPublisher::FeaturesPublisher()
{
	ros::NodeHandle n;

	markers_pub = n.advertise<visualization_msgs::Marker>(
	      "/visualization/features", 1);

}

void FeaturesPublisher::publishFeatureMarkers(Features3Dn& features)
{
  static unsigned int id = 0;
  visualization_msgs::Marker msg;

  msg.header.stamp = ros::Time::now();
  //msg.header.frame_id = "/world";
  msg.header.frame_id = "/prosilica_camera";
  msg.type = visualization_msgs::Marker::CUBE_LIST;
  msg.frame_locked = false;
  msg.ns = "vo";
  msg.id = id++;
  msg.action = visualization_msgs::Marker::ADD;

  msg.color.r = 1.0;
  msg.color.g = 0.0;
  msg.color.b = 0.0;
  msg.color.a = 1.0;

  msg.scale.x = 0.01;
  msg.scale.y = 0.01;
  msg.scale.z = 0.01;

  msg.pose.position.x = 0.0;
  msg.pose.position.y = 0.0;
  msg.pose.position.z = 0.0;

  msg.pose.orientation.x =  0.5;
  msg.pose.orientation.y = -0.5;
  msg.pose.orientation.z =  0.5;
  msg.pose.orientation.w = -0.5;

  msg.points.resize(features.size());

  for (int k = 0; k < features.size(); ++k)
  {
	cv::Vec3d f = features[k];

    msg.points[k].x = f(0);
    msg.points[k].y = f(1);
    msg.points[k].z = f(2);
  }

  markers_pub.publish(msg);
}
