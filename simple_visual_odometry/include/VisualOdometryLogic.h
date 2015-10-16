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

#ifndef EXTRACTOR_H_
#define EXTRACTOR_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <tf/transform_broadcaster.h>

#include <opencv2/opencv.hpp>

#include "VisualFrontend.h"

class VisualOdometryLogic
{
public:
	VisualOdometryLogic(std::string imageTopic, ros::NodeHandle& n);
	virtual void handleImage(const sensor_msgs::ImageConstPtr& msg,
				const sensor_msgs::CameraInfoConstPtr& info_msg);
	bool getImage(const sensor_msgs::ImageConstPtr& msg,
				cv_bridge::CvImagePtr& cv_ptr,
				cv_bridge::CvImagePtr& cv_ptr_color);

	virtual ~VisualOdometryLogic();

protected:
	void publishFeatures(const std::string& frame_id, ros::Time stamp);
	void trackPose(const sensor_msgs::CameraInfoConstPtr& info_msg);
	void display(cv_bridge::CvImagePtr cv_ptr);

private:
	void drawFeatures(cv::Mat& frame, Features2D& features,
				cv::Scalar colorMatched, cv::Scalar colorNew);

private:
	//Ros management
	image_transport::ImageTransport it;
	image_transport::CameraSubscriber imageSubscriber;

	//Pose Tracking
	tf::Transform T;
	tf::TransformBroadcaster tfBroadcaster;

	//Visual Frontend
	VisualFrontend frontend;

	//debug display
	std::string src_window;
};

#endif /* EXTRACTOR_H_ */
