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

#include <opencv2/opencv.hpp>

#include <image_geometry/pinhole_camera_model.h>

#include <tf2_eigen/tf2_eigen.h>

#include "backend/Backend2D.h"

namespace enc = sensor_msgs::image_encodings;
using namespace cv;
using namespace std;

VisualOdometryLogic::VisualOdometryLogic(string imageTopic, ros::NodeHandle& n) :
			it(n), trajectoryPublisher("camera"), gtTrajectoryPublisher("gt"),
			tfListener(tfBuffer)
{
	imageSubscriber = it.subscribeCamera(imageTopic + "image_rect_color", 1,
				&VisualOdometryLogic::handleImage, this);

	//Init rotation camera/robot
	Eigen::Quaterniond q_CR(0.5, 0.5, -0.5, 0.5);
	Eigen::Quaterniond q_RC(-0.5, 0.5, -0.5, 0.5);

	T_CR.setIdentity();
	T_CR.rotate(q_CR);

	T_RC.setIdentity();
	T_RC.rotate(q_RC);

	//Init pose
	T_WC = config.T_WR * T_RC;
	Tgt = config.T_WR;

	//debug window
	src_window = "Extracted Features";
	namedWindow(src_window, CV_WINDOW_AUTOSIZE);

	//Init Backend
	backend = new Backend2D();
	backend->setCameraPose(T_WC);

}

void VisualOdometryLogic::handleImage(const sensor_msgs::ImageConstPtr& msg,
			const sensor_msgs::CameraInfoConstPtr& info_msg)
{
	cv_bridge::CvImagePtr cv_ptr, cv_ptr_color;

	//if I can get the image from message...
	if (getImage(msg, cv_ptr, cv_ptr_color))
	{
		Mat image = cv_ptr->image;

		Features2D trackedFeatures;
		frontend.trackAndExtract(image, trackedFeatures);

		//Track pose
		trackPose(info_msg, trackedFeatures);

		//publish features
		//publishFeatures(msg->header.frame_id, msg->header.stamp);

		//display image
		display(cv_ptr_color);
	}
}

bool VisualOdometryLogic::getImage(const sensor_msgs::ImageConstPtr& msg,
			cv_bridge::CvImagePtr& cv_ptr, cv_bridge::CvImagePtr& cv_ptr_color)
{
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, enc::MONO8);
		cv_ptr_color = cv_bridge::toCvCopy(msg, enc::BGR8);

		return true;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return false;
	}
}

VisualOdometryLogic::~VisualOdometryLogic()
{

}

void VisualOdometryLogic::display(cv_bridge::CvImagePtr cv_ptr)
{
	//Print debug image
	Mat coloredImage;
	coloredImage = cv_ptr->image;
	drawFeatures(coloredImage, frontend.getCurrentFeatures(), Scalar(255, 0, 0),
				Scalar(0, 255, 0));
	imshow(src_window, coloredImage);
	waitKey(1);
}

void VisualOdometryLogic::publishFeatures(const string& frame_id,
			ros::Time stamp)
{

}

void VisualOdometryLogic::trackPose(
			const sensor_msgs::CameraInfoConstPtr& info_msg,
			Features2D& trackedFeatures)
{
	image_geometry::PinholeCameraModel cameraModel;
	cameraModel.fromCameraInfo(info_msg);

	//Compute Camera Pose
	Matx34d P = cameraModel.fullProjectionMatrix();
	Matx33d K = P.get_minor<3, 3>(0, 0);
	backend->setK(K);
	T_WC = backend->computePose(trackedFeatures);

	Eigen::Affine3d T_WR = T_WC * T_CR;

	//Publish features
	featurespublisher.publishFeatureMarkers(backend->getFeatures());

	//Send Transform
	publishEigenTransform(info_msg->header.stamp, "world",
				info_msg->header.frame_id, T_WR);
	publishEigenTransform(info_msg->header.stamp, info_msg->header.frame_id,
				"camera_link", T_RC);

	//Send trajectory
	trajectoryPublisher.publishPath(T_WR, info_msg->header.stamp);

	//publish gt trajectory
	try
	{
		Tgt = getEigenTransform(info_msg->header.stamp, "world",
					"prosilica/camera_link_gt");
		gtTrajectoryPublisher.publishPath(Tgt, info_msg->header.stamp);
	}
	catch (tf2::TransformException &ex)
	{

	}
}

void VisualOdometryLogic::publishEigenTransform(const ros::Time& stamp,
			const std::string& parent, const std::string& frame_id,
			const Eigen::Affine3d& T)
{
	geometry_msgs::TransformStamped transformStamped;

	transformStamped.header.stamp = stamp;
	transformStamped.header.frame_id = parent;
	transformStamped.child_frame_id = frame_id;

	transformStamped.transform.translation.x = T.translation().x();
	transformStamped.transform.translation.y = T.translation().y();
	transformStamped.transform.translation.z = T.translation().z();

	Eigen::Quaterniond q(T.rotation());
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	tfBroadcaster.sendTransform(transformStamped);
}

Eigen::Affine3d VisualOdometryLogic::getEigenTransform(const ros::Time& stamp,
			const std::string& parent, const std::string& frame_id)
{
	geometry_msgs::TransformStamped transformStamped;
	transformStamped = tfBuffer.lookupTransform(parent, frame_id, stamp);

	return tf2::transformToEigen(transformStamped);
}

void VisualOdometryLogic::drawFeatures(Mat& frame, Features2D& features,
			cv::Scalar colorMatched, cv::Scalar colorNew)
{

	static unsigned int max_id = 0;
	unsigned int new_max_id = 0;

	for (int j = 0; j < features.size(); j++)
	{
		cv::Scalar drawColor =
					(features.getId(j) > max_id) ? colorNew : colorMatched;
		circle(frame, features[j], 3, drawColor);

		new_max_id = std::max(new_max_id, features.getId(j));
	}

	max_id = new_max_id;
}
