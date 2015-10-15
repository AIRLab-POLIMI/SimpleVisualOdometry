/*
 * roamfree_feature_extractor,
 *
 *
 * Copyright (C) 2014 Davide Tateo
 * Versione 1.0
 *
 * This file is part of roamros_extractor.
 *
 * roamros_extractor is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * roamros_extractor is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with roamros_extractor.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "VisualOdometryLogic.h"

#include <opencv2/opencv.hpp>

#include <image_geometry/pinhole_camera_model.h>

#include <tf/tf.h>

namespace enc = sensor_msgs::image_encodings;
using namespace cv;
using namespace std;

VisualOdometryLogic::VisualOdometryLogic(string imageTopic, ros::NodeHandle& n) :
			it(n)
{
	imageSubscriber = it.subscribeCamera(imageTopic + "image_rect_color", 1,
				&VisualOdometryLogic::handleImage, this);

	//Init transform
	tf::Vector3 tn(0, 0, 0);
	tf::Matrix3x3 Rn;
	Rn.setIdentity();

	T = tf::Transform(Rn, tn);


	//debug window
	src_window = "Extracted Features";
	namedWindow(src_window, CV_WINDOW_AUTOSIZE);
}

void VisualOdometryLogic::handleImage(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
	cv_bridge::CvImagePtr cv_ptr, cv_ptr_color;

	//if I can get the image from message...
	if (getImage(msg, cv_ptr, cv_ptr_color))
	{
		Mat image = cv_ptr->image;

		//featureManager.trackAndFindNew(image);

		//Track pose
		//trackPose(info_msg);

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

void VisualOdometryLogic::display(cv_bridge::CvImagePtr cv_ptr)
{
	//Print debug image
	Mat coloredImage;
	coloredImage = cv_ptr->image;
	//drawKeypoints(coloredImage, featureManager.getKeyPoints(),
	//			Scalar(255, 0, 0), Scalar(0, 255, 0));
	imshow(src_window, coloredImage);
	waitKey(1);
}

void VisualOdometryLogic::publishFeatures(const string& frame_id, ros::Time stamp)
{


}

void VisualOdometryLogic::trackPose(const sensor_msgs::CameraInfoConstPtr& info_msg)
{


}

void VisualOdometryLogic::drawKeypoints(Mat& frame, const vector<KeyPoint>& keypoints,
			cv::Scalar colorMatched, cv::Scalar colorNew)
{

	static uint64_t max_id = 0;
	uint64_t new_max_id = 0;

	for (int j = 0; j < keypoints.size(); j++)
	{
	/*	cv::Scalar drawColor =
					(featureManager.getId(j) > max_id) ?
								colorNew : colorMatched;

		if (featureManager.getId(j) <= max_id || !hideNew)
			circle(frame, keypoints[j].pt, 3, drawColor);

		new_max_id = std::max(new_max_id, featureManager.getId(j));*/
	}

	max_id = new_max_id;
}

VisualOdometryLogic::~VisualOdometryLogic()
{

}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "roamros_extractor");

	if (argc < 4)
	{
		ROS_FATAL("Three arguments needed: camera_source, keyframe_feature_percentage, force_extraction");
		return -1;
	}

	ros::NodeHandle n;
	string imageTopic = argv[1];
	double keyFramePercentage = std::atof(argv[2]);
	bool forceExtraction = (std::atoi(argv[3]) == 0) ? false : true;

	VisualOdometryLogic logic(imageTopic, n);

	ROS_INFO("Feature extraction node started");

	ros::spin();
}
