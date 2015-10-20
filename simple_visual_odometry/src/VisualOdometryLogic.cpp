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

	T = config.T_W_CAMERA;


	//debug window
	src_window = "Extracted Features";
	namedWindow(src_window, CV_WINDOW_AUTOSIZE);

	//Init Backend
	backend = new Backend2D();

}

void VisualOdometryLogic::handleImage(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
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

void VisualOdometryLogic::display(cv_bridge::CvImagePtr cv_ptr)
{
	//Print debug image
	Mat coloredImage;
	coloredImage = cv_ptr->image;
	drawFeatures(coloredImage, frontend.getCurrentFeatures(),
				Scalar(255, 0, 0), Scalar(0, 255, 0));
	imshow(src_window, coloredImage);
	waitKey(1);
}

void VisualOdometryLogic::publishFeatures(const string& frame_id, ros::Time stamp)
{


}

void VisualOdometryLogic::trackPose(const sensor_msgs::CameraInfoConstPtr& info_msg, Features2D& trackedFeatures)
{
	image_geometry::PinholeCameraModel cameraModel;
	cameraModel.fromCameraInfo(info_msg);

	//Compute Camera translation
	Matx34d P = cameraModel.fullProjectionMatrix();
	Matx33d K = P.get_minor<3, 3>(0, 0);
	backend->setK(K);
	backend->computeTransformation(trackedFeatures, frontend.getCurrentFeatures());


	if(backend->transformationComputed())
	{
		Matx33d R = backend->getRotation();
		Vec3d t = backend->getTranslation();


		//Compute new camera pose
		tf::Vector3 t_tf(t[0], t[1], t[2]);

		tf::Matrix3x3 R_tf(R(0, 0), R(0, 1), R(0, 2),
						   R(1, 0), R(1, 1), R(1, 2),
						   R(2, 0), R(2, 1), R(2, 2));

		tf::Transform T_new(R_tf, t_tf);

		//std::cout << t << std::endl;
		//std::cout << R << std::endl;

		T = T*T_new;

	}

	//Send Transform
    tfBroadcaster.sendTransform(tf::StampedTransform(T, ros::Time::now(), "world", info_msg->header.frame_id));


}

void VisualOdometryLogic::drawFeatures(Mat& frame, Features2D& features,
			cv::Scalar colorMatched, cv::Scalar colorNew)
{

	static unsigned int max_id = 0;
	unsigned int new_max_id = 0;

	for (int j = 0; j < features.size(); j++)
	{
		cv::Scalar drawColor = (features.getId(j) > max_id) ?
								colorNew : colorMatched;
		circle(frame, features[j], 3, drawColor);

		new_max_id = std::max(new_max_id, features.getId(j));
	}

	max_id = new_max_id;
}

VisualOdometryLogic::~VisualOdometryLogic()
{

}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "roamros_extractor");

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
