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

#include "VisualFrontend.h"

#include <ros/ros.h>

using namespace std;
using namespace cv;

VisualFrontend::VisualFrontend()
{
	//Initialise detector
	std::string detectorType = "Feature2D.BRISK";

	detector = Algorithm::create<FeatureDetector>(detectorType);
	detector->set("thres", thresholdExtraction);

	//Initialize ID
	newId = 0;
}

void VisualFrontend::trackAndExtract(cv::Mat& im_gray, Features2D& newPoints)
{

	if (oldPoints.size() > 0)
	{
		//Track prevoius points with optical flow
		track(im_gray, newPoints);

		//Save tracked points
		oldPoints = newPoints;
	}

	//Extract new points
	extract(im_gray);

	//save old image
	im_prev = im_gray;
}

void VisualFrontend::extract(Mat& im_gray)
{
	vector<KeyPoint> newPoints;
	detector->detect(im_gray, newPoints);

	//Prepare grid
	grid.setImageSize(im_gray.cols, im_gray.rows);
	for (Point2f& oldPoint : oldPoints)
	{
		grid.addPoint(oldPoint);
	}

	for (auto point : newPoints)
	{
		if (grid.isNewFeature(point.pt))
		{
			oldPoints.addPoint(point.pt, newId);
			newId++;
		}
	}

	grid.resetGrid();
}

void VisualFrontend::track(Mat& im_gray, Features2D& newPoints)
{
	vector<unsigned char> status;
	vector<unsigned char> status_back;

	vector<Point2f> pts_back;
	vector<Point2f> points = oldPoints.getPoints();
	vector<Point2f> nextPts;

	vector<float> err;
	vector<float> err_back;
	vector<float> fb_err;

	//Calculate forward optical flow for prev_location
	calcOpticalFlowPyrLK(im_prev, im_gray, points, nextPts, status, err);
	//Calculate backward optical flow for prev_location
	calcOpticalFlowPyrLK(im_gray, im_prev, nextPts, pts_back, status_back,
				err_back);

	//Calculate forward-backward error
	for (int i = 0; i < points.size(); i++)
	{
		fb_err.push_back(norm(pts_back[i] - points[i]));
	}

	//Set status depending on fb_err and lk error
	for (size_t i = 0; i < status.size(); i++)
	{
		ROS_INFO_STREAM("FB ERROR      : " << fb_err[i]);
		ROS_INFO_STREAM("over threshold: " << (fb_err[i] <= thresholdFBError));
		ROS_INFO_STREAM("status        : " << (fb_err[i] <= thresholdFBError) && status[i]);
		status[i] = (fb_err[i] <= thresholdFBError) && status[i];
	}

	newPoints = Features2D(oldPoints, nextPts, status);

}
