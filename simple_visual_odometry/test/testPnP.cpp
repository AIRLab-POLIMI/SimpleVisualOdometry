/*
 * simple_visual_odometry,
 *
 *
 * Copyright (C) 2016 Davide Tateo
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

#include <opencv2/opencv.hpp>
#include <iostream>


int main(int argc, char *argv[])
{
	//Create a random 3D scene
	std::vector<cv::Point3f> points3D;


	unsigned int n = 4;

	for(int i = 0; i < n; i++)
	{
		cv::Mat_<float> point3D(1, 3);
		cv::randu(point3D, cv::Scalar(-5.0, -5.0, 1.0), cv::Scalar(5.0, 5.0, 10.0));

		cv::Point3d p3(point3D(0, 0), point3D(0, 1), point3D(0, 2));

		points3D.push_back(p3);
	}


	//Compute 2 camera matrices
	cv::Matx34d C1 = cv::Matx34d::eye();
	cv::Matx34d C2 = cv::Matx34d::eye();
	cv::Mat K = cv::Mat::eye(3, 3, CV_64F);

	C2(2, 3) = 0.1;

	//Compute points projection
	std::vector<cv::Point2f> points;

	for(int i = 0; i < points3D.size(); i++)
	{
		cv::Vec4d hpt3;

		hpt3[0] = points3D[i].x;
		hpt3[1] = points3D[i].y;
		hpt3[2] = points3D[i].z;
		hpt3[3] = 1.0;

		cv::Vec3d hpt = C2*hpt3;

		hpt /= hpt[2];

		cv::Point2f p(hpt[0], hpt[1]);

		points.push_back(p);
	}


	//Print
	std::cout << C1 << std::endl;
	std::cout << C2 << std::endl;
	std::cout << points3D << std::endl;

	//Recover essential
	cv::Mat rvec;
	cv::Mat tvec;

	cv::solvePnP(points3D, points, K, cv::Mat(), rvec, tvec, false, CV_ITERATIVE);

	cv::Mat R;
	cv::Mat t;

	Rodrigues(rvec, R);
	t = tvec;

	std::cout << "R: " << R << std::endl;
	std::cout << "t: " << t << std::endl;


}
