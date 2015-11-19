/*
 * test.cpp
 *
 *  Created on: 04 set 2015
 *      Author: dave
 */

#include "five-point.hpp"
#include <iostream>


int main(int argc, char *argv[])
{
	//Create a random 3D scene
	cv::Mat points3D(1, 16, CV_64FC4);
	cv::randu(points3D, cv::Scalar(-5.0, -5.0, 1.0, 1.0), cv::Scalar(5.0, 5.0, 10.0, 1.0 ));


	//Compute 2 camera matrices
	cv::Matx34d C1 = cv::Matx34d::eye();
	cv::Matx34d C2 = cv::Matx34d::eye();

	C2(2, 3) = 1;

	//Compute points projection
	std::vector<cv::Vec2d> points1;
	std::vector<cv::Vec2d> points2;

	for(int i = 0; i < points3D.cols; i++)
	{
		cv::Vec3d hpt1 = C1*points3D.at<cv::Vec4d>(0, i);
		cv::Vec3d hpt2 = C2*points3D.at<cv::Vec4d>(0, i);

		hpt1 /= hpt1[2];
		hpt2 /= hpt2[2];

		cv::Vec2d p1(hpt1[0], hpt1[1]);
		cv::Vec2d p2(hpt2[0], hpt2[1]);

		points1.push_back(p1);
		points2.push_back(p2);
	}


	//Print
	std::cout << C1 << std::endl;
	std::cout << C2 << std::endl;
	std::cout << points3D << std::endl;

	//Recover essential
	cv::Mat tmp = findEssentialMat(points1, points2, 1, cv::Point(0, 0), 8, 0.90, 0.001);
	cv::Matx33d E = tmp;
	cv::Matx33d F = cv::findFundamentalMat(points1, points2, FM_RANSAC, 0.001, 0.9);

	std::cout << "E: " << E << std::endl;
	std::cout << "F: " << F << std::endl;


	double errorSumF = 0;
	double errorSumE = 0;

	for(int i = 0; i < points1.size(); i++)
	{
		cv::Vec2d p1 = points1[i];
		cv::Vec2d p2 = points2[i];

		cv::Vec3d hp1(p1[0], p1[1], 1);
		cv::Vec3d hp2(p2[0], p2[1], 1);

		cv::Mat r1 =  (cv::Mat)(hp2.t()*E*hp1);
		cv::Mat r2 =  (cv::Mat)(hp2.t()*F*hp1);

		errorSumE += std::pow(r1.at<double>(0), 2);
		errorSumF += std::pow(r2.at<double>(0), 2);
	}

	std::cout << "J(E): " << errorSumE/points1.size() << std::endl;
	std::cout << "J(F): " << errorSumF/points1.size() << std::endl;

	cv::Mat R;
	cv::Mat t;
	recoverPose((Mat)F, points1, points2, R, t);

	std::cout << "R: " << R << std::endl;
	std::cout << "t: " << t << std::endl;


	cv::Mat R5;
	cv::Mat t5;
	recoverPose((Mat)E, points1, points2, R5, t5);

	std::cout << "R5: " << R5 << std::endl;
	std::cout << "t5: " << t5 << std::endl;


}

