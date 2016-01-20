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
#include <Eigen/Geometry>
#include "five-point.hpp"

using namespace cv;

int main(int argc, char *argv[])
{
	// Set up a movement
	Eigen::Matrix4d M1;
	M1 << 1, 0, 0, 0, //
	0, 1, 0, 0, //
	0, 0, 1, 0, //
	0, 0, 0, 1;

	Eigen::Matrix4d M2;
	double theta = M_PI/2;
	M2 << cos(theta), -sin(theta), 0, 0, //
	sin(theta), cos(theta), 0, 0, //
	0, 0, 1, 1, //
	0, 0, 0, 1;

	Eigen::Affine3d T1(M1);
	Eigen::Affine3d T2(M2);

	// Compute essential
	Eigen::Affine3d T_CW = T2.inverse() * T1;
	Eigen::Vector3d t = T_CW.translation();
	Eigen::Matrix3d R = T_CW.rotation();

	std::cout << "delta transform" << std::endl;
	std::cout << t << std::endl;
	std::cout << R << std::endl;

	Mat tx;
	tx = (Mat_<double>(3, 3) << 0, -t(2), t(1), //
	t(2), 0, -t(0), //
	-t(1), t(0), 0);

	Mat Rx = (Mat_<double>(3, 3) << R(0, 0), R(0, 1), R(0, 2), //
	R(1, 0), R(1, 1), R(1, 2), //
	R(2, 0), R(2, 1), R(2, 2));

	Mat E = tx * Rx;

	std::cout << "E" << std::endl << E << std::endl;

	// recover essential
	Mat R1;
	Mat R2;
	Mat tcv;
	decomposeEssentialMat(E, R1, R2, tcv);

	std::cout << "decomposed matrix" << std::endl;
	std::cout << "R1" << std::endl << R1 << std::endl;
	std::cout << "R2" << std::endl << R2 << std::endl;
	std::cout << "t" << std::endl << tcv << std::endl;

}
