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

#include "backend/Backend.h"

using namespace cv;
using namespace std;

Backend::Backend()
{
	state = Initial;
}

Eigen::Affine3d Backend::computePose(Features2D& trackedFeatures,
			Features2D& newFeatures)
{

		prelude();

		try
		{
			switch (state)
			{
				case Initial:
					startup(trackedFeatures, newFeatures);
					break;

				case Initializing:
					initialization(trackedFeatures, newFeatures);
					break;

				case Tracking:
					tracking(trackedFeatures, newFeatures);
					break;

				case Lost:
					recovery(trackedFeatures, newFeatures);
					break;

				default:
					break;

			}
		}
		catch (Backend::low_parallax_exception& e)
		{
			lowParalalxHandler();
		}
		catch (Backend::no_points_exception& e)
		{
			lostExceptionHandler();
		}

		return T_WC;

}

void Backend::lostExceptionHandler()
{
	std::cout << "Lost" << std::endl;

	//Update state
	state = Lost;
}

void Backend::lowParalalxHandler()
{

}

Vec2d Backend::computeNormalizedPoint(Point2f& point)
{
	double fx = K(0, 0);
	double fy = K(1, 1);
	double cx = K(0, 2);
	double cy = K(1, 2);

	return Vec2d((point.x - cx) / fx, (point.y - cy) / fy);
}

double Backend::getCorrespondecesAndDelta(Features2D& oldFeatures,
			Features2D& newFeatures, Features2D& oldCF, Features2D& newCF)
{

	double deltaMean = 0;
	unsigned int N = 0;

	for (unsigned int i = 0; i < newFeatures.size(); i++)
	{
		unsigned int id = newFeatures.getId(i);

		if (oldFeatures.contains(id))
		{
			unsigned int index = oldFeatures.getIndex(id);

			oldCF.addPoint(oldFeatures[index], id);
			newCF.addPoint(newFeatures[i], id);

			deltaMean += norm(oldFeatures[index] - newFeatures[i]);
			N++;
		}
	}

	deltaMean /= N;
	return deltaMean;
}

Features3D Backend::triangulate(Features2D& oldFeatures,
			Features2D& newFeatures, Mat C, Mat C0)
{
	Mat points4D(1, oldFeatures.size(), CV_64FC4);
	Features3D triangulated;

	Mat K = Mat(this->K);
	triangulatePoints(K * C0, K * C, oldFeatures.getPoints(),
				newFeatures.getPoints(), points4D);

	for (unsigned int i = 0; i < points4D.cols; i++)
	{
		Vec4d point4d = points4D.col(i);
		point4d = point4d / point4d[3];
		Point3f point(point4d[0], point4d[1], point4d[2]);
		triangulated.addPoint(point, oldFeatures.getId(i));
	}

	return triangulated;

}

void Backend::recoverCameraFromEssential(Features2D& oldFeatures,
			Features2D& newFeatures, vector<unsigned char>& mask, Mat& C,
			Mat& E)
{
	vector<Point2f> points1 = oldFeatures.getPoints();
	vector<Point2f> points2 = newFeatures.getPoints();

	//Compute essential matrix
	E = findEssentialMat(points1, points2, K, FM_RANSAC, 0.999, 0.5, mask);
	vector<Point2f> points1Corrected;
	vector<Point2f> points2Corrected;

	Mat K = Mat(this->K);
	Mat F = K.t().inv() * E * K.inv();
	correctMatches(F, points1, points2, points1Corrected, points2Corrected);

	Mat R;
	Mat t;

	int inliers = countNonZero(mask);

	int newInliers = recoverPose(E, points1Corrected, points2Corrected, K, R, t,
				mask);

	if (newInliers < inliers / 2)
		throw Backend::low_parallax_exception();

	//compute camera matrix
	hconcat(R, t, C);

	//Update Features
	oldFeatures = Features2D(oldFeatures, points1Corrected, mask);
	newFeatures = Features2D(newFeatures, points2Corrected, mask);

}

cv::Mat Backend::computeEssential(const Eigen::Affine3d& T1,
			const Eigen::Affine3d& T2)
{
	Eigen::Affine3d T_CW = T2.inverse() * T1;
	Eigen::Vector3d t = T_CW.translation();
	Eigen::Matrix3d R = T_CW.rotation();

	Mat tx;
	tx = (Mat_<double>(3, 3) << 0, -t(2), t(1), //
	t(2), 0, -t(0), //
	-t(1), t(0), 0);

	Mat Rx = (Mat_<double>(3, 3) << R(0, 0), R(0, 1), R(0, 2), //
	R(1, 0), R(1, 1), R(1, 2), //
	R(2, 0), R(2, 1), R(2, 2));

	return tx * Rx;

}

Eigen::Affine3d Backend::cameraToTransform(const Mat& C, double scale)
{
	Eigen::Matrix4d Tm;
	Tm << //
				C.at<double>(0, 0), C.at<double>(0, 1), C.at<double>(0, 2), scale
				* C.at<double>(0, 3), //
	C.at<double>(1, 0), C.at<double>(1, 1), C.at<double>(1, 2), scale
				* C.at<double>(1, 3), //
	C.at<double>(2, 0), C.at<double>(2, 1), C.at<double>(2, 2), scale
				* C.at<double>(2, 3), //
	0.0, 0.0, 0.0, 1.0;

	return Eigen::Affine3d(Tm).inverse();
}

Mat Backend::transformToCamera(const Eigen::Affine3d& T)
{
	Eigen::Affine3d Tinv = T.inverse();

	Eigen::Matrix3d R = Tinv.rotation();
	Eigen::Vector3d t = Tinv.translation();

	Mat C = (Mat_<double>(3, 4) <<  //
				R(0, 0), R(0, 1), R(0, 2), t(0), //
	R(1, 0), R(1, 1), R(1, 2), t(1), //
	R(2, 0), R(2, 1), R(2, 2), t(2));

	return C;
}

bool Backend::sufficientDelta(double deltaFeatures)
{
	switch (state)
	{
		case Initial:
		case Lost:
			return deltaFeatures > 20.0;

		default:
			return deltaFeatures > 1.0;
	}
}

