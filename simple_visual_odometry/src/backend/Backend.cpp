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
	Kscale = 0;
	state = Initial;
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
			Features2D& newFeatures, vector<unsigned char>& mask, Mat C, Mat C0)
{
	Features3D triangulated;

	Mat points4D(1, oldFeatures.size(), CV_64FC4);
	triangulatePoints(Mat(K) * C0, Mat(K) * C, oldFeatures.getPoints(),
				newFeatures.getPoints(), points4D);

	for (unsigned int i = 0; i < points4D.cols; i++)
	{
		if (mask[i])
		{
			Vec4d point4d = points4D.col(i);
			point4d = point4d / point4d[3];
			Point3f point(point4d[0], point4d[1], point4d[2]);
			triangulated.addPoint(point, oldFeatures.getId(i));
		}
	}

	return triangulated;

}

Mat Backend::recoverCameraFromEssential(Features2D& oldFeaturesNorm,
			Features2D& newFeaturesNorm, vector<unsigned char>& mask)
{
	vector<Point2f> points1 = oldFeaturesNorm.getPoints();
	vector<Point2f> points2 = newFeaturesNorm.getPoints();

	Mat E = findEssentialMat(points1, points2, K, FM_RANSAC, 0.999, 0.5, mask);

	Mat R;
	Mat t;

	int inliers = countNonZero(mask);

	int newInliers = recoverPose(E, points1, points2, K, R, t, mask);

	if (newInliers < inliers / 2)
		throw Backend::low_parallax_exception();

	Mat C;
	hconcat(R, t, C);

	return C;

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

