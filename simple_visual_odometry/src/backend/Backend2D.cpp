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

#include "backend/Backend2D.h"

using namespace std;
using namespace cv;

Eigen::Affine3d Backend2D::computePose(Features2D& features)
{
	try
	{

		if (state == Initial || state == Lost)
		{
			if (features.size() > minInitialFeatures)
			{
				//Accept first features
				oldFeatures = features;

				//Update state
				state = Initializing;
			}
		}
		else
		{
			if(features.size() < minFeatures)
					throw Backend::no_points_exception();

			Features2Dn featuresOldnorm;
			Features2Dn featuresNewnorm;

			double deltaMean = computeNormalizedFeatures(oldFeatures,
						features, featuresOldnorm, featuresNewnorm);

			if(featuresOldnorm.size() < minFeatures)
				throw Backend::no_points_exception();

			if (sufficientDelta(deltaMean))
			{
				vector<unsigned char> mask;
				Mat C = recoverCameraFromEssential(featuresOldnorm,
							featuresNewnorm, mask);

				Features3Dn&& triangulated = triangulatePoints(featuresOldnorm,
							featuresNewnorm, C, mask);

				double scale = 1.0;

				if (state != Initializing)
				{
					scale = estimateScale(triangulated);
				}

				//Compute transform
				Eigen::Affine3d T = cameraToTransform(C, scale);

				//Compute new pose
				T_WC = T_WC * T;

				//Compute points
				old3DPoints = triangulated;
				old3DPoints.scalePoints(scale);

				//compute Features
				oldFeatures = features;

				//Update state
				state = Tracking;
			}
		}
	}
	catch (Backend::low_parallax_exception& e)
	{

	}
	catch (Backend::no_points_exception& e)
	{
		std::cout << "LOST!" << std::endl;
		old3DPoints = Features3Dn();

		//Update state
		state = Lost;
	}

	return T_WC;

}

Mat Backend2D::recoverCameraFromEssential(Features2Dn& oldFeaturesNorm,
			Features2Dn& newFeaturesNorm, vector<unsigned char>& mask)
{
	vector<Vec2d> points1 = oldFeaturesNorm.getPoints();
	vector<Vec2d> points2 = newFeaturesNorm.getPoints();

	Mat E = findEssentialMat(points1, points2, 1.0, cv::Point2d(0, 0),
				FM_RANSAC, 0.99, 0.5 / Kscale, mask);

	Mat R_e;
	Mat t_e;

	int inliers = countNonZero(mask);

	int newInliers = recoverPose(E, points1, points2, R_e, t_e, mask);

	/*std::cout << "points: " << points1.size() << " ransac inliers: " << inliers
	 << " triang inliers: " << newInliers << std::endl;*/

	if (newInliers < inliers / 2)
		throw Backend::low_parallax_exception();

	Mat C;
	hconcat(R_e, t_e, C);

	return C;

}

Features3Dn Backend2D::triangulatePoints(Features2Dn& oldFeaturesNorm,
			Features2Dn& newFeaturesNorm, cv::Mat C,
			vector<unsigned char>& mask)
{
	Features3Dn triangulated;

	cv::Mat C0 = cv::Mat::eye(3, 4, CV_64FC1);

	cv::Mat points4D(1, oldFeaturesNorm.size(), CV_64FC4);
	cv::triangulatePoints(C0, C, oldFeaturesNorm.getPoints(),
				newFeaturesNorm.getPoints(), points4D);

	for (unsigned int i = 0; i < points4D.cols; i++)
	{
		if (mask[i])
		{
			Vec4d point4d = points4D.col(i);
			point4d = point4d / point4d[3];
			Vec3d point(point4d[0], point4d[1], point4d[2]);
			triangulated.addPoint(point, oldFeaturesNorm.getId(i));
		}
	}

	return triangulated;

}

double Backend2D::estimateScale(Features3Dn& new3DPoints)
{
	vector<double> scaleVector;

	unsigned int count = 0;

	for (unsigned int i = 0; i < new3DPoints.size(); i++)
	{
		for (unsigned int j = i + 1; j < new3DPoints.size(); j++)
		{
			unsigned int id_i = new3DPoints.getId(i);
			unsigned int id_j = new3DPoints.getId(j);

			if (old3DPoints.contains(id_i) && old3DPoints.contains(id_j))
			{
				unsigned int index_i = old3DPoints.getIndex(id_i);
				unsigned int index_j = old3DPoints.getIndex(id_j);

				double num = cv::norm(
							old3DPoints[index_i] - old3DPoints[index_j]);
				double den = cv::norm(new3DPoints[i] - new3DPoints[j]);

				double localScale = num / den;

				if (isfinite(localScale))
					scaleVector.push_back(localScale);

				count++;
			}
		}

	}

	if (count == 0)
	{
		throw Backend::no_points_exception();
	}

	int N = scaleVector.size();

	//std::cout << "N: " << N << std::endl;

	if (N == 0)
		throw Backend::low_parallax_exception();

	return estimateScaleMedian(scaleVector);
}

double Backend2D::estimateScaleMedian(vector<double>& scaleVector)
{
	unsigned int N = scaleVector.size();

	std::sort(scaleVector.begin(), scaleVector.end());

	int index = N / 2;

	if (N % 2 == 0)
	{
		return 0.5 * scaleVector[index] + 0.5 * scaleVector[index - 1];
	}
	else
	{
		return scaleVector[index];
	}

}

double Backend2D::estimateScaleMean(vector<double>& scaleVector)
{
	double scale = 0;

	for (auto localScale : scaleVector)
	{
		scale += localScale;
	}

	return scale / static_cast<double>(scaleVector.size());
}
