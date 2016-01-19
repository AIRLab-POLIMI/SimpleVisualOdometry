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

Eigen::Affine3d Backend2D::computePose(Features2D& trackedFeatures,
			Features2D& newFeatures)
{
	//Points frame is last frame
	Fpoints = T_WC;

	try
	{

		switch (state)
		{
			case Initial:
			case Lost:
			{
				if (newFeatures.size() + trackedFeatures.size()
							> minInitialFeatures)
				{
					//Accept first features
					oldFeatures = trackedFeatures;
					oldFeatures.addPoints(newFeatures);

					//Update state
					state = Initializing;
					std::cout << "Initializing" << std::endl;
				}
			}

				break;

			case Initializing:
			case Tracking:
			{
				if (trackedFeatures.size() < minFeatures)
					throw Backend::no_points_exception();

				Features2D oldCorrespondences;
				Features2D newCorrespondences;

				double deltaMean = getCorrespondecesAndDelta(oldFeatures,
							trackedFeatures, oldCorrespondences,
							newCorrespondences);

				if (oldCorrespondences.size() < minFeatures)
					throw Backend::no_points_exception();

				if (sufficientDelta(deltaMean))
				{
					vector<unsigned char> mask;
					cv::Mat C;
					cv::Mat E;
					recoverCameraFromEssential(oldCorrespondences,
								newCorrespondences, mask, C, E);

					Features3D&& triangulated = triangulate(oldCorrespondences,
								newCorrespondences, C);

					double scale = 1.0;

					if (state != Initializing)
					{
						scale = estimateScale(triangulated);
					}

					//Compute transform
					Eigen::Affine3d T = cameraToTransform(C, scale);

					//Compute new pose
					T_WC = T_WC * T;

					//Save 3d points
					old3DPoints = triangulated;
					old3DPoints.scalePoints(scale);

					//Save current features
					oldFeatures = trackedFeatures;
					oldFeatures.addPoints(newFeatures);

					//Update state
					state = Tracking;
				}

			}

				break;

			default:
				break;
		}

	}
	catch (Backend::low_parallax_exception& e)
	{

	}
	catch (Backend::no_points_exception& e)
	{
		std::cout << "Lost" << std::endl;
		old3DPoints = Features3D();

		//Update state
		state = Lost;
	}

	return T_WC;

}

Features3D Backend2D::getFeatures() const
{
	return old3DPoints;
}

double Backend2D::estimateScale(Features3D& new3DPoints)
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
