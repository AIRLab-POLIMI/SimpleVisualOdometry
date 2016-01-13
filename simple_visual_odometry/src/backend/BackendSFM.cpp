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

#include "backend/BackendSFM.h"

BackendSFM::BackendSFM(const Eigen::Affine3d& F)
{
	Fpoints = F;
}

Eigen::Affine3d BackendSFM::computePose(Features2D& features)
{
	try
	{
		switch (state)
		{
			case Initial:
			case Lost:
			{
				if (features.size() > minInitialFeatures)
				{
					//Accept first features
					oldFeatures = features;

					//Update state
					state = Initializing;
				}

				break;
			}

			case Initializing:
			{
				if (features.size() < minFeatures)
					throw Backend::no_points_exception();

				Features2Dn featuresOldnorm;
				Features2Dn featuresNewnorm;

				double deltaMean = computeNormalizedFeatures(oldFeatures,
							features, featuresOldnorm, featuresNewnorm);

				if (featuresOldnorm.size() < minFeatures)
					throw Backend::no_points_exception();

				if (sufficientDelta(deltaMean))
				{
					vector<unsigned char> mask;
					Mat C = recoverCameraFromEssential(featuresOldnorm,
								featuresNewnorm, mask);

					Features3Dn&& triangulated = triangulate(
								featuresOldnorm, featuresNewnorm, mask, C);

					//Save points
					old3DPoints = triangulated;
					new3DPoints = triangulated;

					//Compute transform
					Eigen::Affine3d T = cameraToTransform(C);

					//Compute new pose
					T_WC = T_WC * T;
				}

				//Update state
				state = Tracking;
				break;
			}

			case Tracking:
			{


				break;
			}

			default:
				break;

		}
	}
	catch (Backend::no_points_exception& e)
	{
		std::cout << "LOST!" << std::endl;

		//Update state
		state = Lost;
	}

	return T_WC;

}

Features3Dn BackendSFM::getFeatures() const
{
	return new3DPoints;
}
