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

#ifndef INCLUDE_BACKEND_H_
#define INCLUDE_BACKEND_H_

#include "five-point.hpp"
#include "core/Features.h"

#include <Eigen/Geometry>

class Backend
{
public:
	enum State
	{
		Initial, Initializing, Tracking, Lost
	};

	class low_parallax_exception: public Exception
	{

	};

	class no_points_exception: public Exception
	{

	};

public:
	Backend();

	inline void setCameraPose(Eigen::Affine3d& T_WC)
	{
		this->T_WC = T_WC;
	}

	virtual Eigen::Affine3d computePose(Features2D& trackedFeatures) = 0;

	virtual Features3Dn getFeatures() const = 0;

	inline void setK(const cv::Matx33d& K)
	{
		Kinv = K.inv();
		double fx = K(0, 0);
		double fy = K(1, 1);
		Kscale = (fx + fy) / 2;
	}

	State getState() const
	{
		return state;
	}

	virtual ~Backend()
	{

	}

protected:
	inline bool sufficientDelta(double deltaFeatures)
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

	inline Eigen::Affine3d cameraToTransform(const cv::Mat& C, double scale =
				1.0)
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

	double computeNormalizedFeatures(Features2D& oldFeatures,
				Features2D& newFeatures, Features2Dn& oldFeaturesNorm,
				Features2Dn& newFeaturesNorm);

	cv::Vec2d computeNormalizedPoint(cv::Point2f& point);

protected:
	cv::Matx33d Kinv;
	double Kscale;

	//Tracking status
	State state;

	//Pose data
	Eigen::Affine3d T_WC;
};

#endif /* INCLUDE_BACKEND_H_ */
