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

	Eigen::Affine3d computePose(Features2D& trackedFeatures, Features2D& newFeatures);

	virtual Features3D getFeatures() const = 0;

	inline void setCameraPose(Eigen::Affine3d& T_WC)
	{
		this->T_WC = T_WC;
	}

	inline void setK(const cv::Matx33d& K)
	{
		this->K = K;
	}

	inline State getState() const
	{
		return state;
	}

	inline Eigen::Affine3d getPointsFrame() const
	{
		return Fpoints;
	}

	virtual ~Backend()
	{

	}

protected:
	virtual void prelude() = 0;

	virtual void startup(Features2D& trackedFeatures, Features2D& newFeatures) = 0;
	virtual void initialization(Features2D& trackedFeatures,
				Features2D& newFeatures) = 0;
	virtual void tracking(Features2D& trackedFeatures, Features2D& newFeatures) = 0;
	virtual void recovery(Features2D& trackedFeatures, Features2D& newFeatures) = 0;

	virtual void lostExceptionHandler();
	virtual void lowParalalxHandler();

protected:
	bool sufficientDelta(double deltaFeatures);

	Eigen::Affine3d cameraToTransform(const cv::Mat& C, double scale = 1.0);
	cv::Mat transformToCamera(const Eigen::Affine3d& T);

	double getCorrespondecesAndDelta(Features2D& oldFeatures,
				Features2D& newFeatures, Features2D& oldCF, Features2D& newCF);

	cv::Vec2d computeNormalizedPoint(cv::Point2f& point);

	Features3D triangulate(Features2D& oldFeatures, Features2D& newFeatures,
				cv::Mat C, cv::Mat C0 = cv::Mat::eye(3, 4, CV_64FC1));

	void recoverCameraFromEssential(Features2D& oldFeaturesNorm,
				Features2D& newFeaturesNorm, std::vector<unsigned char>& mask,
				cv::Mat& C, cv::Mat& E);

	cv::Mat computeEssential(const Eigen::Affine3d& T1,
				const Eigen::Affine3d& T2);

protected:
	//Camera matrix
	cv::Matx33d K;

	//Tracking status
	State state;

	//Pose data
	Eigen::Affine3d T_WC;

	//Points reference Frame
	Eigen::Affine3d Fpoints;
};

#endif /* INCLUDE_BACKEND_H_ */
