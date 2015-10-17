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

#include "opencv3.h"
#include "Features.h"

class Backend
{
public:
	Backend();

	virtual void computeTransformation(Features2D& features) = 0;

	inline bool transformationComputed()
	{
		bool val = computed;
		computed = false;
		return val;
	}

	inline cv::Vec3d getTranslation()
	{
		return t;
	}

	inline cv::Matx33d getRotation()
	{
		return R;
	}

	inline void setK(const cv::Matx33d& K)
	{
		Kinv = K.inv();
	}

	virtual ~Backend()
	{

	}

protected:
	bool computed;
	cv::Vec3d t;
	cv::Matx33d R;

	cv::Matx33d Kinv;

};

class Backend2D: public Backend
{
public:
	void computeTransformation(Features2D& features) override;

private:
	double computeNormalizedFeatures(Features2D& oldFeatures,
				Features2D& newFeatures, Features2Dn& oldFeaturesNorm,
				Features2Dn& newFeaturesNorm);

	cv::Mat recoverCameraFromEssential(Features2Dn& oldFeaturesNorm, Features2Dn& newFeaturesNorm,
				std::vector<unsigned char>& mask);

	Features3Dn triangulatePoints(Features2Dn& oldFeaturesNorm,
				Features2Dn& newFeaturesNorm, cv::Mat C, std::vector<unsigned char>& mask);

	cv::Vec2d computeNormalizedPoint(cv::Point2f& point);

	double estimateScale(Features3Dn& new3DPoints);

	double estimateScaleMedian(std::vector<double>& scaleVector);
	double estimateScaleMean(std::vector<double>& scaleVector);

private:
	Features2D oldFeatures;
	Features3Dn old3DPoints;

};

#endif /* INCLUDE_BACKEND_H_ */
