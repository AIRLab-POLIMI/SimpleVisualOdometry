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

#ifndef INCLUDE_BACKENDSFM_H_
#define INCLUDE_BACKENDSFM_H_

#include "Backend.h"

class BackendSFM: public Backend
{
public:
	BackendSFM();
	virtual Eigen::Affine3d computePose(Features2D& trackedFeatures, Features2D& newFeatures) override;
	virtual Features3D getFeatures() const override;

private:
	void getCorrespondences(const Features2D& trackedFeatures, Features2D& features2D, Features3D& features3D);


	cv::Mat rodriguesFromPose(const Eigen::Affine3d& T);
	cv::Mat translationFromPose(const Eigen::Affine3d& T);
	cv::Mat computeCameraMatrix(const cv::Mat& rodrigues, const cv::Mat& t);

private:
	//Features data
	Features2D oldFeatures;
	Features3D old3DPoints;
	Features3D new3DPoints;

private:
	static const unsigned int minInitialFeatures = 100;
	static const unsigned int minFeatures = 10;

};



#endif /* INCLUDE_BACKENDSFM_H_ */
