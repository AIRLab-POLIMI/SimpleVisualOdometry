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

#include <list>

class BackendSFM: public Backend
{
public:
	BackendSFM();
	virtual Features3D getFeatures() const override;


protected:
	virtual void prelude() override;

	virtual void startup(Features2D& trackedFeatures, Features2D& newFeatures) override;
	virtual void initialization(Features2D& trackedFeatures,
				Features2D& newFeatures) override;
	virtual void tracking(Features2D& trackedFeatures, Features2D& newFeatures) override;
	virtual void recovery(Features2D& trackedFeatures, Features2D& newFeatures) override;

private:
	void getCorrespondences(const Features2D& trackedFeatures, Features2D& features2D, Features3D& features3D);

	void computeInitialCameras(cv::Mat C, cv::Mat& C0, cv::Mat& C1);

	cv::Mat rodriguesFromPose(const Eigen::Affine3d& T);
	cv::Mat translationFromPose(const Eigen::Affine3d& T);
	cv::Mat computeCameraMatrix(const cv::Mat& rodrigues, const cv::Mat& t);

private:
	struct Candidates
	{
		Candidates(const cv::Mat& C,
				   const Eigen::Affine3d F,
				   const Features2D& features) : C(C), F(F), features(features)
		{

		}

		cv::Mat C;
		Eigen::Affine3d F;
		Features2D features;
	};

	typedef std::list<Candidates> CandidatesList;

private:
	//Features data
	Features2D oldFeatures;
	Features3D old3DPoints;
	Features3D new3DPoints;
	CandidatesList candidates;

private:
	static const unsigned int minInitialFeatures = 100;
	static const unsigned int minFeatures = 10;

};



#endif /* INCLUDE_BACKENDSFM_H_ */
