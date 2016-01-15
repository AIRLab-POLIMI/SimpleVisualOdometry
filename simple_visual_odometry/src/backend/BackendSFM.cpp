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

BackendSFM::BackendSFM()
{
	Eigen::Quaterniond q_RC(-0.5, 0.5, -0.5, 0.5);

	Fpoints.setIdentity();
	Fpoints.rotate(q_RC);
}

Eigen::Affine3d BackendSFM::computePose(Features2D& trackedFeatures,
			Features2D& newFeatures)
{

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
					//compute transform
					vector<unsigned char> mask;
					Mat C = recoverCameraFromEssential(oldCorrespondences,
								newCorrespondences, mask);

					//compute absolute cameras
					cv::Mat C0, C1;
					computeInitialCameras(C, C0, C1);

					std::cout << "C0" << std::endl;
					std::cout << C0 << std::endl;
					std::cout << "C1" << std::endl;
					std::cout << C1 << std::endl;
					std::cout << "C" << std::endl;
					std::cout << C << std::endl;

					//triangulate
					Features3D&& triangulated = triangulate(oldCorrespondences,
								newCorrespondences, mask, C1, C0);

					//Save points
					old3DPoints = triangulated;
					new3DPoints = triangulated;

					std::cout << "T_C0W" << std::endl;
					std::cout << T_WC.inverse().matrix() << std::endl;

					//Compute new pose
					T_WC = cameraToTransform(C1);

					std::cout << "T_C1W" << std::endl;
					std::cout << T_WC.inverse().matrix() << std::endl;


					std::cout << "T" << std::endl;
					std::cout << cameraToTransform(C).inverse().matrix() << std::endl;

					//Update state
					state = Tracking;
					std::cout << "Tracking" << std::endl;
				}

			}

			break;

			case Tracking:
			{
				//Compute motion
				Features2D features2D;
				Features3D features3D;
				getCorrespondences(trackedFeatures, features2D, features3D);

				Eigen::Affine3d T_CW = T_WC.inverse();
				cv::Mat rvec = rodriguesFromPose(T_CW);
				cv::Mat tvec = translationFromPose(T_CW);

				vector<unsigned char> mask;

				solvePnPRansac(features3D.getPoints(), features2D.getPoints(),
							K, Mat(), rvec, tvec, true, 300, 1.0,
							0.9 * features2D.size(), mask, CV_ITERATIVE);

				/*solvePnP(features3D.getPoints(), features2D.getPoints(),
											K, Mat(), rvec, tvec, true, CV_ITERATIVE);*/

				cv::Mat C = computeCameraMatrix(rvec, tvec);

				//Compute structure


				//Compute new pose
				T_WC = cameraToTransform(C);
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

		//Update state
		state = Lost;
	}

	return T_WC;

}

Features3D BackendSFM::getFeatures() const
{
	return new3DPoints;
}

void BackendSFM::getCorrespondences(const Features2D& trackedFeatures,
			Features2D& features2D, Features3D& features3D)
{
	for (unsigned int i = 0; i < trackedFeatures.size(); i++)
	{
		unsigned int id = trackedFeatures.getId(i);

		if (old3DPoints.contains(id))
		{
			unsigned int index = old3DPoints.getIndex(id);

			features2D.addPoint(trackedFeatures[i], id);
			features3D.addPoint(old3DPoints[index], id);
		}
	}
}

void BackendSFM::computeInitialCameras(cv::Mat C, cv::Mat& C0, cv::Mat& C1)
{
	Eigen::Affine3d T = cameraToTransform(C);

	Eigen::Affine3d T_WC1 = T_WC*T;

	C0 = transformToCamera(T_WC);
	C1 = transformToCamera(T_WC1);
}

cv::Mat BackendSFM::rodriguesFromPose(const Eigen::Affine3d& T)
{
	Eigen::Matrix3d R = T.rotation();
	Mat Rcv = (Mat_<double>(3, 3) <<  //
				R(0, 0), R(0, 1), R(0, 2), //
	R(1, 0), R(1, 1), R(1, 2), //
	R(2, 0), R(2, 1), R(2, 2));

	Mat rvec;
	Rodrigues(Rcv, rvec);

	return rvec;
}

cv::Mat BackendSFM::translationFromPose(const Eigen::Affine3d& T)
{
	Mat_<double> t(3, 1);

	t(0, 0) = T.translation().x();
	t(1, 0) = T.translation().y();
	t(2, 0) = T.translation().z();

	return t;
}

cv::Mat BackendSFM::computeCameraMatrix(const cv::Mat& rvec, const cv::Mat& t)
{
	Mat R;
	Rodrigues(rvec, R);

	Mat C;
	hconcat(R, t, C);

	return C;

}
