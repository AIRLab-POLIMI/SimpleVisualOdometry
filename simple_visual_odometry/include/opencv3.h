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

#ifndef INCLUDE_OPENCV3_H_
#define INCLUDE_OPENCV3_H_

#include <opencv2/opencv.hpp>

namespace cv
{
//Opencv3 function, use original ones when they come out
void decomposeEssentialMat(cv::InputArray _E, cv::OutputArray _R1,
		cv::OutputArray _R2, cv::OutputArray _t);
int recoverPose(cv::InputArray E, cv::InputArray _points1,
		cv::InputArray _points2, cv::OutputArray _R, cv::OutputArray _t,
		cv::InputOutputArray _mask = cv::noArray(), double focal = 1.0,
		cv::Point2d pp = cv::Point2d(0, 0));
}


#endif /* INCLUDE_OPENCV3_H_ */
