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

#ifndef INCLUDE_VISUALFRONTEND_H_
#define INCLUDE_VISUALFRONTEND_H_

#include "Features.h"
#include "OccupancyGrid.h"

class VisualFrontend
{
public:
	VisualFrontend();

	void trackAndExtract(cv::Mat& im_gray, Features2D& newPoints);

	inline Features2D& getCurrentFeatures()
	{
		return oldPoints;
	}

protected:
	void extract(cv::Mat& im_gray);
	void track(cv::Mat& im_gray, Features2D& points);

protected:
	//extracted data
	Features2D oldPoints;
	cv::Mat im_prev;

private:
	//ID
	unsigned int newId;

	//algorithms
	cv::Ptr<cv::FeatureDetector> detector;

	//Data needed to guide extraction
	OccupancyGrid grid;

	//parameters
	const int thresholdExtraction = 50;
	const double thresholdFBError = 0.5;

};



#endif /* INCLUDE_VISUALFRONTEND_H_ */
