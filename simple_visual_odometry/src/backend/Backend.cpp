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

#include "backend/Backend.h"

using namespace cv;
using namespace std;


Backend::Backend()
{
	Kscale = 0;
	state = Initial;
}

cv::Vec2d Backend::computeNormalizedPoint(cv::Point2f& point)
{
	cv::Vec3d hpt;
	hpt[0] = point.x;
	hpt[1] = point.y;
	hpt[2] = 1;

	hpt = Kinv * hpt;

	cv::Vec2d pt;
	pt[0] = hpt[0];
	pt[1] = hpt[1];

	return pt;
}

double Backend::computeNormalizedFeatures(Features2D& oldFeatures,
			Features2D& newFeatures, Features2Dn& oldFeaturesNorm,
			Features2Dn& newFeaturesNorm)
{

	double deltaMean = 0;
	unsigned int N = 0;

	for (unsigned int i = 0; i < newFeatures.size(); i++)
	{
		unsigned int id = newFeatures.getId(i);

		if (oldFeatures.contains(id))
		{
			unsigned int index = oldFeatures.getIndex(id);

			Vec2d oldPoint = computeNormalizedPoint(oldFeatures[index]);
			Vec2d newPoint = computeNormalizedPoint(newFeatures[i]);

			oldFeaturesNorm.addPoint(oldPoint, id);
			newFeaturesNorm.addPoint(newPoint, id);

			deltaMean += cv::norm(oldFeatures[index] - newFeatures[i]);
			N++;
		}
	}

	deltaMean /= N;
	return deltaMean;
}
