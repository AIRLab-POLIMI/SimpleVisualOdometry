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

#ifndef INCLUDE_FEATURES_H_
#define INCLUDE_FEATURES_H_

#include<opencv2/opencv.hpp>

class Features
{
public:
	Features();

	inline std::vector<cv::KeyPoint> getKeyPoints()
	{
		return keyPoints;
	}

	inline unsigned int getId(unsigned int index)
	{
		return id[index];
	}

	inline unsigned int getIndex(unsigned int id)
	{
		return indexes[id];
	}

private:
	std::vector<cv::KeyPoint> keyPoints;
	std::vector<unsigned int> id;
	std::map<unsigned int, unsigned int> indexes;

};



#endif /* INCLUDE_FEATURES_H_ */
