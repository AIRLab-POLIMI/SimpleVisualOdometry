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

template<class Type>
class Features
{
public:
	Features()
	{

	}

	Features(Features<Type>& oldFeatures,
			std::vector<Type> newPoints,
			std::vector<unsigned char> status)
	{
		unsigned int j = 0;
		for(unsigned int i = 0; i < oldFeatures.size(); i++)
		{
			if(status[i])
			{
				unsigned int id_j = oldFeatures.getId(i);
				ids.push_back(id_j);
				indexes[id_j] = j;
				points.push_back(newPoints[i]);
				j++;
			}
		}

	}

	void addPoint(const Type& point, unsigned int id)
	{
		unsigned int index = points.size();
		points.push_back(point);
		ids.push_back(id);
		indexes[id] = index;
	}

	Type& operator[](unsigned int i)
	{
		return points[i];
	}

	typename std::vector<Type>::iterator begin()
	{
		return points.begin();
	}

	typename std::vector<Type>::iterator end()
	{
		return points.end();
	}

	inline std::vector<Type>& getPoints()
	{
		return points;
	}

	inline unsigned int getId(unsigned int index)
	{
		return ids[index];
	}

	inline unsigned int getIndex(unsigned int id)
	{
		if(indexes.count(id) == 0)
			throw std::runtime_error("no index");
		return indexes[id];
	}

	inline size_t size()
	{
		return points.size();
	}

	inline bool contains(unsigned int id)
	{
		return indexes.count(id) != 0;
	}

private:
	std::vector<Type> points;
	std::vector<unsigned int> ids;
	std::map<unsigned int, unsigned int> indexes;

};

typedef Features<cv::Point2f> Features2D;
typedef Features<cv::Point3f> Features3D;

typedef Features<cv::Vec2d> Features2Dn;
typedef Features<cv::Vec3d> Features3Dn;


#endif /* INCLUDE_FEATURES_H_ */
