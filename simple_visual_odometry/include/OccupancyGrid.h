/*
 * simple_visual_odometry,
 *
 *
 * Copyright (C) 2014 Davide Tateo
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

#ifndef OCCUPANCYGRID_H_
#define OCCUPANCYGRID_H_

#include <vector>
#include <opencv2/opencv.hpp>

class OccupancyGrid
{
public:
	OccupancyGrid();

	void setImageSize(size_t cols, size_t rows);
	void addPoint(cv::Point2f& p);
	bool isNewFeature(cv::Point2f& p);
	void resetGrid();

private:
	// number of cells
	static const size_t nx = 16;
	static const size_t ny = 10;

	// Data needed by the algorithm
	bool isFree[nx][ny];
	size_t Ix;
	size_t Iy;

};


#endif /* OCCUPANCYGRID_H_ */
