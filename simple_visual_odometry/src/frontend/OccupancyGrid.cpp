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

#include "frontend/OccupancyGrid.h"

using namespace std;
using namespace cv;

OccupancyGrid::OccupancyGrid()
{
	Iy = 1;
	Ix = 1;

	resetGrid();
}

void OccupancyGrid::setImageSize(size_t cols, size_t rows)
{
	Ix = cols / nx;
	Iy = rows / ny;
}

void OccupancyGrid::addPoint(Point2f& p)
{
	size_t i = p.x / Ix;
	size_t j = p.y / Iy;

	if(i >= nx || j >= ny)
		return;

	isFree[i][j] = false;
}

bool OccupancyGrid::isNewFeature(Point2f& p)
{
	int i = p.x / Ix;
	int j = p.y / Iy;

	bool isNew = true;

	unsigned int minX = std::max(0, i - 1);
	unsigned int maxX = std::min((int)nx, i + 2);

	unsigned int minY = std::max(0, j - 1);
	unsigned int maxY = std::min((int)ny, j + 2);

	for(unsigned int x = minX; x < maxX; x++)
		for(unsigned int y = minY; y < maxY; y++)
			isNew = isNew && isFree[x][y];


	return isNew;
}

void OccupancyGrid::resetGrid()
{
	for (size_t i = 0; i < nx; i++)
		for (size_t j = 0; j < ny; j++)
			isFree[i][j] = true;
}

