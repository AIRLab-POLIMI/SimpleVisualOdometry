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

#ifndef INCLUDE_UTILS_H_
#define INCLUDE_UTILS_H_

#include <vector>

inline std::ostream& operator<<(std::ostream& os,
			const std::vector<unsigned char>& v)
{
	if (!v.empty())
	{
		size_t i;
		for (i = 0; i + 1 < v.size(); i++)
			os << (v[i] != 0) << ",";

		os << v[i];
	}
	return os;
}

inline unsigned int countInlier(const std::vector<unsigned char>& v)
{
	unsigned int count = 0;
	for(auto c : v)
	{
		if(c != 0)
			count++;
	}

	return count;

}

#endif /* INCLUDE_UTILS_H_ */
