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

#include "ConfigManager.h"

using namespace std;

ConfigManager::ConfigManager()
{
	ros::NodeHandle n("~");

	 //Init camera pose wrt odometric center
	  vector<double> T_WR_std(7);
	  if (!n.getParam("T_WR", T_WR_std)) {
	    throw runtime_error("No camera pose specified");
	  }

	  tf::Vector3 t(T_WR_std[0], T_WR_std[1], T_WR_std[2]);
	  tf::Quaternion q(T_WR_std[4], T_WR_std[5], T_WR_std[6], T_WR_std[3]);

	  T_WR = tf::Transform(q, t);
}
