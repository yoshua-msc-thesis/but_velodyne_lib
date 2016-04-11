/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 22/02/2016
 *
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef VELODYNE_H_
#define VELODYNE_H_

#include <iostream>
#include <map>

namespace but_velodyne {

float radToDeg(float rad);

float degToRad(float deg);

class VelodyneSpecification {

public:

  /**
   * Angles in degrees, O = sensor.
   *
   *    /^ maximal angle
   *   /
   *  /
   * O--------> horizontal line (90°)
   * |\
   * | \
   * |  \
   * |   v minimal angle
   * V
   * (0°)
   */

#if VELODYNE_MODEL == 16

  static const float MIN_ANGLE =  75.00;        // deg
  static const float MAX_ANGLE = 105.00;

#elif VELODYNE_MODEL == 36

  static const float MIN_ANGLE =  59.33;
  static const float MAX_ANGLE = 100.67;

#elif VELODYNE_MODEL == 64

  static const float MIN_ANGLE = 62.13;
  static const float MAX_ANGLE = 92.00;

#else

#error Type of the Velodyne LiDAR is not specified.

#endif

#ifdef VELODYNE_MODEL
  static const int RINGS = VELODYNE_MODEL;
#endif

  static const float KITTI_HEIGHT = 1.73;        // m


  /**!
   * O sensor
   * | \
   * |  \ laser ray
   * |   \
   * |    \
   * |-----x point
   * |     |
   *  Range
   *
   * @param ring ID (rings are indexed from furthest to nearest)
   * @param height how height is Velodyne positioned
   * @return expected horizontal range of points from given ring
   */
  static float getExpectedRange(const int ring, const int height = KITTI_HEIGHT) {
    int n = RINGS - ring - 1;       // invert indexing (0=nearest)
    static std::map<int, float> ranges;

    if(ranges.empty()) {
      float delta = (MAX_ANGLE - MIN_ANGLE)/(RINGS-1);
      for(int n = 0; n < RINGS; n++) {
        float angle = MIN_ANGLE + n*delta;
        ranges[n] = (angle < 90.0f) ? (tan(degToRad(angle))*height) : INFINITY;
      }
    }

    return ranges[n];
  }
};

}

#endif /* VELODYNEPOINTCLOUD_H_ */
