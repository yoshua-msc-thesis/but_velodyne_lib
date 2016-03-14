/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 24/02/2016
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

#ifndef REGULAR2DGRIDGENERATOR_H_
#define REGULAR2DGRIDGENERATOR_H_

#include <cv.h>

#include <but_velodyne/Regular2DGrid.h>

namespace but_velodyne {

class Regular2DGridGenerator {
public:
  class Parameters {
  public:
    Parameters(
        float width_ = 40,
        float height_ = 50,
        float resolution_ = 0.2,
        float h_origin_ = -20,
        float v_origin_ = -20) :
          width(width_), height(height_),
          rows(height_/resolution_), cols(width_/resolution_),
          h_origin(h_origin_), v_origin(v_origin_),
          resolution(resolution_) {
    }

    bool operator==(const Parameters &other) {
      static const float eps = 0.01;
      return
          fabs(this->width - other.width) < eps &&
          fabs(this->height - other.height) < eps &&
          this->rows == other.rows &&
          this->cols == other.cols &&
          this->h_origin == other.h_origin &&
          this->v_origin == other.v_origin;
    }

    cv::Point getSensorPosition() const {
      cv::Point pos;
      pos.x = -h_origin/resolution;
      pos.y = -v_origin/resolution;
      return pos;
    }

    float width, height;
    int rows, cols;
    float h_origin, v_origin;
    float resolution;
  } params;

  Regular2DGridGenerator(Parameters params_) :
    params(params_) {
  }

  template <typename PointT>
  void generate(const pcl::PointCloud<PointT> &in_cloud,
                Regular2DGrid< pcl::PointCloud<PointT> > &output) {
    cv::Rect_<float> dimensions(params.h_origin, params.v_origin, params.width, params.height);
    for(typename pcl::PointCloud<PointT>::const_iterator pt = in_cloud.begin(); pt < in_cloud.end(); pt++) {
      cv::Point2f pt_2D(pt->x, pt->z);
      if(pt_2D.inside(dimensions)) {
        int col = MAX(0, MIN((pt->x-params.h_origin)/params.resolution, params.cols-1));
        int row = MAX(0, MIN((pt->z-params.v_origin)/params.resolution, params.rows-1));
        output.at(row, col)->push_back(*pt);
      }
    }
  }
};

}

#endif /* REGULAR2DGRIDGENERATOR_H_ */
