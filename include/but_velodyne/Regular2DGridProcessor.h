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

#ifndef REGULAR2DGRIDPROCESSOR_H_
#define REGULAR2DGRIDPROCESSOR_H_

#include <stdexcept>

#include <cv.h>

#include <but_velodyne/Regular2DGrid.h>

namespace but_velodyne {

template <typename SourceCellT, typename TargetT>
class Modificator {
public:

  virtual ~Modificator() {
  }

  virtual void operator()(const SourceCellT &s, TargetT &t) =0;
};

template <typename CellT>
class Aggregator {
public:

  virtual ~Aggregator() {
  }

  virtual void operator()(const std::vector< boost::shared_ptr<CellT> > &sources, CellT &sum) =0;
};

class Regular2DGridProcessor {
public:
  template <typename SourceCellT, typename TargetCellT>
  static void modify(const Regular2DGrid<SourceCellT> &source,
                     Regular2DGrid<TargetCellT> &target,
                     Modificator<SourceCellT, TargetCellT> &m) {
    if(source.cols != target.cols || source.rows != target.rows) {
      throw std::invalid_argument("Source and target grid have different size.");
    }
    for(int r = 0; r < source.rows; r++) {
      for(int c = 0; c < source.cols; c++) {
        m(*source.at(r, c), *target.at(r,c));
      }
    }
  }

  template <typename CellT>
  static void aggregate(const std::vector< typename Regular2DGrid<CellT>::Ptr > &source,
                        Regular2DGrid<CellT> &sum_target,
                        Aggregator<CellT> &aggregator) {
    for(typename std::vector< typename Regular2DGrid<CellT>::Ptr >::const_iterator s_grid = source.begin(); s_grid < source.end(); s_grid++) {
      if((*s_grid)->cols != sum_target.cols || (*s_grid)->rows != sum_target.rows) {
        throw std::invalid_argument("Source and target grid have different size.");
      }
    }
    for(int r = 0; r < sum_target.rows; r++) {
      for(int c = 0; c < sum_target.cols; c++) {
        std::vector< boost::shared_ptr<CellT> > cells;
        for(typename std::vector< typename Regular2DGrid<CellT>::Ptr >::const_iterator s_grid = source.begin(); s_grid < source.end(); s_grid++) {
          cells.push_back((*s_grid)->at(r, c));
        }
        aggregator(cells, *sum_target.at(r, c));
      }
    }
  }
};

}

#endif /* REGULAR2DGRIDPROCESSOR_H_ */
