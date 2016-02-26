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

#ifndef REGULAR2DGRID_H_
#define REGULAR2DGRID_H_

#include <iostream>

namespace but_velodyne {

/**!
 * Row organized.
 */
template <typename CellType>
class Regular2DGrid {
public:

  typedef boost::shared_ptr< Regular2DGrid<CellType> > Ptr;

  Regular2DGrid(int rows_, int cols_) :
    cells(rows_*cols_),
    rows(rows_), cols(cols_) {
    for(int i = 0; i < rows*cols; i++) {
      cells[i].reset(new CellType);
    }
  }

  const boost::shared_ptr<CellType> at(int r, int c) const {
    return cells[toIndex(r, c)];
  }

  boost::shared_ptr<CellType> at(int r, int c) {
    return cells[toIndex(r, c)];
  }

  const int cols, rows;

protected:
  int toIndex(int r, int c) const {
    return r*cols + c;
  }

private:
  std::vector< boost::shared_ptr<CellType> > cells;
};

template <typename CellType>
std::ostream& operator<< (std::ostream &out, const Regular2DGrid<CellType> &grid) {
  for(int r = 0; r < grid.rows; r++) {
    for(int c = 0; c < grid.cols; c++) {
      out << *grid.at(r,c) << "; ";
    }
    out << std::endl;
  }
  return out;
}


}

#endif /* REGULAR2DGRID_H_ */
