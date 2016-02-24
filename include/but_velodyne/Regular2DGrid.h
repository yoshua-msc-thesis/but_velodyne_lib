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

namespace but_velodyne {

/**!
 * Row organized.
 */
template <typename CellType>
class Regular2DGrid {
public:

  Regular2DGrid(int rows_, int cols_) :
    cells(rows_*cols_),
    rows(rows_), cols(cols_) {
  }

  const CellType& at(int r, int c) const {
    return cells[r*cols + c];
  }

  CellType& at(int r, int c) {
    return cells[r*cols + c];
  }

  const int cols, rows;

//private:
  std::vector<CellType> cells;
};

}

#endif /* REGULAR2DGRID_H_ */
