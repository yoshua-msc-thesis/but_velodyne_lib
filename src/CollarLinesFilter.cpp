/*
 * CollarLineFilter.cpp
 *
 *  Created on: 23.2.2016
 *      Author: ivelas
 */

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <but_velodyne/CollarLinesFilter.h>
#include <but_velodyne/Visualizer3D.h>

using namespace std;
using namespace pcl;

namespace but_velodyne
{

bool CollarLinesFilter::checkLine(const PointCloudLine &line, const CellId &src_cell, const CellId &targ_cell) const {
  return true;
}

bool orderLinesByHorizontalRangeDiff(const PointCloudLine &first, const PointCloudLine &second) {
  return first.horizontalRangeDiff() < second.horizontalRangeDiff();
}

void CollarLinesFilter::filterLines(const vector<PointCloudLine> &in_lines, vector<PointCloudLine> &out_lines,
                                   const CellId &src_cell, const CellId &targ_cell) const {
  //Visualizer3D::getCommonVisualizer()->getViewer()->removeAllShapes();
  vector<PointCloudLine> filtered_lines;
  for(vector<PointCloudLine>::const_iterator l = in_lines.begin(); l < in_lines.end(); l++) {
    if(checkLine(*l, src_cell, targ_cell)) {
      //Visualizer3D::getCommonVisualizer()->addLine(*l, 0.0, 0.0, 1.0);
      filtered_lines.push_back(*l);
    } else {
      //Visualizer3D::getCommonVisualizer()->addLine(*l, 1.0, 0.0, 0.0);
    }
  }
  //if(abs(src_cell.ring - targ_cell.ring) > 1)
    //Visualizer3D::getCommonVisualizer()->show();
  if(comparation_metric == LINE_LENGTH) {
    sort(filtered_lines.begin(), filtered_lines.end());
  } else if (comparation_metric == HORIZONTAL_RANGE_DIFF) {
    sort(filtered_lines.begin(), filtered_lines.end(), orderLinesByHorizontalRangeDiff);
  }
  int lines_to_effectively_preserve = MIN(lines_to_preserve, filtered_lines.size());
  out_lines.insert(out_lines.end(), filtered_lines.begin(), filtered_lines.begin()+lines_to_effectively_preserve);
}

bool AngularCollarLinesFilter::checkLine(const PointCloudLine &line, const CellId &src_cell, const CellId &targ_cell) const {
  float src_range = VelodyneSpecification::getExpectedRange(src_cell.ring, VelodyneSpecification::KITTI_HEIGHT);
  float targ_range = VelodyneSpecification::getExpectedRange(targ_cell.ring, VelodyneSpecification::KITTI_HEIGHT);
  float expected_diff = (isinf(src_range)||isinf(targ_range)) ? max_horizontal_range_diff : MIN(fabs(src_range - targ_range), max_horizontal_range_diff);

  float actual_diff = line.horizontalRangeDiff();
  //if(line.orientation.norm() > 4 && actual_diff < (expected_diff + TOLERANCE))
    //cerr << "line: " << line << " exp: " << expected_diff << " actual: " << actual_diff << endl;
  return actual_diff < (expected_diff*horizontal_range_diff_tolerance_rel + horizontal_range_diff_tolerance_abs);
}

} /* namespace but_visual_registration */
