/*
 * CollarLineFilter.h
 *
 *  Created on: 23.2.2016
 *      Author: ivelas
 */

#ifndef COLLARLINEFILTER_H_
#define COLLARLINEFILTER_H_

#include <vector>

#include <but_velodyne/PointCloudLine.h>
#include <but_velodyne/PolarGridOfClouds.h>

namespace but_velodyne
{

class CollarLinesFilter
{
public:

  enum COMPARATION {
    LINE_LENGTH,
    HORIZONTAL_RANGE_DIFF
  };

  CollarLinesFilter(int lines_to_preserve_, COMPARATION comparation_metric_ = LINE_LENGTH) :
    lines_to_preserve(lines_to_preserve_),
    comparation_metric(comparation_metric_){
  }

  virtual ~CollarLinesFilter() {
  }

  void filterLines(const std::vector<PointCloudLine> &in_line_cloud,
                   std::vector<PointCloudLine> &out_line_cloud,
                   const CellId &src_cell, const CellId &targ_cell) const;

protected:
  virtual bool checkLine(const PointCloudLine &line, const CellId &src_cell, const CellId &targ_cell) const;

private:
  const int lines_to_preserve;
  COMPARATION comparation_metric;
};

class AngularCollarLinesFilter : public CollarLinesFilter {
public:
  AngularCollarLinesFilter(int lines_to_preserve_, COMPARATION comparation_metric_,
                          float horizontal_range_diff_tolerance_rel_,
                          float horizontal_range_diff_tolerance_abs_,
                          float max_horizontal_range_diff_) :
                            CollarLinesFilter(lines_to_preserve_, comparation_metric_),
                            horizontal_range_diff_tolerance_abs(horizontal_range_diff_tolerance_abs_),
                            horizontal_range_diff_tolerance_rel(horizontal_range_diff_tolerance_rel_),
                            max_horizontal_range_diff(max_horizontal_range_diff_) {
  }

protected:
  virtual bool checkLine(const PointCloudLine &line, const CellId &src_cell, const CellId &targ_cell) const;

private:
  const float horizontal_range_diff_tolerance_rel;
  const float horizontal_range_diff_tolerance_abs;
  const float max_horizontal_range_diff;
};

} /* namespace but_visual_registration */

#endif /* COLLARLINEFILTER_H_ */
