#ifndef _GROUND_DETECTION_DATA_GENERATOR_H_
#define _GROUND_DETECTION_DATA_GENERATOR_H_

#include <cstdlib>
#include <cstdio>
#include <libgen.h>

#include <pcl/common/eigen.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>

#include <Eigen/Eigenvalues>

#include <boost/math/distributions/normal.hpp>

#include <cv.h>
#include <opencv2/highgui.hpp>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/PolarGridOfClouds.h>
#include <but_velodyne/EigenUtils.h>

namespace but_velodyne {

template <typename T>
class FillingFM {
public:
  typedef enum {
    START, NOT_FIXABLE, OCCUPIED, FIXABLE
  } State;

  typedef struct {
    T first_value, last_value;
    int first_index, last_index;
  } FillData;

  FillingFM() :
    state(START), index(0), valid_fill(false) {
  }

  void next(bool occupancy, T value) {
    if(occupancy) {
      if(state == FIXABLE) {
        current_data.last_value = value;
        current_data.last_index = index-1;
        last_data = current_data;
        valid_fill = true;
      }
      current_data.first_value = value;
      state = OCCUPIED;
    } else {
      switch(state) {
        case START:
        case NOT_FIXABLE:
          state = NOT_FIXABLE; break;
        case OCCUPIED:
          current_data.first_index = index;
        case FIXABLE:
          state = FIXABLE; break;
      }
    }
    index++;
  }

  bool getFillData(FillData &out) {
    out = last_data;
    return valid_fill;
  }

private:
  State state;
  FillData current_data, last_data;
  int index;
  bool valid_fill;
};

class GroundDetectionDataGenerator {
public:

  class Parameters {
  public:
    Parameters(
        float threshold_ = 0.5,
        int polar_bins_ = 360,
        std::string labels_output_ = ".",
				bool save_visualization_ = true
        ) :
          threshold(threshold_),
          polar_bins(polar_bins_),
          labels_output(labels_output_),
					save_visualization(save_visualization_) {
    }

    float threshold;
    int polar_bins;
    std::string labels_output;
    bool save_visualization;
  } params;

  typedef enum {
    X, Y, Z, RANGE, INTENSITY
  } InputDataTypes;

  GroundDetectionDataGenerator(const VelodynePointCloud &cloud,
                               const std::string &file_basename_ = "",
                               Parameters params_ = Parameters());

  cv::Mat getMatrixOf(const InputDataTypes &type);

  void getGroundLabels(const std::vector<float> &probabilities, cv::Mat &out_probabilities, cv::Mat &out_labels);

  void getGroundLabelsFromAnn(const std::vector<int> &annotations, cv::Mat &out_labels);

  void getGroundLabelsFromBinaryAnn(const std::vector<int> &annotations, cv::Mat &out_labels);

  void saveDato(const cv::Mat &matrix, const std::string &data_name);

  void saveData(const std::vector<cv::Mat> &matrices, const std::vector<std::string> &data_names);

  void saveAsImage(const cv::Mat &matrix, const std::string &data_name);

  const std::vector<CellId> &getIndices () const {
    return indices;
  }

  const cv::Mat& getOccupancy () const {
    return occupancy;
  }

protected:

  template <typename T>
  void fillMissing(cv::Mat &data) {
    for(int c = 0; c < PolarGridOfClouds::getPolarBins(); c++) {
      FillingFM<T> fill_fm;
      for(int r = 0; r < VelodyneSpecification::RINGS; r++) {
        fill_fm.next(occupancy.at<uchar>(r, c) != 0, data.at<T>(r, c));
        typename FillingFM<T>::FillData fill_data;
        if(fill_fm.getFillData(fill_data)) {
	  T delta = (fill_data.last_value - fill_data.first_value) / (fill_data.last_index - fill_data.first_index + 1);
	  for(int i = 0; i <= (fill_data.last_index - fill_data.first_index); i++) {
	    data.at<T>(fill_data.first_index + i, c) = delta*i + fill_data.first_value;
	  }
        }
      }
    }
  }

  void fillMissing(PolarGridOfClouds &summarized_data);

  bool isValid(velodyne_pointcloud::VelodynePoint pt);

  void summarizeProbabilities(const std::vector<float> &probabilities,
  		cv::Mat &out_matrix);

  void prob_to_labels(const cv::Mat &matrix_probabilities,
  		cv::Mat &matrix_ground_labels,
                        float prob_threshold);

private:
  PolarGridOfClouds::Ptr summary;
  std::vector<CellId> indices;
  cv::Mat occupancy;
  std::string file_basename;
  cv::FileStorage storage;
};

} // namespace but_velodyne

#endif
