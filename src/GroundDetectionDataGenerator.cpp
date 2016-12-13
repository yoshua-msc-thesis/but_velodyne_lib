#include <but_velodyne/GroundDetectionDataGenerator.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace velodyne_pointcloud;

namespace but_velodyne {

GroundDetectionDataGenerator::GroundDetectionDataGenerator(const VelodynePointCloud &cloud,
			     const string &file_basename_,
			     Parameters params_) :
  file_basename(file_basename_),
  storage(params_.labels_output + "/" + file_basename_ + ".gd_data.yaml.gz", FileStorage::WRITE),
  params(params_),
  occupancy(VelodyneSpecification::RINGS, params_.polar_bins, CV_8UC1) {

  PolarGridOfClouds::POLAR_SUPERBINS = params.polar_bins;
  PolarGridOfClouds::BIN_SUBDIVISION = 1;
  PolarGridOfClouds polar_grid(cloud);
  indices = polar_grid.getIndices();

  summary = polar_grid.summarize();

  for(int r = 0; r < VelodyneSpecification::RINGS; r++) {
    for(int p = 0; p < PolarGridOfClouds::getPolarBins(); p++) {
      const CellId cell_id(p, r);
      if(!summary->at(cell_id).empty() && isValid(summary->at(cell_id).front())) {
	occupancy.at<uchar>(r, p) = 1;
      } else {
	occupancy.at<uchar>(r, p) = 0;
      }
    }
  }
}

Mat GroundDetectionDataGenerator::getMatrixOf(const InputDataTypes &type) {
  Mat output(VelodyneSpecification::RINGS, PolarGridOfClouds::getPolarBins(), CV_32FC1);
  for(int r = 0; r < VelodyneSpecification::RINGS; r++) {
    for(int p = 0; p < PolarGridOfClouds::getPolarBins(); p++) {
      const int c = p;
      const CellId cell_id(p, r);
      if(occupancy.at<uchar>(r, p) != 0) {
	PointXYZIR pt = summary->at(cell_id).front();
	switch(type) {
	  case X:
	    output.at<float>(r, c) = pt.x; break;
	  case Y:
	    output.at<float>(r, c) = pt.y / 3; break;
	  case Z:
	    output.at<float>(r, c) = pt.z; break;
	  case RANGE:
	    output.at<float>(r, c) = log10(pt.x*pt.x + pt.z*pt.z) / 2; break;	// log(sqrt(x^2 + z^2)) = 1/2 * log(x^2 + z^2)
	  case INTENSITY:
	    output.at<float>(r, c) = pt.intensity; break;
	}
      } else {
	output.at<float>(r, c) = 0.0;
      }
    }
  }
  fillMissing<float>(output);
  return output;
}

void GroundDetectionDataGenerator::getGroundLabels(const vector<float> &probabilities, Mat &out_probabilities, Mat &out_labels) {
  out_probabilities = Mat(VelodyneSpecification::RINGS, PolarGridOfClouds::getPolarBins(), CV_32FC1);
  summarizeProbabilities(probabilities, out_probabilities);
  fillMissing<float>(out_probabilities);
  out_labels = Mat(VelodyneSpecification::RINGS, PolarGridOfClouds::getPolarBins(), CV_8UC1);
  prob_to_labels(out_probabilities, out_labels, params.threshold);
}

/*
 * Label number / Object class / RGB
 *  0 - NOT GROUND TRUTHED - 255 255 255
 *  1 - building - 153 0 0
 *  2 - sky - 0 51 102
 *  3 - road - 160 160 160
 *  4 - vegetation - 0 102 0
 *  5 - sidewalk - 255 228 196
 *  6 - car - 255 200 50
 *  7 - pedestrian - 255 153 255
 *  8 - cyclist - 204 153 255
 *  9 - signage - 130 255 255
 * 10 - fence - 193 120 87
 */
void GroundDetectionDataGenerator::getGroundLabelsFromAnn(const vector<int> &annotations, Mat &out_labels) {
  out_labels = Mat::zeros(VelodyneSpecification::RINGS, PolarGridOfClouds::getPolarBins(), CV_8UC1);
  vector<int> counters(VelodyneSpecification::RINGS*PolarGridOfClouds::getPolarBins()*11, 0);
  for(int i = 0; i < annotations.size() && i < indices.size(); i++) {
    int ring = indices[i].ring;
    int polar = indices[i].polar;
    int ann = annotations[i];
    counters[ring*PolarGridOfClouds::getPolarBins()*11 +
	     polar*11 +
	     ann]++;
  }
  for(int r = 0; r < out_labels.rows; r++) {
    for(int c = 0; c < out_labels.cols; c++) {
      vector<int>::iterator first = counters.begin() + r*PolarGridOfClouds::getPolarBins()*11 + c*11;
      vector<int>::iterator max_it = max_element(first, first + 11);
       int label = (max_it - first);
       out_labels.at<uchar>(r, c) = (label == 3 || label == 5) ? 1 : 0;
    }
  }
  fillMissing<uchar>(out_labels);
}

void GroundDetectionDataGenerator::getGroundLabelsFromBinaryAnn(const vector<int> &annotations, Mat &out_labels) {
  out_labels = Mat::zeros(VelodyneSpecification::RINGS, PolarGridOfClouds::getPolarBins(), CV_8UC1);
  vector<int> counters(VelodyneSpecification::RINGS*PolarGridOfClouds::getPolarBins(), 0);
  for(int i = 0; i < annotations.size() && i < indices.size(); i++) {
    int ring = indices[i].ring;
    int polar = indices[i].polar;
    if(annotations[i] == 0) {
      counters[ring*PolarGridOfClouds::getPolarBins() + polar]--;
    } else {
      counters[ring*PolarGridOfClouds::getPolarBins() + polar]++;
    }
  }
  for(int r = 0; r < out_labels.rows; r++) {
    for(int c = 0; c < out_labels.cols; c++) {
      out_labels.at<uchar>(r, c) = (counters[r*PolarGridOfClouds::getPolarBins() + c] >= 0) ? 1 : 0;
    }
  }
  fillMissing<uchar>(out_labels);
}

void GroundDetectionDataGenerator::saveData(const Mat &matrix, const string &data_name) {
  storage << data_name << matrix;
  if(params.save_visualization) {
    Mat equalized;
    normalize(matrix, equalized, 0.0, 255.0, NORM_MINMAX);
    equalized.convertTo(equalized, CV_8UC1);
    imwrite(params.labels_output + "/" + file_basename + "." + data_name + ".png", equalized);
  }
}

void GroundDetectionDataGenerator::fillMissing(PolarGridOfClouds &summarized_data) {
  for(int c = 0; c < PolarGridOfClouds::getPolarBins(); c++) {
    FillingFM<PointXYZIR> fill_fm;
    for(int r = 0; r < VelodyneSpecification::RINGS; r++) {
      bool occupied = !summarized_data.at(CellId(c, r)).empty();
      PointXYZIR pt;
      if(occupied) {
	pt = summarized_data.at(CellId(c, r)).front();
      }
      fill_fm.next(occupied, pt);
      FillingFM<PointXYZIR>::FillData fill_data;
      if(fill_fm.getFillData(fill_data)) {
	PointXYZIR delta = (fill_data.last_value - fill_data.first_value) / (fill_data.last_index - fill_data.first_index + 1);
	for(int i = 0; i <= (fill_data.last_index - fill_data.first_index); i++) {
	  summarized_data.at(CellId(c, fill_data.first_index + i)).push_back(delta*i + fill_data.first_value);
	}
      }
    }
  }
}

bool GroundDetectionDataGenerator::isValid(PointXYZIR pt) {
  return EigenUtils::allFinite(pt.getVector4fMap()) &&
      !isnan(pt.intensity) && !isinf(pt.intensity) &&
      pt.y < 5 && pt.y > -5;
}

void GroundDetectionDataGenerator::summarizeProbabilities(const vector<float> &probabilities,
			    Mat &out_matrix) {
  out_matrix = 0.0;
  Mat counters(out_matrix.rows, out_matrix.cols, CV_32SC1);
  counters = Scalar(0);
  for(int i = 0; i < probabilities.size(); i++) {
    out_matrix.at<float>(indices[i].ring, indices[i].polar) += probabilities[i];
    counters.at<int>(indices[i].ring, indices[i].polar)++;
  }
  for(int i = 0; i < out_matrix.rows*out_matrix.cols; i++) {
    out_matrix.at<float>(i) /= counters.at<int>(i);
  }
}

void GroundDetectionDataGenerator::prob_to_labels(const Mat &matrix_probabilities,
		      Mat &matrix_ground_labels,
		      float prob_threshold) {
  for(int i = 0; i < matrix_probabilities.rows*matrix_probabilities.cols; i++) {
    if(occupancy.at<uchar>(i) == 0) {
      matrix_ground_labels.at<uchar>(i) = 1;
    } else {
      matrix_ground_labels.at<uchar>(i) = (matrix_probabilities.at<float>(i) > prob_threshold) ? 1 : 0;
    }
  }
}

} // namespace but_velodyne
