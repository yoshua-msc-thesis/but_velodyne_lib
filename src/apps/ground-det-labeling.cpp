/*
 * Velodyne data labeling for training of ground detector.
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 19/09/2014
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

#include <cstdlib>
#include <cstdio>
#include <libgen.h>

#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>

#include <Eigen/Eigenvalues>

#include <boost/program_options.hpp>
#include <boost/math/distributions/normal.hpp>

#include <cv.h>
#include <opencv2/highgui.hpp>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/PolarGridOfClouds.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/EigenUtils.h>
#include <but_velodyne/GroundDetectionDataGenerator.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

namespace po = boost::program_options;


#define log cerr

float gauss(float mean, float sd, float x, bool normalize) {
  float exponent = (x - mean) / sd;
  exponent *= -exponent / 2.0;
  if(normalize) {
    return exp(exponent) / (sd * sqrt(2*M_PI));
  } else {
    return exp(exponent);
  }
}

class NormalizedFeature {
public:
  NormalizedFeature() :
    sorted(true) {
    setRelativeMinIndex();
    setRelativeMaxIndex();
  }

  virtual ~NormalizedFeature() {
  }

  float normalize(float value) {
    if(!sorted) {
      for(vector<float>::iterator v = values.begin(); v < values.end();) {
        if(isnan(*v) || isinf(*v)) {
          v = values.erase(v);
        } else {
          v++;
        }
      }
      std::sort(values.begin(), values.end());
      sorted = true;
    }
    float min = values[values.size()*relative_min_index];
    float max = values[values.size()*relative_max_index];
    if(value > max) {
      return 1.0;
    } else if(value < min) {
      return 0.0;
    }
    return (value - min) / (max - min);
  }

  void setRelativeMaxIndex(float relativeMaxIndex = 0.9) {
    relative_max_index = relativeMaxIndex;
  }

  void setRelativeMinIndex(float relativeMinIndex = 0.1) {
    relative_min_index = relativeMinIndex;
  }

protected:
  void updateMinMax(float value) {
    sorted = false;
    values.push_back(value);
  }

  vector<float> values;
  bool sorted;
  float relative_min_index;
  float relative_max_index;
};

float deviation(vector<float>::const_iterator begin, vector<float>::const_iterator end) {
  float mean = 0.0;
  int count = 0;
  for(vector<float>::const_iterator range = begin; range < end; range++) {
    if(!isnan(*range)) {
      mean += *range;
      count++;
    }
  }
  mean /= count;

  float deviation = 0.0;
  for(vector<float>::const_iterator range = begin; range < end; range++) {
    if(!isnan(*range)) {
      deviation += pow(*range - mean, 2.0);
    }
  }
  deviation = sqrt(deviation/count);
  return deviation;
}

class GroundProbabilityByHeight : public NormalizedFeature {
public:
  float compute(const Eigen::Vector3f &current_point) {
    updateMinMax(current_point.y());
    return current_point.y();
  }
};

class GroundProbabilityByHeightDev : public NormalizedFeature {
public:
  GroundProbabilityByHeightDev() :
    ranges_diff(VelodyneSpecification::RINGS) {
  }

  void setData(PolarGridOfClouds::Ptr grid, int polar_id) {
    for (int ring = VelodyneSpecification::RINGS - 1; ring >= 0; ring--) {
      if (grid->at(CellId(polar_id, ring)).empty()) {
        ranges_diff[ring] = NAN;
      }
      else {
        velodyne_pointcloud::PointXYZIR pt = grid->at(CellId(polar_id, ring)).front();
        ranges_diff[ring] = pt.y;
      }
    }
  }

  float compute(int ring) {
    vector<float>::iterator first_range = ranges_diff.begin() + MAX(ring - MAX_RANGES_USED/2, 0);
    vector<float>::iterator end_range   = ranges_diff.begin() + MIN(ring + MAX_RANGES_USED/2, VelodyneSpecification::RINGS);
    float result = -deviation(first_range, end_range);
    updateMinMax(result);
    return result;
  }

private:
  vector<float> ranges_diff;
  static const int MAX_RANGES_USED = 7;
};

class GroundProbabilityByElevationDiff {
public:
  GroundProbabilityByElevationDiff() : cells_processed(0) {
  }

  float compute(const Eigen::Vector3f &current_point) {
    float output;
    Eigen::Vector3f current_vector = (current_point - last_point);
    current_vector.normalize();
    if(cells_processed >= 2) {
      output = current_vector.dot(last_vector);
      if(output < 0.0) {
        output = (last_point - current_point).dot(last_vector);
      }
    } else {
      output = 1.0;
    }
    last_vector = current_vector;
    last_point = current_point;
    cells_processed++;
    return output;
  }
private:
  Eigen::Vector3f last_vector;
  Eigen::Vector3f last_point;
  int cells_processed;
};

class GroundProbabilityByRingDist {
public:
  GroundProbabilityByRingDist() : last_range(-1.0) {
  }

  float compute(const Eigen::Vector3f &current_point, int ring) {
    float current_range = sqrt(current_point.x()*current_point.x() + current_point.z()*current_point.z());
    float ret_val;
    if(last_range > 0) {
      float expected_diff = fabs(VelodyneSpecification::getExpectedRange(ring) - VelodyneSpecification::getExpectedRange(ring+1));
      float found_diff = fabs(last_range-current_range);
      ret_val = gauss(0.0, 1.0, MAX(expected_diff - found_diff, 0), false);
    } else {
      ret_val = 1.0;
    }
    last_range = current_range;
    return ret_val;
  }
private:
  float last_range;
};

vector<float> groundSegmentationByRings(const VelodynePointCloud &cloud) {
  PolarGridOfClouds::POLAR_SUPERBINS = 360;
  PolarGridOfClouds::BIN_SUBDIVISION = 1;
  PolarGridOfClouds polar_grid(cloud);
  PolarGridOfClouds::Ptr summary = polar_grid.summarize();

  map<CellId, float> normalized_probabilities;
  map<CellId, float> height_probabilities;
  map<CellId, float> hdev_probabilities;

  // probabilities requiring normalization:
  GroundProbabilityByHeight prob_height;
  GroundProbabilityByHeightDev prob_hdev;

  for(int polar = 0; polar < PolarGridOfClouds::getPolarBins(); polar++) {
    // normalized probabilities:
    GroundProbabilityByElevationDiff prob_elevation;
    GroundProbabilityByRingDist prob_rdist;
    prob_hdev.setData(summary, polar);
    for(int ring = VelodyneSpecification::RINGS-1; ring >= 0; ring--) {
      if(!summary->at(CellId(polar, ring)).empty()) {
        Eigen::Vector3f current_point = summary->at(CellId(polar, ring)).front().getVector3fMap();
        normalized_probabilities[CellId(polar, ring)] =
            prob_elevation.compute(current_point)*
            prob_rdist.compute(current_point, ring);
        hdev_probabilities[CellId(polar, ring)] = prob_hdev.compute(ring);
        height_probabilities[CellId(polar, ring)] = prob_height.compute(current_point);
      }
    }
  }

  map<CellId, float>::iterator height_prob_it = height_probabilities.begin();
  map<CellId, float>::iterator hdev_prob_it = hdev_probabilities.begin();
  for(int i = 0; i < normalized_probabilities.size();
      i++, height_prob_it++, hdev_prob_it++) {
    height_prob_it->second = prob_height.normalize(height_prob_it->second);
    hdev_prob_it->second = prob_hdev.normalize(hdev_prob_it->second);
  }

  const vector<CellId>& polar_indices = polar_grid.getIndices();
  vector<float> resulting_probabilities;
  for(vector<CellId>::const_iterator cell_id = polar_indices.begin(); cell_id < polar_indices.end(); cell_id++) {
    resulting_probabilities.push_back(
        normalized_probabilities[*cell_id]*
        height_probabilities[*cell_id]*
        hdev_probabilities[*cell_id]);
  }
  return resulting_probabilities;
}

class NormalizedFeatureInRegularGrid : public NormalizedFeature {
public:
  virtual float compute(const VelodynePointCloud &cell_pts) =0;
};

class GroundProbabilityByDevInCell : public NormalizedFeatureInRegularGrid {
public:
  virtual float compute(const VelodynePointCloud &cell_pts) {
    vector<float> heights;
    for(VelodynePointCloud::const_iterator pt = cell_pts.begin(); pt < cell_pts.end(); pt++) {
      heights.push_back(pt->y);
    }
    float prob = -deviation(heights.begin(), heights.end());
    updateMinMax(prob);
    return prob;
  }
};

vector<float> segmentationRegularPolarGrid(
    const VelodynePointCloud &cloud,
    NormalizedFeatureInRegularGrid &feature_esimator) {
  PolarGridOfClouds::POLAR_SUPERBINS = 72;
  PolarGridOfClouds::BIN_SUBDIVISION = 1;
  PolarGridOfClouds grid(cloud, true);

  map<CellId, float> hdist_probabilities;
  for(int polar = 0; polar < PolarGridOfClouds::getPolarBins(); polar++) {
    for(int ring = VelodyneSpecification::RINGS-1; ring >= 0; ring--) {
      CellId cell_id(polar, ring);
      const VelodynePointCloud &cell_content = grid.at(cell_id);
      if(!cell_content.empty()) {
        hdist_probabilities[cell_id] = feature_esimator.compute(cell_content);
      }
    }
  }

  for(map<CellId, float>::iterator cell_p = hdist_probabilities.begin(); cell_p != hdist_probabilities.end(); cell_p++) {
    cell_p->second = feature_esimator.normalize(cell_p->second);
  }

  const vector<CellId>& polar_indices = grid.getIndices();
  vector<float> resulting_probabilities;
  for(vector<CellId>::const_iterator cell_id = polar_indices.begin(); cell_id < polar_indices.end(); cell_id++) {
    resulting_probabilities.push_back(hdist_probabilities[*cell_id]);
  }

  return resulting_probabilities;
}

class ScatteredProbabilityInCell : public NormalizedFeatureInRegularGrid {
public:
  virtual float compute(const VelodynePointCloud &cell_pts) {
    Eigen::Matrix3f covaraince;
    Eigen::Vector4f centroid;
    compute3DCentroid(cell_pts, centroid);
    computeCovarianceMatrix(cell_pts, centroid, covaraince);
    Eigen::Vector3cf eigenvalues = covaraince.eigenvalues();
    float e1 = std::real(eigenvalues(0));
    float e2 = std::real(eigenvalues(1));
    float e3 = std::real(eigenvalues(2));
    float largest  = MAX(MAX(e1, e2), e3);
    float smallest = MIN(MIN(e1, e2), e3) / largest;
    updateMinMax(smallest);
    return smallest;
  }
};

void printHistogram(const Mat &matrix, const string &name) {
  const int BINS = 20;
  vector<int> histogram(BINS, 0);
  double min, max;
  minMaxLoc(matrix, &min, &max);
  double bin_size = (max - min) / BINS;
  for(int i = 0; i < matrix.rows*matrix.cols; i++) {
    double val = matrix.at<float>(i);
    int bin = (val - min) / (max - min) * (BINS - 1);
    histogram[bin]++;
  }
  cout << "============================================================" << endl;
  cout << name << endl;
  for(int i = 0; i < BINS; i++) {
    cout << (max - min) / (BINS - 1) * i + min << ": " << histogram[i] << endl;
  }
  cout << "============================================================" << endl;
}

void joinProbabilities(const vector<float> &prob1, const vector<float> &prob2, vector<float> &out) {
  for(int i = 0; i < prob1.size(); i++) {
    out.push_back(prob1[i]*prob2[i]);
  }
}

bool parse_arguments(int argc, char **argv,
                     GroundDetectionDataGenerator::Parameters &ground_params,
                     vector<string> &clouds_to_process,
										 Eigen::Affine3f &init_transform);

int main(int argc, char** argv) {
  GroundDetectionDataGenerator::Parameters ground_params;
  vector<string> clouds_to_process;
  Eigen::Affine3f init_transform;
  if(!parse_arguments(argc, argv, ground_params, clouds_to_process, init_transform)) {
    return EXIT_FAILURE;
  }

  for(vector<string>::iterator filename = clouds_to_process.begin(); filename < clouds_to_process.end(); filename++) {
    VelodynePointCloud new_cloud;
    VelodynePointCloud::fromFile(*filename, new_cloud);
    transformPointCloud(new_cloud, new_cloud, init_transform);

    /* Uncomment for fake ground annotation:
    vector<float> prob_from_rings = groundSegmentationByRings(new_cloud);
    GroundProbabilityByDevInCell height_deviation_estimator;
    vector<float> prob_from_polar_grid = segmentationRegularPolarGrid(new_cloud, height_deviation_estimator);
    vector<float> joined_prob;
    joinProbabilities(prob_from_rings, prob_from_polar_grid, joined_prob);*/

    /* Not used for fake ground annotation:
    ScatteredProbabilityInCell scatter_estimator;
    scatter_estimator.setRelativeMinIndex(0.001);
    scatter_estimator.setRelativeMaxIndex(0.999);
    vector<float> scattered_prob = segmentationRegularPolarGrid(new_cloud, scatter_estimator);*/

    GroundDetectionDataGenerator data_generator(new_cloud,
                                                boost::filesystem::path(*filename).stem().string(),
                                                ground_params);
    map<string, Mat> data;
    data["y"] = data_generator.getMatrixOf(GroundDetectionDataGenerator::Y);
    data["range"] = data_generator.getMatrixOf(GroundDetectionDataGenerator::RANGE);
    data["intensity"] = data_generator.getMatrixOf(GroundDetectionDataGenerator::INTENSITY);
    //data_generator.getGroundLabels(joined_prob, data["ground_prob"], data["ground_labels"]);

    vector<string> labels;
    labels.push_back("y"); labels.push_back("range"); labels.push_back("intensity");
    vector<Mat> matrices;
    for(vector<string>::iterator label = labels.begin(); label < labels.end(); label++) {
    	matrices.push_back(data[*label]);
    }
    data_generator.saveData(matrices, labels);

    /*for(map<string, Mat>::iterator m = data.begin(); m != data.end(); m++) {
        data_generator.saveData(m->second, m->first);
        if(m->first != "ground_labels" && m->first != "ground_prob") {
        	printHistogram(m->second, m->first);
				}
    }*/
  }
  return EXIT_SUCCESS;
}

bool parse_arguments(int argc, char **argv,
                     GroundDetectionDataGenerator::Parameters &ground_params,
                     vector<string> &clouds_to_process,
										 Eigen::Affine3f &init_transform) {
  bool use_kalman = false;
  int linear_estimator = 3;

  float rx, ry, rz;

  po::options_description desc("Data labeling for ground detection.\n"
      "======================================\n"
//      " * Reference(s): Velas et al, ???? 201?\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("ground_thresh", po::value<float>(&ground_params.threshold)->default_value(ground_params.threshold),
          "Threshold for ground detection")
      ("polar_bins", po::value<int>(&ground_params.polar_bins)->default_value(ground_params.polar_bins),
          "Number of angular polar bins")
			("labels_output", po::value<string>(&ground_params.labels_output)->default_value(ground_params.labels_output),
					"Output directory of labeled data")
			("save_visualization", po::value<bool>(&ground_params.save_visualization)->default_value(ground_params.save_visualization),
					"Output also images")
			("rx", po::value<float>(&rx)->default_value(0.0), "Initial rotation around X axis [rad]")
			("ry", po::value<float>(&ry)->default_value(0.0), "Initial rotation around Y axis [rad]")
			("rz", po::value<float>(&rz)->default_value(0.0), "Initial rotation around Z axis [rad]")
   ;

    po::variables_map vm;
    po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
    po::store(parsed, vm);
    clouds_to_process = po::collect_unrecognized(parsed.options, po::include_positional);

    if (vm.count("help") || clouds_to_process.size() < 1)
    {
        std::cerr << desc << std::endl;
        return false;
    }

    try
    {
        po::notify(vm);
    }
    catch(std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
        return false;
    }

    init_transform = getTransformation(0, 0, 0, rx, ry, rz);

    return true;
}
