/*
 * Detection of moving objects.
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 18/02/2016
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
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>

#include <boost/program_options.hpp>
#include <boost/math/distributions/normal.hpp>
#include <cv.h>
#include <highgui.h>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/EigenUtils.h>
#include <but_velodyne/LineCloud.h>
#include <but_velodyne/PolarGridOfClouds.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/Visualizer2D.h>
#include <but_velodyne/Regular2DGridGenerator.h>
#include <but_velodyne/Regular2DGridProcessor.h>
#include <but_velodyne/KittiUtils.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

namespace po = boost::program_options;


#define log cerr

template <class PointType>
typename pcl::PointCloud<PointType>::Ptr downsampleCloud(
                typename pcl::PointCloud<PointType>::Ptr input,
                double resolution = 0.005f) {

    pcl::VoxelGrid<PointType> vg;
    typename pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType>());
    vg.setInputCloud (input);
    vg.setLeafSize (resolution, resolution, resolution);
    vg.filter (*cloud_filtered);

    return cloud_filtered;
}

const float GROUND_CELL = 0.0f;

bool isGround(float cell_height) {
  return cell_height == GROUND_CELL;
}

const float EMPTY_CELL = NAN;

bool isEmpty(float cell_height) {
  return isnan(cell_height);
}

bool isOccupied(float cell_height) {
  return !isGround(cell_height) && !isEmpty(cell_height);
}

class HeightGridCreator : public Modificator<PointCloud<PointXYZ>, float> {
public:
  HeightGridCreator(float mean_threshold_, float variance_threshold_, float delta_threshold_) :
    mean_threshold(mean_threshold_), variance_threshold(variance_threshold_), delta_threshold(delta_threshold_) {
  }

  virtual void operator()(const PointCloud<PointXYZ> &cloud, float &height_mean) {
    if(cloud.empty()) {
      height_mean = EMPTY_CELL;
    } else {
      Eigen::Vector4f centroid;
      Eigen::Matrix3f covariance;
      PointXYZ min, max;
      compute3DCentroid(cloud, centroid);
      computeCovarianceMatrix(cloud, centroid, covariance);
      getMinMax3D(cloud, min, max);
      float delta = max.y - min.y;
      centroid.y() = VelodyneSpecification::KITTI_HEIGHT - centroid.y();
      //cerr << "centroid: " << centroid << "; covariance: " << covariance << endl;
      if(centroid.y() < mean_threshold && covariance(1,1) < variance_threshold && delta < delta_threshold) {
        height_mean = GROUND_CELL;
      } else {
        height_mean = centroid.y();
      }
    }
  }

private:
  float mean_threshold;
  float variance_threshold;
  float delta_threshold;
};

class GridAvg : public Aggregator<float> {
public:
  virtual void operator()(const vector< boost::shared_ptr<float> > &cells, float &avg) {
    avg = 0.0;
    int count = 0;
    for(int i = 0; i < cells.size(); i++) {
      if(isOccupied(*cells[i])) {
        avg += *cells[i];
        count++;
      }
    }
    avg /= count;
  }
};

class MoveDetection {
public:

  class Parameters {
  public:
    Parameters(
        int lines_generated_ = 100,
        int points_per_cell_ = 400,
        float height_mean_threshold_ = 0.50,
        float height_variance_threshold_ = 0.1,
        float delta_threshold_ = 0.2,
        int history_size_ = 10,
        int spatial_cell_dist_tolerance_ = 2,
        float value_diff_rel_tolerance_ = 0.2) :
          lines_generated(lines_generated_),
          points_per_cell(points_per_cell_),
          height_mean_threshold(height_mean_threshold_),
          height_variance_threshold(height_variance_threshold_),
          height_delta_threshold(delta_threshold_),
          history_size(history_size_),
          spatial_cell_dist_tolerance(spatial_cell_dist_tolerance_),
          value_diff_rel_tolerance(value_diff_rel_tolerance_) {
    }

  public:
    int lines_generated;
    int points_per_cell;
    float height_mean_threshold;
    float height_variance_threshold;
    float height_delta_threshold;
    int history_size;
    int spatial_cell_dist_tolerance;
    float value_diff_rel_tolerance;
  };

  MoveDetection(Parameters params_,
                AngularCollarLinesFilter &filter_,
                Regular2DGridGenerator::Parameters grid_params) :
    params(params_),
    filter(filter_),
    grid_generator(grid_params) {
  }

  // TODO return 2D bool grid
  void run(const VelodynePointCloud &new_cloud, Eigen::Affine3f last_odometry) {
    PolarGridOfClouds polar_grid(new_cloud);
    filter.addNewMaxRingRanges(new_cloud.getMaxOfRingRanges());
    LineCloud lines(polar_grid, params.lines_generated, filter);
    PointCloud<PointXYZ>::Ptr dense_cloud = lines.generateDenseCloud(params.points_per_cell);
    PointCloud<PointXYZ>::Ptr dense_downsampled_cloud = downsampleCloud<pcl::PointXYZ>(dense_cloud, 0.1);
    //Visualizer3D().addCloudColoredByHeight(*dense_downsampled_cloud).addRingColoredCloud(new_cloud).show();
    *dense_downsampled_cloud += *new_cloud.getXYZCloudPtr();

    //Visualizer3D vis;
    //vis.addCloudColoredByHeight(*dense_downsampled_cloud).show();
    //removeGroundPlane(dense_downsampled_cloud);
    //vis.keepOnlyClouds(0).addCloudColoredByHeight(*dense_downsampled_cloud).show();

    Regular2DGrid< PointCloud<PointXYZ> > new_grid(grid_generator.params.rows, grid_generator.params.cols);
    grid_generator.generate(*dense_downsampled_cloud, new_grid);
    //Visualizer3D().addEvenOddColoredGrid(new_grid).show();

    Regular2DGrid<float>::Ptr new_grid_height(new Regular2DGrid<float>(grid_generator.params.rows, grid_generator.params.cols));
    HeightGridCreator height_avg(params.height_mean_threshold, params.height_variance_threshold, params.height_delta_threshold);
    Regular2DGridProcessor::modify(new_grid, *new_grid_height, height_avg);
    Visualizer2D(cv::Rect(0, 0, grid_generator.params.cols, grid_generator.params.rows), "Current")
        .addSenzor(grid_generator.params.getSensorPosition())
        .addHeightMap(*new_grid_height).show(100, 2.0);

    updateHistoryPose(last_odometry);

    Regular2DGrid<float> map_grid(grid_generator.params.rows, grid_generator.params.cols);
    PointCloud<PointXYZ>::Ptr history_cloud(new PointCloud<PointXYZ>);
    squashHistory(map_grid);
    Visualizer2D(cv::Rect(0, 0, grid_generator.params.cols, grid_generator.params.rows), "Map")
        .addSenzor(grid_generator.params.getSensorPosition())
        .addHeightMap(map_grid).show(100, 2.0);

    Regular2DGrid<float>::Ptr diff = gridDifference(map_grid, *new_grid_height);
    Visualizer2D(cv::Rect(0, 0, grid_generator.params.cols, grid_generator.params.rows), "Move")
        .addSenzor(grid_generator.params.getSensorPosition())
        .addHeightMap(*diff).show(100, 2.0);

    addFrame(new_grid_height);
  }

protected:
  void removeGroundPlaneRansac(PointCloud<PointXYZ>::Ptr cloud) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.3);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
      PCL_ERROR("Could not estimate a planar model for the given dataset.");
    }

    pcl::ExtractIndices<PointXYZ> eifilter;
    eifilter.setIndices(inliers);
    eifilter.setNegative(true);
    eifilter.filterDirectly(cloud);
  }

  void addFrame(Regular2DGrid<float>::Ptr &grid) {
    last_frames.push_back(grid);
    if(last_frames.size() > params.history_size) {
      last_frames.erase(last_frames.begin());
    }
  }

  void updateHistoryPose(Eigen::Affine3f last_odometry) {
    Eigen::Affine3f inverse = last_odometry.inverse();
    for(vector<Regular2DGrid<float>::Ptr>::iterator h = last_frames.begin(); h < last_frames.end(); h++) {
      transformGrid(**h, **h, inverse, grid_generator.params);
    }
  }

  void squashHistory(Regular2DGrid<float> &output) {
    static GridAvg average;
    Regular2DGridProcessor::aggregate(last_frames, output, average);
  }

  Regular2DGrid<float>::Ptr gridDifference(const Regular2DGrid<float> &map_grid,
                                           const Regular2DGrid<float> &new_grid) {
    Regular2DGrid<float>::Ptr diff(new Regular2DGrid<float>(new_grid.rows, new_grid.cols));
    for(int c = 0; c < new_grid.cols; c++) {
      for(int r = 0; r < new_grid.rows; r++) {
        float min_diff = INFINITY;
        float new_val = *new_grid.at(r, c);
        if(isOccupied(new_val)) {
          for(int map_c = c-params.spatial_cell_dist_tolerance; map_c < c+params.spatial_cell_dist_tolerance; map_c++) {
            for(int map_r = r-params.spatial_cell_dist_tolerance; map_r < r+params.spatial_cell_dist_tolerance; map_r++) {
              if(map_c < 0 || map_r < 0) {
                continue;
              } else if(map_c >= map_grid.cols || map_r >= map_grid.rows) {
                break;
              } else {
                float map_val = *map_grid.at(map_r, map_c);
                min_diff = (isEmpty(map_val) || isEmpty(new_val)) ? -INFINITY : MIN(min_diff, fabs(map_val - new_val));
              }
            }
          }
          *diff->at(r,c) = (min_diff < params.value_diff_rel_tolerance*fabs(new_val)) ? EMPTY_CELL : min_diff;
        } else {
          *diff->at(r,c) = EMPTY_CELL;
        }
      }
    }
    return diff;
  }

  void gridToMatrix(const Regular2DGrid<float> &src, Mat &target) {
    for(int r = 0; r < src.rows; r++) {
      for(int c = 0; c < src.cols; c++) {
        target.at<float>(r, c) = *src.at(r, c);
      }
    }
  }

  void matrixToGrid(const Mat &src, Regular2DGrid<float> &target) {
    for(int r = 0; r < src.rows; r++) {
      for(int c = 0; c < src.cols; c++) {
        *target.at(r, c) = src.at<float>(r, c);
      }
    }
  }

  void transform3Dto2D(const Eigen::Affine3f &t,
                       Mat &transform2D,
                       const Point2f &grid_origin,
                       float resolution) {
    //cerr << "original: " << t.matrix() << endl;
    //cerr << "grid origin: " << grid_origin << endl;
    float x, y, z, roll, pitch, yaw;
    getTranslationAndEulerAngles(t, x, y, z, roll, pitch, yaw);
    Mat rotation = getRotationMatrix2D(grid_origin, radToDeg(pitch), 1.0);
    Mat offset = (Mat_<double>(2,3) << 0, 0, x/resolution, 0, 0, z/resolution);
    //cerr << "2D rot: " << rotation << endl;
    //cerr << "offset: " << offset << endl;
    transform2D = rotation + offset;
    //cerr << "2D: " << transform2D << endl;
  }

  void transformGrid(const Regular2DGrid<float> &src,
                     Regular2DGrid<float> &target,
                     const Eigen::Affine3f &t,
                     const Regular2DGridGenerator::Parameters grid_params) {
    Mat matrix(src.rows, src.cols, CV_32FC1);
    gridToMatrix(src, matrix);
    Point2f grid_origin = grid_params.getSensorPosition();
    grid_origin.x = grid_params.cols - grid_origin.x;
    Mat transform2D;
    transform3Dto2D(t, transform2D, grid_origin, grid_params.resolution);
    warpAffine(matrix, matrix, transform2D, matrix.size(), INTER_NEAREST);
    matrixToGrid(matrix, target);
  }

  Parameters params;
  AngularCollarLinesFilter &filter;
  Regular2DGridGenerator grid_generator;
  vector<Regular2DGrid<float>::Ptr> last_frames;
};

bool parse_arguments(int argc, char **argv, MoveDetection::Parameters &parameters,
                     AngularCollarLinesFilter::Parameters &filter_parameters,
                     vector<string> &clouds_to_process,
                     string &pose_file, string &image_dir);

/*void testGridTransformation(const VelodynePointCloud &cloud) {
  Regular2DGridGenerator::Parameters grid_params;
  Regular2DGridGenerator generator(grid_params);

  Regular2DGrid< PointCloud<PointXYZ> > original_grid(grid_params.rows, grid_params.cols);
  generator.generate(*cloud.getXYZCloudPtr(), original_grid);
  Regular2DGrid<float> original_height_grid(grid_params.rows, grid_params.cols);
  HeightGridCreator height_avg(0.30, 0.03);
  Regular2DGridProcessor::modify(original_grid, original_height_grid, height_avg);

  float x = 10;
  float y = 0;
  float z = 20;
  float roll = degToRad(0);
  float pitch = degToRad(30);
  float yaw = degToRad(0);
  Eigen::Affine3f t;
  getTransformation(x, y, z, roll, pitch, yaw, t);

  PointCloud<PointXYZ> transformed_cloud;
  transformPointCloud(*cloud.getXYZCloudPtr(), transformed_cloud, t);
  Regular2DGrid< PointCloud<PointXYZ> > grid_from_transformed(grid_params.rows, grid_params.cols);
  generator.generate(transformed_cloud, grid_from_transformed);
  Regular2DGrid<float> height_grid_from_transformed(grid_params.rows, grid_params.cols);
  Regular2DGridProcessor::modify(grid_from_transformed, height_grid_from_transformed, height_avg);

  Regular2DGrid<float> transformed_height_grid(grid_params.rows, grid_params.cols);
  transformGrid(original_height_grid, transformed_height_grid, t, grid_params);

  Visualizer2D(cv::Rect(0, 0, generator.params.cols, generator.params.rows), "Original")
      .addSenzor(generator.params.getSensorPosition())
      .addHeightMap(original_height_grid).show(100, 2.0);
  Visualizer2D(cv::Rect(0, 0, generator.params.cols, generator.params.rows), "From transformed")
      .addSenzor(generator.params.getSensorPosition())
      .addHeightMap(height_grid_from_transformed).show(100, 2.0);
  Visualizer2D(cv::Rect(0, 0, generator.params.cols, generator.params.rows), "Transformed grid")
      .addSenzor(generator.params.getSensorPosition())
      .addHeightMap(transformed_height_grid).show(0, 2.0);
}*/

float gauss(float mean, float sd, float x) {
  float exponent = (x - mean) / sd;
  exponent *= -exponent / 2.0;
  return exp(exponent);// / (sd * sqrt(2*M_PI));
}

void groundSegmentationRangeObsolete(const VelodynePointCloud &cloud) {
  vector<float> ring_ranges = cloud.getMaxOfRingRanges();
  vector<float> p_ground;
  float min_prob = INFINITY;
  float max_prob = -INFINITY;
  float sigma = 1;    // 2*sigma = most of the mass
  bool flag = false;
  for(VelodynePointCloud::const_iterator p = cloud.begin(); p < cloud.end(); p++) {
    float range = sqrt(p->x*p->x + p->z*p->z);
    float range_diff = MAX(ring_ranges[p->ring] - range, 0.0);
    float prob = range_diff;
    p_ground.push_back(prob);
    min_prob = MIN(min_prob, prob);
    max_prob = MAX(max_prob, prob);
  }

  PointCloud<PointXYZRGB>::Ptr color_cloud(new PointCloud<PointXYZRGB>);
  for(int i = 0; i < cloud.size(); i++) {
    uchar r, g, b;
    Visualizer3D::colorizeIntensity((p_ground[i]-min_prob)/(max_prob-min_prob), r, g, b);
    PointXYZRGB pt(r, g, b);
    pt.x = cloud[i].x;
    pt.y = cloud[i].y;
    pt.z = cloud[i].z;
    color_cloud->push_back(pt);
  }
  static Visualizer3D vis;
  vis.keepOnlyClouds(0).addColorPointCloud(color_cloud).show();
}

class NormalizedGroundFeature {
public:
  NormalizedGroundFeature() :
    sorted(true) {
  }

  virtual ~NormalizedGroundFeature() {
  }

  float normalize(float value) {
    if(!sorted) {
      std::sort(values.begin(), values.end());
      sorted = true;
    }
    float min = values[values.size()*0.05];
    float max = values[values.size()*0.95];
    if(value > max) {
      return max;
    } else if(value < min) {
      return min;
    }
    return (value - min) / (max - min);
  }

protected:
  void updateMinMax(float value) {
    sorted = false;
    values.push_back(value);
  }

  vector<float> values;
  bool sorted;
};

class GroundProbabilityByHeight : public NormalizedGroundFeature {
public:
  float compute(const Eigen::Vector3f &current_point) {
    updateMinMax(current_point.y());
    return current_point.y();
  }
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
      ret_val = gauss(0.0, 1.0, MAX(expected_diff - found_diff, 0));
    } else {
      ret_val = 1.0;
    }
    last_range = current_range;
    return ret_val;
  }
private:
  float last_range;
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

void getColorsForProbability(float prob, const Eigen::Vector3f &current_point, uchar &r, uchar &g, uchar &b) {
  float current_range = sqrt(current_point.x()*current_point.x() + current_point.z()*current_point.z());
  if(current_range < 30.0) {
    Visualizer3D::colorizeIntensity(prob, r, g, b);
  } else {
    r = g = b = 125;
  }
}

void groundSegmentation(const VelodynePointCloud &cloud) {
  PolarGridOfClouds::POLAR_SUPERBINS = 360;
  PolarGridOfClouds::BIN_SUBDIVISION = 1;
  PolarGridOfClouds polar_grid(cloud);
  PolarGridOfClouds::Ptr summary = polar_grid.summarize();

  vector<float> normalized_probabilities;
  vector<float> height_probabilities;

  // probabilities requiring normalization:
  GroundProbabilityByHeight prob_height;

  for(int polar = 0; polar < PolarGridOfClouds::getPolarBins(); polar++) {
    // normalized probabilities:
    GroundProbabilityByElevationDiff prob_elevation;
    GroundProbabilityByRingDist prob_rdist;
    for(int ring = VelodyneSpecification::RINGS-1; ring >= 0; ring--) {
      if(!summary->at(CellId(polar, ring)).empty()) {
        Eigen::Vector3f current_point = summary->at(CellId(polar, ring)).front().getVector3fMap();
        normalized_probabilities.push_back(
            prob_elevation.compute(current_point)*
            prob_rdist.compute(current_point, ring));
        height_probabilities.push_back(prob_height.compute(current_point));
      }
    }
  }

  PointCloud<PointXYZRGB>::Ptr ground_map(new PointCloud<PointXYZRGB>);
  int prob_index = 0;
  for(int polar = 0; polar < PolarGridOfClouds::getPolarBins(); polar++) {
    for(int ring = VelodyneSpecification::RINGS-1; ring >= 0; ring--, prob_index++) {
      uchar red, green, blue;
      Eigen::Vector3f current_point = summary->at(CellId(polar, ring)).front().getVector3fMap();
      getColorsForProbability(
          normalized_probabilities[prob_index]*
          prob_height.normalize(height_probabilities[prob_index]), current_point, red, green, blue);
      *ground_map += *Visualizer3D::colorizeCloud(polar_grid[CellId(polar, ring)], red, green, blue);
    }
  }

  static Visualizer3D vis;
  vis.keepOnlyClouds(0).addColorPointCloud(ground_map).show();
}

/**
 * ./move-detection cloud.bin
 */
int main(int argc, char** argv) {

  MoveDetection::Parameters parameters;
  AngularCollarLinesFilter::Parameters filter_parameters;
  vector<string> clouds_to_process;
  string pose_filename;
  string image_dir;
  if(!parse_arguments(argc, argv, parameters, filter_parameters, clouds_to_process, pose_filename, image_dir)) {
    return EXIT_FAILURE;
  }

  std::vector<Eigen::Affine3f> poses = KittiUtils::load_kitti_poses(pose_filename);

  AngularCollarLinesFilter filter(CollarLinesFilter::HORIZONTAL_RANGE_DIFF, filter_parameters);
  Regular2DGridGenerator::Parameters grid_params;
  MoveDetection move_detection(parameters, filter, grid_params);

  std::vector<Eigen::Affine3f>::iterator pose = poses.begin();
  for(vector<string>::iterator filename = clouds_to_process.begin(); filename < clouds_to_process.end(); filename++, pose++) {
    VelodynePointCloud new_cloud;
    log << "Processing KITTI file: " << *filename << endl << flush;
    if (filename->find(".pcd") != string::npos) {
      io::loadPCDFile(*filename, new_cloud);
    } else {
      VelodynePointCloud::fromKitti(*filename, new_cloud);
    }
    Eigen::Affine3f last_transformation;
    if(filename == clouds_to_process.begin()) {
      last_transformation = Eigen::Affine3f::Identity();
    } else {
      last_transformation = (pose-1)->inverse() * (*pose);
    }

    groundSegmentation(new_cloud);
    continue;

    move_detection.run(new_cloud, last_transformation);

    boost::filesystem::path p(filename->c_str());
    Mat image = imread(image_dir + "/" + p.stem().string() + ".png");
    imshow("Image", image);
    waitKey(100);
  }

  return EXIT_SUCCESS;
}

bool parse_arguments(int argc, char **argv, MoveDetection::Parameters &parameters, AngularCollarLinesFilter::Parameters &filter_parameters,
                     vector<string> &clouds_to_process, string &pose_file, string &image_dir) {
  bool use_kalman = false;
  int linear_estimator = 3;

  po::options_description desc("Detection of moving objects\n"
      "======================================\n"
//      " * Reference(s): Velas et al, ???? 201?\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("lines_generated,g", po::value<int>(&parameters.lines_generated)->default_value(parameters.lines_generated),
          "How many collar lines are generated per single polar bin")
      ("lines_preserved,p", po::value<int>(&filter_parameters.lines_to_preserve)->default_value(filter_parameters.lines_to_preserve),
          "How many collar lines are preserved per single polar bin after filtering")
      ("points_per_cell", po::value<int>(&parameters.points_per_cell)->default_value(parameters.points_per_cell),
          "How many points are generated within each polar bin")
      ("polar_superbins", po::value<int>(&PolarGridOfClouds::POLAR_SUPERBINS)->default_value(PolarGridOfClouds::POLAR_SUPERBINS),
          "Number of polar bins in the grid")
      ("bin_subdivision", po::value<int>(&PolarGridOfClouds::BIN_SUBDIVISION)->default_value(5),
          "How many times is the polar bin sub-divided")
      ("max_line_horizontal_diff", po::value<float>(&filter_parameters.max_horizontal_range_diff)->default_value(filter_parameters.max_horizontal_range_diff),
          "Max difference of horizontal ranges for preserved line")
      ("line_horizontal_diff_rel_tolerance", po::value<float>(&filter_parameters.horizontal_range_diff_tolerance_rel)->default_value(filter_parameters.horizontal_range_diff_tolerance_rel),
          "Relative tolerance of line horizontal range difference")
      ("line_horizontal_diff_abs_tolerance", po::value<float>(&filter_parameters.horizontal_range_diff_tolerance_abs)->default_value(filter_parameters.horizontal_range_diff_tolerance_abs),
          "Absolute tolerance of line horizontal range difference")
      ("horizontal_range_diff_weight", po::value<float>(&filter_parameters.weight_of_expected_horizontal_range_diff)->default_value(filter_parameters.weight_of_expected_horizontal_range_diff),
          "Weight of horizontal range difference estimation (value < 0 => MAX)")
      ("poses", po::value<string>(&pose_file)->required(),
          "KITTI pose file")
      ("images", po::value<string>(&image_dir)->required(),
          "Directory of KITTI images for given sequence")
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

    return true;
}

