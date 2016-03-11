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
#include <cv.h>

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
  return isinf(cell_height);
}

const float EMPTY_CELL = NAN;

bool isEmpty(float cell_height) {
  return isnan(cell_height);
}

class HeightGridCreator : public Modificator<PointCloud<PointXYZ>, float> {
public:
  HeightGridCreator(float mean_threshold_, float variance_threshold_) :
    mean_threshold(mean_threshold_), variance_threshold(variance_threshold_) {
  }

  virtual void operator()(const PointCloud<PointXYZ> &cloud, float &height_mean) {
    if(cloud.empty()) {
      height_mean = EMPTY_CELL;
    } else {
      Eigen::Vector4f centroid;
      Eigen::Matrix3f covariance;
      compute3DCentroid(cloud, centroid);
      computeCovarianceMatrix(cloud, centroid, covariance);
      centroid.y() = VelodyneSpecification::KITTI_HEIGHT - centroid.y();
      //cerr << "centroid: " << centroid << "; covariance: " << covariance << endl;
      if(centroid.y() < mean_threshold && covariance(1,1) < variance_threshold) {
        height_mean = GROUND_CELL;
      } else {
        height_mean = centroid.y();
      }
    }
  }

private:
  float mean_threshold;
  float variance_threshold;
};

void gridToMatrix(const Regular2DGrid<float> &src, Mat &target) {

}

void transform3Dto2D(const Eigen::Affine3f &t, Mat &transform2D) {

}

void transformGrid(const Regular2DGrid<float> &src, Regular2DGrid<float> &target, const Eigen::Affine3f &t) {
  Mat matix;
  gridToMatrix(src, matix);
  Mat transform2D;
  transform3Dto2D(t, transform2D);
}

class MoveDetection {
public:

  class Parameters {
  public:
    Parameters(
        int lines_generated_ = 100,
        int points_per_cell_ = 400,
        float height_mean_threshold_ = 0.30,
        float height_variance_threshold_ = 0.03,
        int history_size_ = 10,
        int spatial_cell_dist_tolerance_ = 2,
        float value_diff_rel_tolerance_ = 0.2) :
          lines_generated(lines_generated_),
          points_per_cell(points_per_cell_),
          height_mean_threshold(height_mean_threshold_),
          height_variance_threshold(height_variance_threshold_),
          history_size(history_size_),
          spatial_cell_dist_tolerance(spatial_cell_dist_tolerance_),
          value_diff_rel_tolerance(value_diff_rel_tolerance_) {
    }

  public:
    int lines_generated;
    int points_per_cell;
    float height_mean_threshold;
    float height_variance_threshold;
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

    Regular2DGrid<float> new_grid_height(grid_generator.params.rows, grid_generator.params.cols);
    HeightGridCreator height_avg(params.height_mean_threshold, params.height_variance_threshold);
    Regular2DGridProcessor::modify(new_grid, new_grid_height, height_avg);
    Visualizer2D(cv::Rect(0, 0, grid_generator.params.cols, grid_generator.params.rows), "Current")
        .addSenzor(grid_generator.getSensorPosition())
        .addHeightMap(new_grid_height).show(100, 2.0);

    updateHistoryPose(last_odometry);
    PointCloud<PointXYZ>::Ptr history_cloud(new PointCloud<PointXYZ>);
    squashHistory(*history_cloud);
    //Visualizer3D().addCloudColoredByHeight(*history_cloud).show();
    Regular2DGrid< PointCloud<PointXYZ> > history_cloud_grid(grid_generator.params.rows, grid_generator.params.cols);
    grid_generator.generate(*history_cloud, history_cloud_grid);
    Regular2DGrid<float> map_grid(grid_generator.params.rows, grid_generator.params.cols);
    Regular2DGridProcessor::modify(history_cloud_grid, map_grid, height_avg);
    Visualizer2D(cv::Rect(0, 0, grid_generator.params.cols, grid_generator.params.rows), "Map")
        .addSenzor(grid_generator.getSensorPosition())
        .addHeightMap(map_grid).show(100, 2.0);

    Regular2DGrid<float>::Ptr diff = gridDifference(map_grid, new_grid_height);
    Visualizer2D(cv::Rect(0, 0, grid_generator.params.cols, grid_generator.params.rows), "Move")
        .addSenzor(grid_generator.getSensorPosition())
        .addHeightMap(*diff).show(100, 2.0);

    addFrame(dense_downsampled_cloud);
  }

protected:
  void removeGroundPlane(PointCloud<PointXYZ>::Ptr cloud) {
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

  void addFrame(PointCloud<PointXYZ>::Ptr &cloud) {
    last_frames.push_back(cloud);
    if(last_frames.size() > params.history_size) {
      last_frames.erase(last_frames.begin());
    }
  }

  void updateHistoryPose(Eigen::Affine3f last_odometry) {
    Eigen::Affine3f inverse = last_odometry.inverse();
    for(vector<PointCloud<PointXYZ>::Ptr>::iterator h = last_frames.begin(); h < last_frames.end(); h++) {
      transformPointCloud(**h, **h, inverse);
    }
  }

  void squashHistory(PointCloud<PointXYZ> &output) {
    for(vector<PointCloud<PointXYZ>::Ptr>::iterator h = last_frames.begin(); h < last_frames.end(); h++) {
      output += **h;
    }
  }

  Regular2DGrid<float>::Ptr gridDifference(const Regular2DGrid<float> &map_grid,
                                           const Regular2DGrid<float> &new_grid) {
    Regular2DGrid<float>::Ptr diff(new Regular2DGrid<float>(new_grid.rows, new_grid.cols));
    for(int c = 0; c < new_grid.cols; c++) {
      for(int r = 0; r < new_grid.rows; r++) {
        float min_diff = INFINITY;
        float new_val = *new_grid.at(r, c);
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
      }
    }
    return diff;
  }

  Parameters params;
  AngularCollarLinesFilter &filter;
  Regular2DGridGenerator grid_generator;
  vector<PointCloud<PointXYZ>::Ptr> last_frames;
};

bool parse_arguments(int argc, char **argv, MoveDetection::Parameters &parameters,
                     AngularCollarLinesFilter::Parameters &filter_parameters,
                     vector<string> &clouds_to_process,
                     string &pose_file);

/**
 * ./move-detection cloud.bin
 */
int main(int argc, char** argv) {

  MoveDetection::Parameters parameters;
  AngularCollarLinesFilter::Parameters filter_parameters;
  vector<string> clouds_to_process;
  string pose_filename;
  if(!parse_arguments(argc, argv, parameters, filter_parameters, clouds_to_process, pose_filename)) {
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

    move_detection.run(new_cloud, last_transformation);
  }

  return EXIT_SUCCESS;
}

bool parse_arguments(int argc, char **argv, MoveDetection::Parameters &parameters, AngularCollarLinesFilter::Parameters &filter_parameters,
                     vector<string> &clouds_to_process, string &pose_file) {
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

