/*
 * Visualization of KITTI poses file.
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 17/06/2015
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
#include <cmath>
#include <algorithm>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/PolarGridOfClouds.h>
#include <but_velodyne/LineCloud.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <boost/program_options.hpp>

#include <Eigen/Eigenvalues>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

namespace po = boost::program_options;

typedef PointXYZ Keypoint;
typedef PointCloud<PointXYZ> Keypoints;

class PolarHeightMap {
public:
  PolarHeightMap(const PointCloud<PointXYZ> &cloud, int radial_bins_, float max_radius, int angular_bins_) :
    radial_bins(radial_bins_),
    angular_bins(angular_bins_),
    data(radial_bins*angular_bins) {
    float radial_bin_size = max_radius/radial_bins;
    for(PointCloud<PointXYZ>::const_iterator p = cloud.begin(); p < cloud.end(); p++) {
      int ai = PolarGridOfClouds::getPolarBinIndex(*p, angular_bins);
      Eigen::Vector3f horizontal_projection = p->getVector3fMap();
      horizontal_projection(1) = 0;
      int ri = floor(horizontal_projection.norm() / radial_bin_size);
      if(ri < radial_bins) {
        data[ai*radial_bins + ri].push_back(p->y);
      } else {
        cerr << "Warning: skipping point " << horizontal_projection << endl;
      }
    }
  }

  void getStatsFromBin(int ri, int ai, float &mean, float &variance) {
    vector<float> &bin = data[ai*radial_bins + ri];
    if(bin.size() < MIN_POINTS_PER_BIN) {
      mean = variance = NAN;
    } else {
      mean = accumulate(bin.begin(), bin.end(), 0.0) / bin.size();
      variance = 0;
      for(vector<float>::iterator h = bin.begin(); h < bin.end(); h++) {
        variance += pow(*h - mean, 2);
      }
      variance /= bin.size();
    }
  }

private:
  int radial_bins;
  int angular_bins;
  vector< vector<float> > data;

  static const int MIN_POINTS_PER_BIN = 3;
};

class KeypointFeatures {
public:
  Eigen::Vector3f smallest_eigenvector;
  float planarity, cylindricality;
  PointCloud<PointXYZ>::Ptr neightbourhood;
  bool neighbourhood_remaped;
  PointXYZ coords;

  KeypointFeatures() {}

  KeypointFeatures(const PointXYZ &keypoint, PointCloud<PointXYZ>::Ptr neightbourhood) {
    set(keypoint, neightbourhood);
  }

  void set(const PointXYZ &keypoint, PointCloud<PointXYZ>::Ptr neightbourhood_) {
    neightbourhood = neightbourhood_;
    coords = keypoint;
    neighbourhood_remaped = false;
    Eigen::Vector4f centroid;
    Eigen::Matrix3f covariance;
    computeMeanAndCovarianceMatrix(*neightbourhood, covariance, centroid);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    Eigen::Vector3f l = solver.eigenvalues();
    planarity = 2 * (l(1)-l(0)) / l.sum();
    cylindricality = (l(2)-l(1)) / l.sum();

    smallest_eigenvector = solver.eigenvectors().col(0);
  }

  bool good() {
    if(planarity > .9) {
      return false;
    }
    static const Eigen::Vector3f vertical(0, 1, 0);
    float angle_with_vertical = RAD2DEG(fabs(acos(vertical.dot(smallest_eigenvector))));
    while(angle_with_vertical > 90.0) {
      angle_with_vertical = 180.0 - angle_with_vertical;
    }
    return angle_with_vertical > 10.0;
  }

  void getRawDescriptor(cv::Mat &raw_desc, int radial_bins, float radius, int angular_bins) {
    assert(raw_desc.cols == radial_bins*angular_bins*2+2);
    if(!neighbourhood_remaped) {
      neighbourhood_remaped = true;
      float remap_yaw = atan2(smallest_eigenvector(2), smallest_eigenvector(0));
      Eigen::Affine3f remap =
          getTransformation(0, 0, 0, 0, remap_yaw, 0) *
          getTransformation(-coords.x, -coords.y, -coords.z, 0, 0, 0);
      transformPointCloud(*neightbourhood, *neightbourhood, remap);
    }
    PolarHeightMap polar_map(*neightbourhood, radial_bins, radius, angular_bins);
    float last_mean = 0.0;
    float last_variance = 0.0;
    for(int ai = 0; ai < angular_bins; ai++) {
      for(int ri = 0; ri < radial_bins; ri++) {
        float mean, variance;
        polar_map.getStatsFromBin(ri, ai, mean, variance);
        if(!isnan(mean) && !isnan(variance)) {
          last_mean = mean;
          last_variance = variance;
        }
        raw_desc.at<float>((ai*radial_bins + ri)*2) = last_mean;
        raw_desc.at<float>((ai*radial_bins + ri)*2 + 1) = last_variance;
      }
    }
    raw_desc.at<float>(radial_bins*angular_bins*2) = planarity;
    raw_desc.at<float>(radial_bins*angular_bins*2 + 1) = cylindricality;
  }
};

PointCloud<PointXYZ>::Ptr get_flatten(const PointCloud<PointXYZ>::Ptr cloud) {
  PointCloud<PointXYZ>::Ptr flat(new PointCloud<PointXYZ>);
  *flat += *cloud;
  for(PointCloud<PointXYZ>::iterator f = flat->begin(); f < flat->end(); f++) {
    f->y = 0;
  }
  return flat;
}

PointCloud<PointXYZ>::Ptr get_neighbourhood(const PointCloud<PointXYZ>::Ptr cloud, const KdTreeFLANN<PointXYZ> &index,
    PointXYZ keypoint, float radius) {
  keypoint.y = 0;
  vector<int> indices;
  vector<float> distances;
  index.radiusSearch(keypoint, radius, indices, distances);
  PointCloud<PointXYZ>::Ptr neightbourhood(new PointCloud<PointXYZ>);
  copyPointCloud(*cloud, indices, *neightbourhood);
  return neightbourhood;
}

void getSampledKeypoints(PointCloud<PointXYZ>::Ptr cloud, Keypoints &keypoints,
    float resolution, float downsampling) {
  pcl::VoxelGrid<Keypoint> grid;
  grid.setLeafSize(resolution, resolution, resolution);
  grid.setInputCloud(cloud);
  grid.filter(keypoints);

  Keypoints keypoints_offset;
  Eigen::Affine3f offset = getTransformation(resolution/2, resolution/2, resolution/2, 0, 0, 0);
  transformPointCloud(*cloud, *cloud, offset);
  grid.setInputCloud(cloud);
  grid.filter(keypoints_offset);
  transformPointCloud(keypoints_offset, keypoints_offset, offset.inverse());
  transformPointCloud(*cloud, *cloud, offset.inverse());
  keypoints += keypoints_offset;

  random_shuffle(keypoints.begin(), keypoints.end());
  keypoints.resize(keypoints.size()*downsampling);
}

void getRawFeatures(PointCloud<PointXYZ>::Ptr cloud, Keypoints &keypoints, cv::Mat &descriptors,
    float resolution, float downsampling, float radius, int radial_bins, int angular_bins) {
  getSampledKeypoints(cloud, keypoints, resolution, downsampling);

  KdTreeFLANN<PointXYZ> index;
  PointCloud<PointXYZ>::Ptr flat_cloud = get_flatten(cloud);
  index.setInputCloud(flat_cloud);
  vector<KeypointFeatures> kpParams(keypoints.size());
  for(int i = 0; i < keypoints.size(); i++) {
    PointCloud<PointXYZ>::Ptr neightbourhood = get_neighbourhood(cloud, index, keypoints[i], radius);
    kpParams[i].set(keypoints[i], neightbourhood);
  }

  assert(kpParams.size() == keypoints.size());
  Keypoints::iterator kp = keypoints.begin();
  vector<KeypointFeatures>::iterator param = kpParams.begin();
  while(kp != keypoints.end()) {
    if(param->good()) {
      param++;
      kp++;
    } else {
      param = kpParams.erase(param);
      kp = keypoints.erase(kp);
    }
  }

  descriptors = cv::Mat(kpParams.size(), radial_bins*angular_bins*2+2, CV_32FC1);
  for(int i = 0; i < kpParams.size(); i++) {
    cv::Mat row = descriptors.row(i);
    kpParams[i].getRawDescriptor(row, radial_bins, radius, angular_bins);
  }
}

bool parse_arguments(int argc, char **argv,
    string &poses_file,
    string &output_keypoints_file,
    string &output_features_file,
    vector<string> &clouds_to_process) {
  po::options_description desc("3D gestalt features extraction\n"
      "======================================\n"
      " * Reference(s): ???\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("poses,p", po::value<string>(&poses_file)->required(), "KITTI poses .txt file")
      ("out_keypoints,k", po::value<string>(&output_keypoints_file)->required(), "Output .pcd file for keypoints")
      ("out_descriptors,d", po::value<string>(&output_features_file)->required(), "Output .yaml file for descriptors")
   ;

    po::variables_map vm;
    po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
    po::store(parsed, vm);
    clouds_to_process = po::collect_unrecognized(parsed.options, po::include_positional);

    if (vm.count("help") || clouds_to_process.size() < 1) {
        std::cerr << desc << std::endl;
        return false;
    }
    try {
        po::notify(vm);
    } catch(std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
        return false;
    }
    return true;
}

int main(int argc, char** argv) {
  string poses_file;
  string output_keypoints_file;
  string output_features_file;
  vector<string> filenames;

  parse_arguments(argc, argv, poses_file, output_keypoints_file, output_features_file, filenames);

  srand(time(NULL));

  vector<Eigen::Affine3f> poses = KittiUtils::load_kitti_poses(poses_file);

  VelodynePointCloud sum_cloud;
  //Visualizer3D vis;
  for (int i = 0; i < filenames.size(); i++) {
    VelodynePointCloud cloud;
    VelodynePointCloud::fromFile(filenames[i], cloud);
    if (!poses.empty()) {
      transformPointCloud(cloud, cloud, poses[i]);
    }
    sum_cloud += cloud;
  }
  PointCloud<PointXYZ> keypoints;
  cv::Mat descriptors;
  getRawFeatures(sum_cloud.getXYZCloudPtr(), keypoints, descriptors, 0.4, 0.1, 1.0, 4, 8);
  //vis.addPointCloud(keypoints).show();

  io::savePCDFileBinary(output_keypoints_file, keypoints);
  cv::FileStorage fs(output_features_file, cv::FileStorage::WRITE);
  fs << "raw_gestalt_descriptors" << descriptors;

  return EXIT_SUCCESS;
}
