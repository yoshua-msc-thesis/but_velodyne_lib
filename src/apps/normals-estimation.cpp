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

#include <boost/program_options.hpp>

#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/common.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>

#include <cv.h>
#include <cxeigen.hpp>
#include <ml.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
using namespace cv;
using namespace cv::ml;

namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
                     vector<Eigen::Affine3f> &poses,
                     SensorsCalibration &calibration,
                     vector<string> &clouds_to_process,
                     float &subsampling_rate) {
  string pose_filename, sensor_poses_filename;

  po::options_description desc("Velodyne Points Clustering\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
    ("sensor_poses,s", po::value<string>(&sensor_poses_filename)->default_value(""), "Sensors calibration file.")
    ("subsampling_rate,r", po::value<float>(&subsampling_rate)->default_value(0.1), "Subsampling rate.")
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

  poses = KittiUtils::load_kitti_poses(pose_filename);

  if(sensor_poses_filename.empty()) {
    calibration = SensorsCalibration();
  } else {
    calibration = SensorsCalibration(sensor_poses_filename);
  }

  return true;
}

float get_rand(float maxval) {
  return rand()/(RAND_MAX/maxval/2)-maxval;
}

void regular_subsampling(PointCloud<PointXYZI>::ConstPtr full_cloud, const float sampling_rate,
    PointIndices::Ptr indices, PointCloud<PointXYZI>::Ptr subsampled_cloud) {
  static const float LEAF_SIZE = 0.2;
  pcl::VoxelGrid<PointXYZI> grid;
  grid.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
  PointCloud<PointXYZI>::Ptr cumulated_grid_clouds(new PointCloud<PointXYZI>);
  srand(time(NULL));
  while(cumulated_grid_clouds->size() < full_cloud->size()*sampling_rate) {
    Eigen::Affine3f t = getTransformation(get_rand(LEAF_SIZE), get_rand(LEAF_SIZE), get_rand(LEAF_SIZE), 0, 0, 0);
    PointCloud<PointXYZI>::Ptr tmp_cloud(new PointCloud<PointXYZI>);
    transformPointCloud(*full_cloud, *tmp_cloud, t);
    grid.setInputCloud(tmp_cloud);
    grid.filter(*tmp_cloud);
    transformPointCloud(*tmp_cloud, *tmp_cloud, t.inverse());
    *cumulated_grid_clouds += *tmp_cloud;
  }

  KdTreeFLANN<PointXYZI> index;
  index.setInputCloud(full_cloud);
  for(PointCloud<PointXYZI>::const_iterator p = cumulated_grid_clouds->begin(); p < cumulated_grid_clouds->end(); p++) {
    vector<int> idxs(1);
    vector<float> dist(1);
    index.nearestKSearch(*p, 1, idxs, dist);
    indices->indices.push_back(idxs.front());
    subsampled_cloud->push_back(full_cloud->at(idxs.front()));
  }
}

void getNormals(const PointCloud<PointXYZI> &subsampled_points,
    const PointCloud<PointXYZI> &original_points,
    const vector<int> origins,
    const PointCloud<PointXYZ> sensor_positions,
    pcl::PointCloud<pcl::Normal> &normals) {
  cerr << "Normal estimation" << endl;

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
  ne.setInputCloud(subsampled_points.makeShared());
  ne.setSearchSurface(original_points.makeShared());

  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.1);
  ne.compute(normals);

  for(int i = 0; i < subsampled_points.size(); i++) {
    Normal &n = normals[i];
    const PointXYZ &p = sensor_positions[origins[i]];
    flipNormalTowardsViewpoint(subsampled_points[i], p.x, p.y, p.z,
        n.normal_x, n.normal_y, n.normal_z);
  }
}

int main(int argc, char** argv) {

  vector<string> filenames;
  vector<Eigen::Affine3f> poses;
  SensorsCalibration calibration;
  float subsampling_rate;

  if(!parse_arguments(argc, argv,
      poses, calibration, filenames,
      subsampling_rate)) {
    return EXIT_FAILURE;
  }

  vector<int> sum_origins;
  PointCloud<PointXYZI>::Ptr sum_cloud(new PointCloud<PointXYZI>);

  VelodyneFileSequence sequence(filenames, calibration);
  for(int frame_i = 0; sequence.hasNext(); frame_i++) {
    VelodyneMultiFrame multiframe = sequence.getNext();
    PointCloud<PointXYZI>::Ptr joined(new PointCloud<PointXYZI>);
    multiframe.joinTo(*joined);
    transformPointCloud(*joined, *joined, poses[frame_i]);
    *sum_cloud += *joined;
    vector<int> origins(joined->size(), frame_i);
    sum_origins.insert(sum_origins.end(), origins.begin(), origins.end());
  }

  vector<int> subsampled_origins;
  PointCloud<PointXYZI>::Ptr subsampled_cloud(new PointCloud<PointXYZI>);
  PointIndices::Ptr indices(new PointIndices);
  regular_subsampling(sum_cloud, subsampling_rate, indices, subsampled_cloud);
  extract_indices(sum_origins, indices->indices, subsampled_origins);

  /*Visualizer3D vis;
  vis.setColor(150, 150, 150).addPointCloud(*subsampled_cloud).show();*/

  PointCloud<Normal> subsampled_normals;
  getNormals(*subsampled_cloud, *sum_cloud,
      subsampled_origins, Visualizer3D::posesToPoints(poses),
      subsampled_normals);

  PointIndices::Ptr filtered_indices(new PointIndices);
  removeNaNNormalsFromPointCloud(subsampled_normals, subsampled_normals, filtered_indices->indices);
  extract_indices(subsampled_cloud, filtered_indices, *subsampled_cloud);
  extract_indices(indices->indices, filtered_indices->indices, indices->indices);

  /*for(int i = 0; i < subsampled_cloud->size(); i+=1000) {
    Eigen::Vector3f start = subsampled_cloud->at(i).getVector3fMap();
    Eigen::Vector3f orient = subsampled_normals[i].getNormalVector3fMap();
    vis.addArrow(PointCloudLine(start, orient));
  }
  vis.show();*/

  io::savePCDFileBinary("normals.pcd", subsampled_normals);
  save_vector(indices->indices, "normals.indices");

  return EXIT_SUCCESS;
}
