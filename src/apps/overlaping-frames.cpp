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

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <cv.h>
#include <opencv2/opencv.hpp>
#include <cxeigen.hpp>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
                     vector<Eigen::Affine3f> &poses,
                     SensorsCalibration &calibration,
                     vector<string> &clouds_to_process,
                     int &frames_cumulated,
                     int &lines_generated,
                     int &lines_preserved) {
  string pose_filename, sensor_poses_filename, skip_filename;

  po::options_description desc("Overlap by Collar Lines Matching\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
      ("sensor_poses,s", po::value<string>(&sensor_poses_filename)->default_value(""), "Sensors calibration file.")
      ("frames_cumulated,f", po::value<int>(&frames_cumulated)->default_value(10), "frames_cumulated")
      ("lines_generated,g", po::value<int>(&lines_generated)->default_value(2), "lines_generated")
      ("lines_preserved,k", po::value<int>(&lines_preserved)->default_value(1), "lines_preserved (kept)")
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

void buildLineClouds(VelodyneFileSequence &cloud_sequence,
    const vector<Eigen::Affine3f> &poses,
    const SensorsCalibration &calibration,
    const int frames_cumulated,
    const int lines_per_cell_gen, const int lines_per_cell_preserve,
    vector<LineCloud> &line_clouds,
    vector< KdTreeFLANN<PointXYZ> > &line_trees) {
  int superframes_cnt = cloud_sequence.size()/frames_cumulated;
  line_clouds.resize(superframes_cnt);
  line_trees.resize(superframes_cnt);

  CollarLinesFilter filter(lines_per_cell_preserve);
  for(int superframe_i = 0; superframe_i < superframes_cnt; superframe_i++) {
    for(int frame_i = 0; frame_i < frames_cumulated; frame_i++) {
      const VelodyneMultiFrame multiframe = cloud_sequence.getNext();
      const PolarGridOfClouds polar_grid(multiframe.clouds, calibration);
      LineCloud line_cloud(polar_grid, lines_per_cell_gen, filter);
      line_cloud.transform(poses[frame_i+superframe_i*frames_cumulated].matrix());
      line_clouds[superframe_i] += line_cloud;
    }
    line_trees[superframe_i].setInputCloud(line_clouds[superframe_i].line_middles.makeShared());
  }
}

float getDistance(const PointCloud<PointXYZ> &cloud, const KdTreeFLANN<PointXYZ> &tree) {
  float distance = 0;
  vector<int> idx_found(1);
  vector<float> dist_found(1);
  for(PointCloud<PointXYZ>::const_iterator p = cloud.begin(); p < cloud.end(); p++) {
    tree.nearestKSearch(*p, 1, idx_found, dist_found);
    distance += sqrt(dist_found.front());
  }
  return distance / cloud.size();
}

int main(int argc, char** argv) {

  vector<string> filenames;
  vector<Eigen::Affine3f> poses;
  SensorsCalibration calibration;
  int lines_generated, lines_preserved, frames_cumulated;

  if(!parse_arguments(argc, argv,
      poses, calibration, filenames, frames_cumulated,
      lines_generated, lines_preserved)) {
    return EXIT_FAILURE;
  }

  VelodyneFileSequence sequence(filenames, calibration);

  vector<LineCloud> line_clouds;
  vector< KdTreeFLANN<PointXYZ> > line_trees;
  buildLineClouds(sequence, poses, calibration, frames_cumulated,
      lines_generated, lines_preserved, line_clouds, line_trees);

  cv::Mat distances(line_clouds.size(), line_clouds.size(), CV_32FC1);
  for(int i = 0; i < line_clouds.size(); i++) {
    distances.at<float>(i, i) = 0.0;
    for(int j = 0; j < i; j++) {
      float dist = getDistance(line_clouds[i].line_middles, line_trees[j]);
      distances.at<float>(i, j) = distances.at<float>(j, i) = dist;
      cout << "(" << i*frames_cumulated << "," << j*frames_cumulated << "): " << dist << endl;
    }
  }

  cv::FileStorage fs("distances.yaml", cv::FileStorage::WRITE);
  fs << "distances" << distances;

  cv::normalize(distances, distances, 0.0, 255.0, cv::NORM_MINMAX);
  cv::Mat distances_gray;
  distances.convertTo(distances_gray, CV_8UC1);
  cv::imwrite("distances.png", distances_gray);

  return EXIT_SUCCESS;
}
