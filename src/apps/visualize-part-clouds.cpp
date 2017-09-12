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
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
                     float &sampling_ratio,
                     vector<Eigen::Affine3f> &poses,
                     SensorsCalibration &calibration,
                     vector<string> &clouds_to_process,
                     string &output_file,
                     vector<bool> &mask) {
  string pose_filename, skip_filename, sensor_poses_filename;

  po::options_description desc("Collar Lines Registration of Velodyne scans\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
      ("sampling_ratio,s", po::value<float>(&sampling_ratio)->default_value(0.1), "Reduce size with this ratio.")
      ("skip_file,k", po::value<string>(&skip_filename)->default_value(""), "File with indices to skip")
      ("sensor_poses", po::value<string>(&sensor_poses_filename)->default_value(""), "Sensor poses (calibration).")
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

  mask.resize(poses.size(), true);
  if(!skip_filename.empty()) {
    ifstream skip_file(skip_filename.c_str());
    string line;
    while(getline(skip_file, line)) {
      mask[atoi(line.c_str())] = false;
    }
  }

  if(!sensor_poses_filename.empty()) {
    calibration = SensorsCalibration(sensor_poses_filename);
  } else {
    calibration = SensorsCalibration();
  }

  return true;
}

void subsample_cloud(PointCloud<PointXYZI>::Ptr cloud, float sampling_ratio) {
  pcl::RandomSample<PointXYZI> subsampling;
  subsampling.setInputCloud(cloud);
  subsampling.setSample(cloud->size()*sampling_ratio);
  subsampling.filter(*cloud);
}


class ErrorHighlighter {
public:
  ErrorHighlighter(const PointCloud<PointXYZRGB>::Ptr &cloud_, const vector<int> &origins_) :
    cloud(cloud_), origins(origins_) {
    visualizer.getViewer()->setBackgroundColor(0.0, 0.0, 0.0);
    visualizer.getViewer()->registerAreaPickingCallback(&ErrorHighlighter::pickPointsCallback, *this);
    visualizer.getViewer()->registerKeyboardCallback(&ErrorHighlighter::keyCallback, *this);
  }

  void run() {
    vector<int> indices;
    sedDataToVisualizer(indices);

    for(vector<int>::const_iterator hi = highlighted_clouds.begin(); hi < highlighted_clouds.end(); hi++) {
      cout << *hi << endl;
    }

    visualizer.show();
  }

protected:

  void sedDataToVisualizer(const vector<int> &indices) {
    for(vector<int>::const_iterator i = indices.begin(); i < indices.end(); i++) {
      highlighted_clouds.push_back(origins[*i]);
    }
    sort(highlighted_clouds.begin(), highlighted_clouds.end());
    highlighted_clouds.erase(unique(highlighted_clouds.begin(), highlighted_clouds.end()), highlighted_clouds.end());

    PointCloud<PointXYZRGB>::Ptr cloud_colored(new PointCloud<PointXYZRGB>);
    *cloud_colored += *cloud;

    for(vector<int>::const_iterator hi = highlighted_clouds.begin(); hi < highlighted_clouds.end(); hi++) {
      cerr << *hi << endl;
      for(int oi = 0; oi < origins.size(); oi++) {
        while(*hi == origins[oi] && oi < origins.size()) {
          cloud_colored->at(oi).r = 255;
          cloud_colored->at(oi).g = cloud_colored->at(oi).b = 0;
          oi++;
        }
      }
    }

    visualizer.addColorPointCloud(cloud_colored);
  }

  void pickPointsCallback(const pcl::visualization::AreaPickingEvent& event, void*) {
    vector<int> indices;
    if(event.getPointsIndices(indices)) {
      sedDataToVisualizer(indices);
    }
  }

  void keyCallback(const pcl::visualization::KeyboardEvent &event, void*) {
    if(event.keyDown()) {
      if(event.getKeySym() == "d") {
      }
    }
  }

private:
  PointCloud<PointXYZRGB>::Ptr cloud;
  Visualizer3D visualizer;
  vector<int> highlighted_clouds;
  const vector<int> origins;
};

int main(int argc, char** argv) {

  vector<string> filenames;
  vector<Eigen::Affine3f> poses;
  SensorsCalibration calibration;
  string output_pcd_file;
  float sampling_ratio;
  vector<bool> mask;

  if(!parse_arguments(argc, argv,
      sampling_ratio,
      poses, calibration, filenames,
      output_pcd_file,
      mask)) {
    return EXIT_FAILURE;
  }

  PointCloud<PointXYZI> sum_cloud;
  PointCloud<PointXYZI>::Ptr cloud(new PointCloud<PointXYZI>);
  VelodyneFileSequence file_sequence(filenames, calibration);
  vector<int> points_origins;
  for (int frame_i = 0; file_sequence.hasNext(); frame_i++) {
    if(frame_i >= poses.size()) {
      std::cerr << "No remaining pose for cloud: " << frame_i << std::endl << std::flush;
      break;
    }

    VelodyneMultiFrame multiframe = file_sequence.getNext();
    if(mask[frame_i]) {
      multiframe.joinTo(*cloud);

      subsample_cloud(cloud, sampling_ratio);
      transformPointCloud(*cloud, *cloud, poses[frame_i]);
      sum_cloud += *cloud;

      vector<int> new_origins(frame_i, cloud->size());
      points_origins.insert(points_origins.end(), new_origins.begin(), new_origins.end());
    } else {
      cout << frame_i << endl;
    }
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud;
  rgb_cloud = Visualizer3D::colorizeCloud(sum_cloud, true);

  ErrorHighlighter highlighter(rgb_cloud, points_origins);
  highlighter.run();

  return EXIT_SUCCESS;
}
