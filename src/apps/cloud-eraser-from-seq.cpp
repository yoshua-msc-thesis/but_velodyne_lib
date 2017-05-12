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

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>
#include <pcl/filters/random_sample.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

class CloudEraser {
public:
  CloudEraser(const vector<PointCloud<PointXYZI>::Ptr> &clouds_) :
    clouds(clouds_) {
    preserve_mask.resize(clouds.size(), 1);
    visualizer.getViewer()->setBackgroundColor(0.0, 0.0, 0.0);
    visualizer.getViewer()->registerAreaPickingCallback(&CloudEraser::pickPointsCallback, *this);
    visualizer.getViewer()->registerKeyboardCallback(&CloudEraser::keyCallback, *this);
  }

  vector<int> run() {
    visualizeCloud();
    visualizer.show();
    return preserve_mask;
  }

protected:

  void visualizeCloud() {
    PointCloud<PointXYZRGB>::Ptr cloud_colored(new PointCloud<PointXYZRGB>);
    point_src_idx.clear();
    for(int i = 0; i < preserve_mask.size(); i++) {
      if(preserve_mask[i] != 0) {
        if(preserve_mask[i] > 0) {
          *cloud_colored += *Visualizer3D::colorizeCloud(*clouds[i], true);
        } else if(preserve_mask[i] < 0) {
          *cloud_colored += *Visualizer3D::colorizeCloud(*clouds[i], 200, 0, 0);
        }
        point_src_idx.resize(point_src_idx.size()+clouds[i]->size(), i);
      }
    }
    visualizer.keepOnlyClouds(0).addColorPointCloud(cloud_colored);
  }

  void pickPointsCallback(const pcl::visualization::AreaPickingEvent& event, void*) {
    vector<int> indices;
    if(event.getPointsIndices(indices)) {
      for(vector<int>::iterator i = indices.begin(); i < indices.end(); i++) {
        preserve_mask[point_src_idx[*i]] = -1;
      }
      visualizeCloud();
    }
  }

  void keyCallback(const pcl::visualization::KeyboardEvent &event, void*) {
    if(event.keyDown()) {
      if(event.getKeySym() == "d") {
        for(vector<int>::iterator i = preserve_mask.begin(); i < preserve_mask.end(); i++) {
          *i = MAX(*i, 0);
        }
        visualizeCloud();
      }
    }
  }

private:
  vector<PointCloud<PointXYZI>::Ptr> clouds;
  Visualizer3D visualizer;
  vector<int> preserve_mask;
  vector<int> point_src_idx;
};

bool parse_arguments(int argc, char **argv,
                     float &sampling_ratio,
                     vector<Eigen::Affine3f> &poses,
                     vector<string> &clouds_to_process,
                     string &output_file) {
  string pose_filename;

  po::options_description desc("Collar Lines Registration of Velodyne scans\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
      ("sampling_ratio,s", po::value<float>(&sampling_ratio)->default_value(0.1), "Reduce size with this ratio.")
      ("output_file,o", po::value<string>(&output_file)->required(), "Output PCD file")
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
  if(!pose_filename.empty()) {
    poses = KittiUtils::load_kitti_poses(pose_filename);
  }
  return true;
}

void subsample_cloud(PointCloud<PointXYZI>::Ptr cloud, float sampling_ratio) {
  pcl::RandomSample<PointXYZI> subsampling;
  subsampling.setInputCloud(cloud);
  subsampling.setSample(cloud->size()*sampling_ratio);
  subsampling.filter(*cloud);
}

int main(int argc, char** argv) {

  float sampling_ratio;
  vector<Eigen::Affine3f> poses;
  vector<string> filenames;
  string output_file;
  if(!parse_arguments(argc, argv, sampling_ratio, poses, filenames, output_file)) {
    return EXIT_FAILURE;
  }

  vector< PointCloud<PointXYZI>::Ptr > input_clouds;
  for (int cloud_i = 0; cloud_i < filenames.size(); cloud_i++) {
    if(cloud_i >= poses.size()) {
      std::cerr << "No remaining pose for cloud: " << cloud_i << std::endl << std::flush;
      break;
    }
    std::cerr << "Processing file: " << filenames[cloud_i] << std::endl << std::flush;
    PointCloud<PointXYZI>::Ptr cloud(new PointCloud<PointXYZI>);
    pcl::io::loadPCDFile(filenames[cloud_i], *cloud);

    vector<int> indices;
    removeNaNFromPointCloud(*cloud, *cloud, indices);
    subsample_cloud(cloud, sampling_ratio);
    transformPointCloud(*cloud, *cloud, poses[cloud_i]);
    input_clouds.push_back(cloud);
  }

  CloudEraser eraser(input_clouds);
  vector<int> indices = eraser.run();
  for(int i = 0; i < indices.size(); i++) {
    //cerr << indices[i] << " " << KittiUtils::getKittiFrameName(i, ".pcd") << endl << flush;
    if(indices[i] == 0) {
      cout << KittiUtils::getKittiFrameName(i, "") << endl << flush;
    }
  }

  return EXIT_SUCCESS;
}
