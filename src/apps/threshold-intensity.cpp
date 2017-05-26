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

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
    float &threshold, float &radius,
    PointCloud<PointXYZI> &cloud, PointCloud<PointXYZI> &avg_cloud, string &out_filename) {
  string cloud_filename, avg_cloud_filename;
  po::options_description desc("Collar Lines Registration of Velodyne scans\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("radius,r", po::value<float>(&radius)->default_value(10.0), "Radius.")
      ("threshold,t", po::value<float>(&threshold)->default_value(5), "Threshold.")
      ("cloud,c", po::value<string>(&cloud_filename)->required(), "Point cloud.")
      ("avg_cloud,a", po::value<string>(&avg_cloud_filename)->default_value(""), "Average point cloud.")
      ("output,o", po::value<string>(&out_filename)->default_value(""), "Out cloud.")
  ;
  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);

  if (vm.count("help")) {
      std::cerr << desc << std::endl;
      return false;
  }
  try {
      po::notify(vm);
  } catch(std::exception& e) {
      std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
      return false;
  }

  io::loadPCDFile(cloud_filename, cloud);
  if(!avg_cloud_filename.empty()) {
    io::loadPCDFile(avg_cloud_filename, avg_cloud);
  }

  return true;
}

void average_intensities(const PointCloud<PointXYZI>::Ptr cloud,
    PointCloud<PointXYZI> &avg_cloud, float radius) {
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(cloud);
  vector<int> indices;
  vector<float> distances;
  avg_cloud += *cloud;
  for(int pi = 0; pi < cloud->size(); pi++) {
    kdtree.radiusSearch(cloud->at(pi), radius, indices, distances);
    float avg_intensity = 0.0;
    for(vector<int>::iterator i = indices.begin(); i < indices.end(); i++) {
      avg_intensity += cloud->at(*i).intensity;
    }
    avg_intensity /= indices.size();
    avg_cloud[pi].intensity = avg_intensity;
  }
}

void threshold_cloud(const PointCloud<PointXYZI> &cloud, float threshold, const PointCloud<PointXYZI> &avg_cloud,
    PointCloud<PointXYZI> &output, PointCloud<PointXYZI> &removed) {
  for(int i = 0; i < cloud.size(); i++) {
    if(avg_cloud[i].intensity > threshold) {
      output.push_back(cloud[i]);
    } else {
      removed.push_back(cloud[i]);
    }
  }
}

int main(int argc, char** argv) {

  float threshold, radius;
  PointCloud<PointXYZI>::Ptr cloud(new PointCloud<PointXYZI>);
  PointCloud<PointXYZI> out_cloud, removed, avg_cloud;
  string out_filename;
  if(!parse_arguments(argc, argv, threshold, radius, *cloud, avg_cloud, out_filename)) {
    return EXIT_FAILURE;
  }

  if(avg_cloud.empty()) {
    average_intensities(cloud, avg_cloud, radius);
  }
  threshold_cloud(*cloud, threshold, avg_cloud, out_cloud, removed);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud = Visualizer3D::colorizeCloud(out_cloud, true);
  Visualizer3D vis;
  vis.getViewer()->setBackgroundColor(0, 0, 0);
  vis.setColor(0, 0, 255).addPointCloud(removed).addColorPointCloud(rgb_cloud).show();

  if(!out_filename.empty()) {
    io::savePCDFileBinary(out_filename, out_cloud);
    io::savePCDFileBinary(out_filename + ".rgb.pcd", *rgb_cloud);
    io::savePCDFileBinary(out_filename + ".avg.pcd", avg_cloud);
  }

  return EXIT_SUCCESS;
}
