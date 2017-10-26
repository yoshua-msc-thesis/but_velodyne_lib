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
#include <but_velodyne/point_types.h>

#include <pcl/PCLPointCloud2.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
                     vector<string> &clouds_to_process) {

  po::options_description desc("Labeled clouds visualization\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
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

  vector<string> filenames;

  if(!parse_arguments(argc, argv, filenames)) {
    return EXIT_FAILURE;
  }

  vector< PointCloud<PointXYZ> > clusters;

  for(vector<string>::const_iterator fn = filenames.begin(); fn < filenames.end(); fn++) {
    PointCloud<LabeledPoint> labeled_cloud;
    io::loadPCDFile(*fn, labeled_cloud);
    for(PointCloud<LabeledPoint>::const_iterator pt = labeled_cloud.begin(); pt < labeled_cloud.end(); pt++) {
      if(clusters.size() <= pt->label) {
        clusters.resize(pt->label+1);
      }
      PointXYZ pt_xyz;
      copyXYZ(*pt, pt_xyz);
      clusters[pt->label].push_back(pt_xyz);
    }
  }

  Visualizer3D().addPointClouds(clusters).show();

  return EXIT_SUCCESS;
}
