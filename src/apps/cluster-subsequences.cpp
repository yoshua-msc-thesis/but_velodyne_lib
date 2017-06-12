/*
 * Visualization of KITTI poses file.
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 19/04/2017
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
#include <but_velodyne/PolarGridOfClouds.h>
#include <but_velodyne/LineCloud.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

const int TEXT_MARGIN = 3;
const int FONT_TITLE = 11;

bool parse_arguments(int argc, char **argv,
                     vector<Eigen::Affine3f> &poses,
                     vector<string> &clouds_to_process,
                     float &voxel_resolution,
                     int &cluster_count) {
  string pose_filename;

  po::options_description desc("Collar Lines Registration of Velodyne scans\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
      ("voxel_resolution,r", po::value<float>(&voxel_resolution)->default_value(1.0), "Resolution of voxel grid.")
      ("cluster_count,c", po::value<int>(&cluster_count)->default_value(10), "Count of clusters.")
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

void subsampleCloud(PointCloud<PointXYZ>::Ptr cloud, float voxel_resolution) {
  pcl::VoxelGrid<PointXYZ> grid;
  grid.setLeafSize(voxel_resolution, voxel_resolution, voxel_resolution);
  grid.setInputCloud(cloud);
  grid.filter(*cloud);
}

float normalize(float value, float min, float max) {
  return (value-min)/(max-min);
}

class StartEnd {
public:
  int start, end, idx;
  StartEnd(void) :
    start(1000000000), end(-1), idx(0) {
  }
  bool operator <(const StartEnd &other) const {
    return this->start < other.start;
  }
};

int main(int argc, char** argv) {

  vector<string> filenames;
  vector<Eigen::Affine3f> poses;
  float voxel_resolution;
  int cluster_count;
  if(!parse_arguments(argc, argv,
      poses, filenames,
      voxel_resolution,
      cluster_count)) {
    return EXIT_FAILURE;
  }

  PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
  PointCloud<PointXYZRGB>::Ptr centroids(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZ> map;
  centroids->resize(filenames.size());
  for (int cloud_i = 0; cloud_i < filenames.size(); cloud_i++) {
    cloud->clear();
    io::loadPCDFile(filenames[cloud_i], *cloud);
    transformPointCloud(*cloud, *cloud, poses[cloud_i]);
    subsampleCloud(cloud, voxel_resolution);
    map += *cloud;
    Eigen::Vector4f centroid;
    compute3DCentroid(*cloud, centroid);
    PointXYZRGB &pt = centroids->at(cloud_i);
    pt.getVector4fMap() = centroid;
    pt.r = pt.g = 0;
    pt.b = 1.0/filenames.size() * cloud_i * 255;
  }

  PointXYZ min_pt, max_pt;
  getMinMax3D(map, min_pt, max_pt);

  cv::Mat data(filenames.size(), 4, CV_32F);
  for (int i = 0; i < filenames.size(); i++) {
    data.at<float>(i, 0) = normalize(centroids->at(i).x, min_pt.x, max_pt.x);
    data.at<float>(i, 1) = normalize(centroids->at(i).y, min_pt.y, max_pt.y);
    data.at<float>(i, 2) = normalize(centroids->at(i).z, min_pt.z, max_pt.z);
    data.at<float>(i, 3) = normalize(i, 0, filenames.size())*2;
  }
  cv::Mat labels(filenames.size(), 1, CV_32SC1);
  cv::TermCriteria termination(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 1000, 1);
  int attempts = 2;
  cv::kmeans(data, cluster_count, labels, termination, attempts, cv::KMEANS_PP_CENTERS);

  vector< PointCloud<PointXYZ> > clusters(cluster_count);
  vector<StartEnd> cluster_borders(cluster_count);
  PointCloud<PointXYZ> poses_points = Visualizer3D::posesToPoints(poses);
  for (int i = 0; i < labels.cols*labels.rows; i++) {
    int label = labels.at<int>(i);
    clusters[label].push_back(poses_points[i]);
    cluster_borders[label].start = MIN(cluster_borders[label].start, i);
    cluster_borders[label].end = MAX(cluster_borders[label].end, i);
    cluster_borders[label].idx = label;
  }

  Visualizer3D vis;
  cv::RNG& rng = cv::theRNG();
  int text_x = TEXT_MARGIN;
  int text_y = TEXT_MARGIN;
  std::sort(cluster_borders.begin(), cluster_borders.end());

  for(int c = 0; c < cluster_count; c++) {
    cout << cluster_borders[c].start << " " << cluster_borders[c].end << endl;
    stringstream ss;
    ss << "poses_cluster." << c << ".pcd";
    io::savePCDFileBinary(ss.str(), clusters[cluster_borders[c].idx]);

    uchar r = rng(256);
    uchar g = rng(256);
    uchar b = rng(256);
    vis.setColor(r, g, b).addPointCloud(clusters[cluster_borders[c].idx]);
    text_y += FONT_TITLE + TEXT_MARGIN;
    stringstream c_str;
    c_str << "cluster-" << c;
    vis.getViewer()->addText(c_str.str(), text_x, text_y, FONT_TITLE, b/255.0, g/255.0, r/255.0, c_str.str());
  }

  // vis.addColorPointCloud(centroids).show();
  vis.show();

  return EXIT_SUCCESS;
}
