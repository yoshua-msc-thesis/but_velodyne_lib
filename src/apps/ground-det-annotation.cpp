/*
 * Annotation tool for the ground detection in Velodyne data.
 *
 * Based on the example: http://pointclouds.org/documentation/tutorials/pcl_visualizer.php
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 19/09/2014
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

#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/EigenUtils.h>
#include <functional>

using namespace but_velodyne;
using namespace pcl;
using namespace std;
using namespace velodyne_pointcloud;

void pointPickedCallback(const pcl::visualization::PointPickingEvent &event, void* cookie);

class Interaction {
public:
  class FillProperties {
  public:
    float max_neighbour_height_diff = 0.02;
    float max_neighbour_dist = 0.05;
    float max_total_height_diff = 0.05;
  } properties;

  Interaction(const VelodynePointCloud &_cloud) :
    cloud(_cloud),
    annotation(cloud.size(), 0),
    first_point_id(-1) {
    cloud.getRings(rings, ring_indices);
  }

  void run() {
    visualizer.getViewer()->registerPointPickingCallback(pointPickedCallback,
							 (void*)this);
    showAnnotation();
  }

  void saveAnnotationTo(const char *filename) {
    ofstream out_file(filename);
    for(vector<int>::const_iterator a = annotation.begin(); a < annotation.end(); a++) {
      out_file << *a << endl;
    }
  }

  void pointPicked(PointXYZ pt, int id) {
    std::cerr << "Clicked point " << pt << std::endl;

    if(first_point_id < 0) {
      first_point_id = id;
    } else {
      fillRings(first_point_id, id);
      first_point_id = -1;
      showAnnotation();
    }
  }

protected:

  void fillRings(int source_id, int target_id) {
    annotation[source_id] = annotation[target_id] = 1;
    PointXYZIR source = cloud[source_id];
    PointXYZIR target = cloud[target_id];
    PointCloudLine lsegment(source, target);
    visualizer.addLine(lsegment, 1.0, 0, 0);

    vector<int> closests_pts = getClosestPts(lsegment, source.ring, target.ring);
    // TODO
  }

  vector<int> getClosestPts(const PointCloudLine &line, int source_ring, int target_ring) {
    vector<int> closest_points;
    // TODO
  }

  void colorByAnnotation(PointCloud<PointXYZRGB>::Ptr cloud_colored) {
    for(int i = 0; i < cloud.size(); i++) {
      PointXYZRGB color_pt;
      copyXYZ(cloud[i], color_pt);
      color_pt.r = color_pt.g = color_pt.b = 0;
      if(annotation[i] == 0) {
        color_pt.b = 255;
      } else if(annotation[i] == 1) {
        color_pt.r = 255;
      } else {
        color_pt.g = 255;
      }
      cloud_colored->push_back(color_pt);
    }
  }

  void showAnnotation() {
    PointCloud<PointXYZRGB>::Ptr cloud_colored(new PointCloud<PointXYZRGB>);
    colorByAnnotation(cloud_colored);
    visualizer.keepOnlyClouds(0).addColorPointCloud(cloud_colored).show();
  }

private:
  Visualizer3D visualizer;
  const VelodynePointCloud &cloud;
  vector< vector<PointXYZIR> > rings;
  vector< vector<int> > ring_indices;
  vector<int> annotation;
  int first_point_id;
};

void pointPickedCallback(const pcl::visualization::PointPickingEvent &event, void* cookie) {
  Interaction* interaction = (Interaction*)(cookie);

  float x, y, z;
  event.getPoint(x, y, z);
  int id = event.getPointIndex();

  interaction->pointPicked(PointXYZ(x, y, z), id);
}

int main(int argc, char *argv[]) {
  if(argc != 3) {
    cerr << "Invalid arguments, expected: <input-cloud> <annotation-txt-file>" << endl;
    return EXIT_FAILURE;
  }

  VelodynePointCloud cloud;
  VelodynePointCloud::fromFile(argv[1], cloud);

  Interaction interaction(cloud);
  interaction.run();
  interaction.saveAnnotationTo(argv[2]);

  return EXIT_SUCCESS;
}
