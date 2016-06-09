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
#include <pcl/kdtree/kdtree_flann.h>

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
    FillProperties(
	float max_neighbour_height_diff_ = 0.05,
	float max_neighbour_dist_ = 0.1,
	float max_total_height_diff_ = 0.1,
	float max_dist_to_line_ = 0.1) :
	  max_neighbour_height_diff(max_neighbour_height_diff_),
	  max_neighbour_dist(max_neighbour_dist_),
	  max_total_height_diff(max_total_height_diff_),
	  max_dist_to_line(max_dist_to_line_) {
    }
    float max_neighbour_height_diff;
    float max_neighbour_dist;
    float max_total_height_diff;
    float max_dist_to_line;
  } properties;

  Interaction(const VelodynePointCloud &_cloud) :
    cloud(_cloud),
    annotation(cloud.size(), 0),
    first_point_id(-1) {
    cloud.getRings(rings, ring_indices);
    kdtree.setInputCloud(cloud.getXYZCloudPtr());
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

  void pointPicked(PointXYZ pt) {
    std::cerr << "Clicked point " << pt << std::endl;
    vector<int> indices;
    vector<float> distances;
    kdtree.nearestKSearch(pt, 1, indices, distances);

    if(first_point_id < 0) {
      first_point_id = indices.front();
    } else {
      fillRings(first_point_id, indices.front());
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

    int from_ring = MIN(source.ring, target.ring);
    int to_ring = MAX(source.ring, target.ring);
    vector<int> closests_pts = getClosestPts(lsegment, from_ring, to_ring);
    for(int r = from_ring; r <= to_ring; r++) {
      volatile int closest_index = closests_pts[r-from_ring];
      vector<PointXYZIR> &ring = rings[r];
      vector<int> &ring_ids = ring_indices[r];
      if(closest_index >= 0) {
	for(int i = closest_index-1; i >= 0; i--) {
	  if(!checkAndFillNext(r, i, i+1, closest_index)) {
	    break;
	  }
	}
	for(int i = closest_index+1; i < ring.size(); i++) {
	  if(!checkAndFillNext(r, i, i-1, closest_index)) {
	    break;
	  }
	}
      }
    }
  }

  bool checkAndFillNext(int ring, int pt_id, int prev_id, int seed_id) {
    PointXYZIR pt = rings[ring][pt_id];
    PointXYZIR prev = rings[ring][prev_id];
    PointXYZIR seed = rings[ring][seed_id];
    if(pt.y-prev.y < properties.max_neighbour_height_diff &&
	pt.y - seed.y < properties.max_total_height_diff &&
	computeRange(pt - prev) < properties.max_neighbour_dist) {
      annotation[ring_indices[ring][pt_id]] = 1;
      return true;
    } else {
      return false;
    }
  }

  vector<int> getClosestPts(const PointCloudLine &line, int from_ring, int to_ring) {
    vector<int> closest_points;
    for(int r = from_ring; r <= to_ring; r++) {
      vector<PointXYZIR> &ring = rings[r];
      int min_id = -1;
      float min_dist = INFINITY;
      for(int i = 0; i < ring.size(); i++) {
	float dist = line.distanceTo(ring[i]);
	if(dist < min_dist) {
	  min_dist = dist;
	  min_id = i;
	}
      }
      if(min_dist < properties.max_dist_to_line) {
	closest_points.push_back(min_id);
      } else {
	closest_points.push_back(-1);
      }
    }
    return closest_points;
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
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
};

void pointPickedCallback(const pcl::visualization::PointPickingEvent &event, void* cookie) {
  Interaction* interaction = (Interaction*)(cookie);

  float x, y, z;
  event.getPoint(x, y, z);

  interaction->pointPicked(PointXYZ(x, y, z));
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
