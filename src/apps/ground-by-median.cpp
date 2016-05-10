#include <cstdlib>
#include <cstdio>
#include <libgen.h>

#include <pcl/common/eigen.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>

#include <Eigen/Eigenvalues>

#include <boost/program_options.hpp>
#include <boost/math/distributions/normal.hpp>

#include <cv.h>
#include <opencv2/highgui.hpp>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/PolarGridOfClouds.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/EigenUtils.h>
#include <but_velodyne/GroundDetectionDataGenerator.h>

namespace po = boost::program_options;
using namespace std;
using namespace but_velodyne;

class CellIndex {
public:
  int x, y;

  CellIndex(int x_, int y_) : x(x_), y(y_) {
  }

  bool operator<(const CellIndex &other) const {
    return (this->y != other.y) ? (this->y < other.y) : (this->x < other.x);
  }
};

typedef map<CellIndex, VelodynePointCloud> GridOfClouds;

template <typename PointT>
CellIndex toGridCoordinates(const PointT &pt, float cell_size) {
  return CellIndex(pt.x / cell_size, pt.y / cell_size);
}

void splitInto(const VelodynePointCloud &cloud, float cell_size, GridOfClouds &grid) {
  for(VelodynePointCloud::const_iterator pt = cloud.begin(); pt < cloud.end(); pt++) {
    CellIndex index = toGridCoordinates(*pt, cell_size);
    grid[index].push_back(*pt);
  }
}

string toFileName(CellIndex index, string output_dir) {
  stringstream ss;
  ss << output_dir << "/cell_" << index.x << "_" << index.y << ".yaml";
  return ss.str();
}

void accumulate(const GridOfClouds &grid, string output_dir) {
  // TODO
}

SparseMat medianFilter(string output_dir) {
  // TODO
}

bool parse_arguments(int argc, char **argv,
                     vector<string> &clouds_to_process,
		     string &poses_file,
		     float &cell_size,
		     string &output_dir);

int main(int argc, char *argv[]) {
  vector<string> clouds_to_process;
  string output_dir;
  string poses_file;
  float cell_size = 20;

  parse_arguments(argc, argv, clouds_to_process, poses_file, cell_size, output_dir);

  vector<Eigen::Affine3f> poses = KittiUtils::load_kitti_poses(poses_file);
  for(int i = 0; i < clouds_to_process.size(); i++) {
    VelodynePointCloud cloud;
    VelodynePointCloud::fromKitti(clouds_to_process[i], cloud);
    transformPointCloud(cloud, cloud, poses[i]);
    GridOfClouds cloud_grid;
    splitInto(cloud, cell_size, cloud_grid);
    accumulate(cloud_grid, output_dir);
  }
  SparseMat map = medianFilter(output_dir);
  return EXIT_SUCCESS;
}

bool parse_arguments(int argc, char **argv,
                     vector<string> &clouds_to_process,
		     string &poses_file,
		     float &cell_size,
		     string &output_dir) {

  po::options_description desc("Ground detection by median filtering.\n"
      "======================================\n"
//      " * Reference(s): Velas et al, ???? 201?\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("output_dir", po::value<string>(&output_dir)->default_value("."),
          "Output directory.")
      ("poses_file", po::value<string>(&poses_file)->required(),
	  "KITTI poses file.")
      ("cell_size", po::value<float>(&cell_size)->default_value(cell_size),
	  "Size of cell in the regular grid.")
  ;

  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line (argc, argv, desc);
  po::store (parsed, vm);
  clouds_to_process = po::collect_unrecognized (parsed.options,
						po::include_positional);

  if (vm.count ("help") || clouds_to_process.size () < 1) {
    std::cerr << desc << std::endl;
    return false;
  }

  try {
    po::notify (vm);
  }
  catch (std::exception& e) {
    std::cerr << "Error: " << e.what () << std::endl << std::endl << desc
	<< std::endl;
    return false;
  }

  return true;
}
