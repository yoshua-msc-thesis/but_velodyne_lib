#include <iostream>

#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>

#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/Visualizer3D.h>

using namespace std;
using namespace but_velodyne;
namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
    string &source_file, string &target_file) {

  po::options_description desc("GICP\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("source_cloud,s", po::value<string>(&source_file)->required(), "Source point cloud file *.pcd")
    ("target_cloud,t", po::value<string>(&target_file)->required(), "Target point cloud file *.pcd")
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

    return true;
}

int main (int argc, char** argv) {

  string src_filename, trg_filename;
  if(!parse_arguments(argc, argv, src_filename, trg_filename)) {
    return EXIT_FAILURE;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile(src_filename, *src_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr trg_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile(trg_filename, *trg_cloud);

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
  gicp.setInputSource(src_cloud);
  gicp.setInputTarget(trg_cloud);
  pcl::PointCloud<pcl::PointXYZ> Final;
  gicp.align(Final);
  std::cerr << "has converged:" << gicp.hasConverged() << " score: " <<
  gicp.getFitnessScore() << std::endl;

  Eigen::Matrix4f t = gicp.getFinalTransformation().inverse();
  std::cerr << t << std::endl;
  KittiUtils::printPose(cout, t);

  return (0);
}
