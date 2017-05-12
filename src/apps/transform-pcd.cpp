#include <iostream>

#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/Visualizer3D.h>

using namespace std;
using namespace but_velodyne;
namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
    string &input_file, string &output_file,
    Eigen::Affine3f &pose) {
  string pose_file;
  string transformation;
  po::options_description desc("Transform point cloud\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("input_cloud,i", po::value<string>(&input_file)->required(), "Input point cloud file *.pcd")
    ("output_cloud,o", po::value<string>(&output_file)->required(), "Output point cloud file *.pcd")
    ("pose_file,p", po::value<string>(&pose_file)->default_value(""), "Poses in KITTI format")
    ("transformation,t", po::value<string>(&transformation)->default_value(""), "Comma separated values: tx,ty,tz,rx,ry,rz")
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

    if(!transformation.empty()) {
      std::vector<float> t_params;
      std::stringstream ss(transformation);
      float p;
      while (ss >> p) {
        t_params.push_back(p);
        if (ss.peek() == ',')
            ss.ignore();
      }
      pose = pcl::getTransformation(t_params[0], t_params[1], t_params[2],
                                    t_params[3], t_params[4], t_params[5]);
    } else if(!pose_file.empty()) {
      pose = KittiUtils::load_kitti_poses(pose_file).front();
    } else {
      std::cerr << "ERROR, pose or transformation must be set!" << endl
          << desc << std::endl;
      return false;
    }

    return true;
}

int main (int argc, char** argv) {

  string in_filename, out_filename;
  Eigen::Affine3f pose;
  if(!parse_arguments(argc, argv, in_filename, out_filename, pose)) {
    return EXIT_FAILURE;
  }

  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::io::loadPCDFile(in_filename, cloud);
  pcl::transformPointCloud(cloud, cloud, pose);
  cerr << pose.matrix() << endl;
  pcl::io::savePCDFileBinary(out_filename, cloud);
  cerr << "Output saved to " << out_filename << endl;

  return (0);
}
