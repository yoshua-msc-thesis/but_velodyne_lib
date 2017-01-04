#include <fstream>
#include <iostream>
#include <cstdlib>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <boost/program_options.hpp>

#include <but_velodyne/Stopwatch.h>
#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/GroundDetectionDataGenerator.h>
#include <but_velodyne/Visualizer3D.h>

using namespace std;
using namespace cv;
using namespace cv::dnn;
using namespace pcl;
using namespace but_velodyne;
using namespace velodyne_pointcloud;
namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
                     string &proto_file, string &model_file,
		     bool &visualize, ofstream &out_file,
                     vector<string> &files_to_process) {
  string out_filename;
  po::options_description desc("Ground detection by pretrained CNN.\n"
      "======================================\n"
//      " * Reference(s): Velas et al, ???? 201?\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("prototxt,p", po::value<string>(&proto_file)->required(),
          "File of net prototype")
      ("caffemodel,m", po::value<string>(&model_file)->required(),
          "Pretrained caffe model")
      ("visualize,v", po::bool_switch(&visualize),
	  "Run visualization")
      ("output,o", po::value<string>(&out_filename)->default_value(""),
	  "Output file of ground probabilities")
  ;

  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line (argc, argv, desc);
  po::store (parsed, vm);
  files_to_process = po::collect_unrecognized (parsed.options,
						po::include_positional);

  if (vm.count ("help") || files_to_process.size () < 1) {
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

  if(!out_filename.empty()) {
    out_file.open(out_filename.c_str());
    if(!out_file.is_open()) {
      perror(out_filename.c_str());
      return false;
    }
  }

  return true;
}

int main (int argc, char **argv) {
  vector<string> files_to_process;
  string modelTxt, modelBin;
  bool visualization;
  ofstream out_file;
  if(!parse_arguments(argc, argv, modelTxt, modelBin, visualization, out_file, files_to_process)) {
    return EXIT_FAILURE;
  }

  Ptr<dnn::Importer> importer;
  try {
    importer = dnn::createCaffeImporter (modelTxt, modelBin);
  } catch (const cv::Exception &err) {
    std::cerr << err.msg << std::endl;
  }
  if (!importer) {
    std::cerr << "Can't load network by using the following files: " << std::endl;
    std::cerr << "prototxt:   " << modelTxt << std::endl;
    std::cerr << "caffemodel: " << modelBin << std::endl;
    return EXIT_FAILURE;
  }

  dnn::Net net;
  importer->populateNet (net);
  importer.release ();                     //We don't need importer anymore

  boost::shared_ptr<Visualizer3D> visualizer;
  if(visualization) {
    visualizer.reset(new Visualizer3D);
  }

  for(vector<string>::iterator featFile = files_to_process.begin(); featFile < files_to_process.end(); featFile++) {
    VelodynePointCloud cloud;
    VelodynePointCloud::fromFile(*featFile, cloud);

    GroundDetectionDataGenerator data_generator(cloud);
    vector<Mat> feature_channels(3);
    feature_channels[0] = data_generator.getMatrixOf(GroundDetectionDataGenerator::RANGE);
    feature_channels[1] = data_generator.getMatrixOf(GroundDetectionDataGenerator::Y);
    feature_channels[2] = data_generator.getMatrixOf(GroundDetectionDataGenerator::INTENSITY);

    Stopwatch stopwatch;
    Mat features;
    merge(feature_channels, features);
    dnn::Blob inputBlob = dnn::Blob (features);  //Convert Mat to dnn::Blob image batch
    net.setBlob (".data", inputBlob);        //set the network input
    net.forward ();                          //compute output
    dnn::Blob probBlob = net.getBlob("prob");   //gather output of "prob" layer
    cerr << "forward pass took: " << stopwatch.elapsed() << "s" << endl;

    Mat probNotGround = probBlob.getPlane(0, 0);
    Mat probIsGround = probBlob.getPlane(0, 1);
    Mat ones = Mat::ones(probIsGround.size(), CV_32F);

    Mat probGround = (ones + probIsGround - probNotGround) * 0.5;

    if(visualization) {
      Mat probGroundEqualized;
      normalize(probNotGround, probGroundEqualized, 0, 255, NORM_MINMAX);
      imshow("Ground labels", probGroundEqualized);
      waitKey(100);
    }

    const vector<CellId> &indicies = data_generator.getIndices();
    PointCloud<PointXYZRGB>::Ptr colored_cloud(new PointCloud<PointXYZRGB>);
    for(int i = 0; i < cloud.size(); i++) {
      uchar r, g, b;
      CellId index = indicies[i];
      float prob = probGround.at<float>(index.ring, index.polar);
      Visualizer3D::colorizeIntensity(prob, r, g, b);
      PointXYZIR pt = cloud[i];
      PointXYZRGB colored_pt(r, g, b);
      colored_pt.x = pt.x;
      colored_pt.y = pt.y;
      colored_pt.z = pt.z;
      colored_cloud->push_back(colored_pt);
      if(out_file.is_open()) {
	out_file << prob << endl;
      }
    }
    if(visualization) {
      visualizer->getViewer()->setBackgroundColor(0,0,0);
      visualization::Camera cam;
      cam.clip[0] = 0.183189; cam.clip[1] = 183.189;
      cam.focal[0] = -0.586755; cam.focal[1] = 1.96262; cam.focal[2] = 0.896533;
      cam.fovy = 49.1311/180*M_PI;
      cam.pos[0] = 0.603312; cam.pos[1] = -16.0206; cam.pos[2] = -26.5259;
      cam.view[0] = -0.0253036; cam.view[1] = -0.836462; cam.view[2] = 0.547441;
      cam.window_pos[0] = 0; cam.window_pos[1] = 52;
      cam.window_size[0] = 1600; cam.window_size[1] = 1000;
      visualizer->getViewer()->setCameraParameters(cam);
      visualizer->keepOnlyClouds(0).addColorPointCloud(colored_cloud).show();
      /*static int i = 0;
      stringstream ss;
      ss << "snapshot_" << i++ << ".png";
      visualizer->keepOnlyClouds(0).addColorPointCloud(colored_cloud).saveSnapshot(ss.str());*/
    }
  }

  return EXIT_SUCCESS;
}
