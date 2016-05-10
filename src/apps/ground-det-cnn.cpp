#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <fstream>
#include <iostream>
#include <cstdlib>

using namespace std;
using namespace cv;
using namespace cv::dnn;

/* Find best class for the blob (i. e. class with maximal probability) */
void getMaxClass (dnn::Blob &probBlob, int *classId, double *classProb) {
  Mat probMat = probBlob.matRefConst ().reshape (1, 1); //reshape the blob to 1x1000 matrix
  Point classNumber;
  minMaxLoc (probMat, NULL, classProb, NULL, &classNumber);
  *classId = classNumber.x;
}

int main (int argc, char **argv) {
  String modelTxt = "/home/ivelas/workspace/ivelas-git/sw/cnn-generator/NETS/ground_simple/net_deploy.prototxt";
  String modelBin = "/home/ivelas/workspace/ivelas-git/sw/cnn-generator/NETS/ground_simple/net_snapshot_iter_60000.caffemodel";
  String featFile = "/media/files/kitti/ground_labels/02_normalized/000111.gd_data.yaml.gz";
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

  FileStorage fs(featFile, FileStorage::READ);
  vector<Mat> feature_channels(3);
  fs["range"] >> feature_channels[0];
  fs["y"] >> feature_channels[1];
  fs["intensity"] >> feature_channels[2];
  Mat features;
  merge(feature_channels, features);

  dnn::Net net;
  importer->populateNet (net);
  importer.release ();                     //We don't need importer anymore
  dnn::Blob inputBlob = dnn::Blob (features);  //Convert Mat to dnn::Blob image batch
  net.setBlob (".data", inputBlob);        //set the network input
  net.forward ();                          //compute output
  dnn::Blob prob = net.getBlob("prob");   //gather output of "prob" layer

  cerr << "channels: " << prob.channels() << endl;
  cerr << "dims: " << prob.dims() << endl;
  cerr << "rows: " << prob.rows() << endl;
  cerr << "cols: " << prob.cols() << endl;

  // TODO

  return EXIT_SUCCESS;
}
