#include "calibDS325.h"

using namespace std;
using namespace google;

// definition for gflags
DECLARE_string(color);
DECLARE_string(depth);
DECLARE_string(folder);
DECLARE_string(type);
DECLARE_int32(num);

void loadImages(cv::vector<cv::Mat> &rgb, 
		cv::vector<cv::Mat> &depth, 
		const int &fileNum){
  rgb.clear();
  depth.clear();
  
  for(int i = 0; i < fileNum; ++i){
    stringstream rgbfilename, depthfilename;
    rgbfilename << FLAGS_folder <<"/" << FLAGS_color << i << FLAGS_type;
    depthfilename << FLAGS_folder << "/" << FLAGS_depth << i << FLAGS_type;
    
    cout << "loading : " << rgbfilename.str() << " and " << depthfilename.str() << endl;

    // load RGB image
    cv::Mat tempRGB = cv::imread(rgbfilename.str(), 0);
    rgb.push_back(tempRGB);


    // load depth image
    cv::Mat tempDepth = cv::imread(depthfilename.str(), CV_LOAD_IMAGE_ANYDEPTH);
    tempDepth.convertTo(tempDepth, CV_8U, 255.0/1000.0);
    cv::Mat maxDist = cv::Mat::ones(tempDepth.rows, tempDepth.cols, CV_8U) * MAX_DEPTH;
    cv::Mat minDist = cv::Mat::ones(tempDepth.rows, tempDepth.cols, CV_8U) * MIN_DEPTH;
    cv::min(tempDepth, maxDist, tempDepth);
    tempDepth -= minDist;
    cv::resize(tempDepth, tempDepth, cv::Size(), 2.0,2.0);
    cv::Mat roiTempDepth;

    cv::resize(tempDepth(cv::Rect(40, 43,498,498 / 4 * 3)), roiTempDepth, cv::Size(640, 480));

    depth.push_back(roiTempDepth);

    std::cout << "loaded" << std::endl;

    cv::imshow("rgb",rgb[i]);
    cv::imshow("depth",depth[i]);
    cv::waitKey(100);
  }

}
