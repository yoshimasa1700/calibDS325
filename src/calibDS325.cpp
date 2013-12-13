#include "calibDS325.h"

using namespace std;
using namespace google;

// definition for gflags
DECLARE_string(color);
DECLARE_string(depth);
DECLARE_string(folder);
DECLARE_string(type);
DECLARE_int32(num);

double g_squareSize = 50.0;

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

int findChessboard(cv::vector<cv::Mat> &rgb, cv::vector<cv::Mat> &depth, 
		   cv::vector<cv::vector<cv::vector<cv::Point2f> > > &imagePoints,
		   const cv::Size patternSize,
		   const int &fileNum){
    for(int i = 0; i < fileNum; ++i){
    cout << i << endl;
    
    if( cv::findChessboardCorners( rgb[i], 
				   patternSize, 
				   imagePoints[0][i],
				   CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE 
				   ) && 
	cv::findChessboardCorners( depth[i], 
				   patternSize, 
				   imagePoints[1][i] ,
				   CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE 
				   ) ) {
    

      std::cout << " ... All corners found." << std::endl;

      cv::cornerSubPix(rgb[i], imagePoints[0][i], cv::Size(11,11), cv::Size(-1,-1),
		       cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
					30, 0.01));

      cv::cornerSubPix(depth[i], imagePoints[1][i], cv::Size(11,11), cv::Size(-1,-1),
   		       cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
   					30, 0.01));
      // 検出点を描画する
      cv::drawChessboardCorners( rgb[i], patternSize, ( cv::Mat )( imagePoints[0][i] ), true );
      cv::drawChessboardCorners( depth[i], patternSize, ( cv::Mat )( imagePoints[1][i] ), true );
      cv::imshow( "rgb", rgb[i] );
      cv::imshow("depth", depth[i]);
      cv::waitKey( 100 );
    } else {
      std::cout << " ... at least 1 corner not found." << std::endl;
      rgb.erase(rgb.begin() + i);
      depth.erase(depth.begin() + i);
      imagePoints[0].erase(imagePoints[0].begin() + i);
      imagePoints[1].erase(imagePoints[1].begin() + i);
      //      fileNum--;
      i--;
      cv::waitKey( 100 );
    }
  }
    return rgb.size();
}

void setWorldPoints(cv::vector<cv::vector<cv::Point3f> > &worldPoints, 
		    const cv::Size patternSize,
		    const int &fileNum){
  worldPoints.clear();
  worldPoints.resize(fileNum);
  for(int i = 0; i < fileNum; i++ )
    {
      for(int j = 0; j < patternSize.height; j++ )
	for(int k = 0; k < patternSize.width; k++ )
	  worldPoints[i].push_back(cv::Point3f(k*g_squareSize, j*g_squareSize, 0));
    }
}
