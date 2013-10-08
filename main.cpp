#include <opencv2/opencv.hpp>
#include "calibDS325.h"

#include <iostream>
#include <string>
#include <sstream>

#include <gflags/gflags.h>

using namespace std;
using namespace google;

#define MAX_DEPTH 1000
#define MIN_DEPTH 0

// definition for gflags
DEFINE_string(rgbFileName, "rgb_", "default file name");
DEFINE_string(depthFileName, "depth_", "defalut depth file name");
DEFINE_string(fileType, ".png", "default file type");
DEFINE_int32(fileNum, 1, "default file num");

int main(int argc, char *argv[]){
  ParseCommandLineFlags(&argc, &argv, true);


  cv::vector<cv::Mat> rgb(0), depth(0); // checker pattern
  int fileNum = FLAGS_fileNum;
  const cv::Size patternSize( 6, 9 );
  cv::vector<cv::vector<cv::Point3f> > worldPoints(fileNum);
  cv::vector<cv::vector<cv::Point2f> > imagePoints(fileNum);	
  cv::TermCriteria criteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.001 );

  // カメラパラメータ行列
  cv::Mat cameraMatrix;		// 内部パラメータ行列
  cv::Mat distCoeffs;		// レンズ歪み行列
  cv::vector<cv::Mat> rotationVectors;	// 撮影画像ごとに得られる回転ベクトル
  cv::vector<cv::Mat> translationVectors;	// 撮影画像ごとに得られる平行移動ベクトル
  cv::vector<cv::vector<cv::Point3f> > depthWorldPoints(fileNum);
  cv::vector<cv::vector<cv::Point2f> > depthImagePoints(fileNum);	
  cv::TermCriteria depthCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.001 );

  // カメラパラメータ行列
  cv::Mat depthCameraMatrix;		// 内部パラメータ行列
  cv::Mat depthDistCoeffs;		// レンズ歪み行列
  cv::vector<cv::Mat> depthRotationVectors;	// 撮影画像ごとに得られる回転ベクトル
  cv::vector<cv::Mat> depthTranslationVectors;	// 撮影画像ごとに得られる平行移動ベクトル


  // load chess board images
  for(int i = 0; i < fileNum; ++i){
    stringstream rgbfilename, depthfilename;
    rgbfilename << FLAGS_rgbFileName << i << FLAGS_fileType;
    depthfilename << FLAGS_depthFileName << i << FLAGS_fileType;
    
    cout << "loading : " << rgbfilename.str() << " and " << depthfilename.str() << endl;

    rgb.push_back(cv::imread(rgbfilename.str(), 3));
    depth.push_back(cv::imread(depthfilename.str(), CV_LOAD_IMAGE_ANYDEPTH));
  }

  cout << "total image num is " << fileNum << endl;

  // create windows
  cv::namedWindow("rgb");
  cv::namedWindow("depth");

  // find chessboard pattern from rgb image
  for(int i = 0; i < fileNum; ++i){
    cout << i << endl;

    // convert image to gray scale(not nessesary)
    cv::Mat grayImg;
    cv::cvtColor(rgb[i], grayImg, CV_BGR2GRAY);
    cv::normalize(grayImg, grayImg, 0, 255, cv::NORM_MINMAX);
    cv::imshow("rgb", grayImg);

    if( cv::findChessboardCorners( rgb[i], patternSize, imagePoints[i] ) ) {
      std::cout << " ... All corners found." << std::endl;
      // 検出点を描画する
      cv::drawChessboardCorners( rgb[i], patternSize, ( cv::Mat )( imagePoints[i] ), true );
      //cv::imshow( "rgb", rgb[i] );
      //cv::waitKey( 500 );
    } else {
      std::cout << " ... at least 1 corner not found." << std::endl;
      //rgb.erase(rgb.begin() + i);
      //imagePoints.erase(imagePoints.begin() + i);
      cv::waitKey( 100 );
    }
  }

  // find chessboard pattern from depth file
  for(int i = 0; i < fileNum; ++i){  
    cv::Mat maxDist = cv::Mat::ones(depth[i].rows, depth[i].cols, CV_16U) * MAX_DEPTH;
    cv::Mat minDist = cv::Mat::ones(depth[i].rows, depth[i].cols, CV_16U) * MIN_DEPTH;
  
    cv::min(depth[i], maxDist, depth[i]);
    depth[i] -= minDist;

    depth[i].convertTo(depth[i], CV_8UC1, 255.0 / (MAX_DEPTH - MIN_DEPTH));
    //cv::threshold(depth[i], depth[i], 200, 255.0, cv::THRESH_BINARY);

    //cv::vector<cv::Point2f> depthCorners;
    //cv::goodFeaturesToTrack(depth[i], depthCorners, 80, 0.01, 5);

    // cv::vector<cv::Point2f>::iterator it_corner = depthCorners.begin();
    // for(; it_corner!=depthCorners.end(); ++it_corner) {
    //   cv::circle(depth[i], cv::Point(it_corner->x, it_corner->y), 1, cv::Scalar(0,200,255), -1);
    //   cv::circle(depth[i], cv::Point(it_corner->x, it_corner->y), 8, cv::Scalar(0,200,255));
    // }

    cv::imshow("depth", depth[i]);
    cv::waitKey(100);
    
    if( cv::findChessboardCorners( depth[i], patternSize, depthImagePoints[i] ) ) {
      std::cout << " ... All corners found." << std::endl;
      // 検出点を描画する
      cv::drawChessboardCorners( depth[i], patternSize, ( cv::Mat )( depthImagePoints[i] ), true );
      //cv::imshow( "depth", depth[i] );
      //cv::waitKey( 100 );
    } else {
      std::cout << " ... at least 1 corner not found." << std::endl;
      cv::waitKey( 100 );
    }
  }

  for( int i = 0; i < fileNum; i++ ) {
    for( int j = 0 ; j < patternSize.area(); j++ ) {
      worldPoints[i].push_back( cv::Point3f(	static_cast<float>( j % patternSize.width * 10 ), 
						static_cast<float>( j / patternSize.width * 10 ), 
						0.0 ) );
    }
  }

  // これまでの値を使ってキャリブレーション
  cv::calibrateCamera( worldPoints, imagePoints, rgb[0].size(), cameraMatrix, distCoeffs, 
		       rotationVectors, translationVectors );
  std::cout << "Camera parameters have been estimated" << std::endl << std::endl;

  cv::calibrateCamera( worldPoints, depthImagePoints, depth[0].size(), depthCameraMatrix, depthDistCoeffs, 
		       depthRotationVectors, depthTranslationVectors );
  std::cout << "Depth camera parameters have been estimated" << std::endl << std::endl;

  cv::waitKey(0);

  // 歪み補正した画像を表示
  std::cout << "Undistorted images" << std::endl;
  cv::Mat	undistorted, depthUndistorted;
  cv::namedWindow( "result" );
  cv::namedWindow( "result_depth" );
  for( int i = 0; i < fileNum; i++ ) {
    cv::undistort( rgb[i], undistorted, cameraMatrix, distCoeffs );
    cv::undistort( depth[i], depthUndistorted, depthCameraMatrix, depthDistCoeffs );

    cv::imshow( "result", undistorted );
    cv::imshow( "rgb", rgb[i] );

    cv::imshow( "result_depth", depthUndistorted );
    cv::imshow( "depth", depth[i] );

    cv::waitKey( 0 );
  }

  cv::destroyAllWindows();

  return 0;

}
