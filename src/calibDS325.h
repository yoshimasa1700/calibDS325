#ifndef _CALIBDS325_
#define _CALIBDS325_

#include <opencv2/opencv.hpp>

#include <gflags/gflags.h>

#include <iostream>
#include <string>
#include <sstream>

#define MAX_DEPTH 1000
#define MIN_DEPTH 0

void loadImages(cv::vector<cv::Mat> &rgb, 
		cv::vector<cv::Mat> &depth, 
		const int &fileNum);
int findChessboard(cv::vector<cv::Mat> &rgb, 
		   cv::vector<cv::Mat> &depth, 
		   cv::vector<cv::vector<cv::vector<cv::Point2f> > > &imagePoints,
		   const cv::Size patternSize,
		   const int &fileNum);
void setWorldPoints(cv::vector<cv::vector<cv::Point3f> > &worldPoints, 
		    const cv::Size patternSize,
		    const int &fileNum);

#endif
