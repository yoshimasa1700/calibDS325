#include "calibDS325.h"

#include <gflags/gflags.h>


using namespace std;
using namespace google;

#define MAX_DEPTH 1000
#define MIN_DEPTH 0

// definition for gflags
DEFINE_string(color, "./calibData/color_", "default file name");
DEFINE_string(depth, "./calibData/depth_", "defalut depth file name");
DEFINE_string(type, ".png", "default file type");
DEFINE_int32(num, 1, "default file num");

int main(int argc, char *argv[]){
  ParseCommandLineFlags(&argc, &argv, true);

  cv::vector<cv::Mat> rgb(0), depth(0); // checker pattern
  int fileNum = FLAGS_num;
  const cv::Size patternSize(9, 6);
  cv::vector<cv::vector<cv::Point3f> > worldPoints(fileNum);
  cv::vector<cv::vector<cv::vector<cv::Point2f> > > imagePoints(2);	
  cv::TermCriteria criteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.001 );
  
  for(int i = 0; i < 2; ++i)
    imagePoints[i].resize(fileNum);

  // カメラパラメータ行列
  cv::vector<cv::Mat> cameraMatrix(2);		// 内部パラメータ行列
  cv::vector<cv::Mat> distCoeffs(2);		// レンズ歪み行列
  cv::vector<cv::Mat> rotationVectors(2);	// 撮影画像ごとに得られる回転ベクトル
  cv::vector<cv::Mat> translationVectors(2);	// 撮影画像ごとに得られる平行移動ベクトル

  // create windows
  cv::namedWindow("rgb");
  cv::namedWindow("depth");

  // load chess board images
  for(int i = 0; i < fileNum; ++i){
    stringstream rgbfilename, depthfilename;
    rgbfilename << FLAGS_color << i << FLAGS_type;
    depthfilename << FLAGS_depth << i << FLAGS_type;
    
    cout << "loading : " << rgbfilename.str() << " and " << depthfilename.str() << endl;

    // load RGB image
    cv::Mat tempRGB = cv::imread(rgbfilename.str(), 0);
    //std::cout << tempRGB.channels() << std::endl;
    //cv::cvtColor(tempRGB, tempRGB, CV_RGB2GRAY,3);
    
    rgb.push_back(tempRGB);


    // load depth image
    cv::Mat tempDepth = cv::imread(depthfilename.str(), CV_LOAD_IMAGE_ANYDEPTH);
    tempDepth.convertTo(tempDepth, CV_8U, 255.0/1000.0);
    //cv::cvtColor(tempDepth, tempDepth, CV_GRAY2RGB, 3);
    cv::Mat maxDist = cv::Mat::ones(tempDepth.rows, tempDepth.cols, CV_8U) * MAX_DEPTH;
    cv::Mat minDist = cv::Mat::ones(tempDepth.rows, tempDepth.cols, CV_8U) * MIN_DEPTH;
    cv::min(tempDepth, maxDist, tempDepth);
    tempDepth -= minDist;
    cv::resize(tempDepth, tempDepth, cv::Size(), 2.0,2.0);

    depth.push_back(tempDepth);

    std::cout << "loaded" << std::endl;

    cv::imshow("rgb",rgb[i]);
    cv::imshow("depth",depth[i]);
    cv::waitKey(100);
  }

  cout << "total image num is " << fileNum << endl;



  // find chessboard pattern from rgb image
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
      imagePoints.erase(imagePoints.begin() + i);
      cv::waitKey( 100 );
    }
  }
  
  // reset file num
  fileNum = rgb.size();

  for(int i = 0; i < fileNum; i++ )
    {
        for(int j = 0; j < patternSize.height; j++ )
            for(int k = 0; k < patternSize.width; k++ )
	      worldPoints[i].push_back(cv::Point3f(k*0.05, j*0.05, 0));
    }
  
  // これまでの値を使ってキャリブレーション
  // cv::calibrateCamera( worldPoints, imagePoints, rgb[0].size(), cameraMatrix, distCoeffs, 
  // 		       rotationVectors, translationVectors );
  // std::cout << "Camera parameters have been estimated" << std::endl << std::endl;

  // cv::calibrateCamera( worldPoints, imagePoints[1], depth[0].size(), depthCameraMatrix, depthDistCoeffs, 
  // 		       depthRotationVectors, depthTranslationVectors );
  // std::cout << "Depth camera parameters have been estimated" << std::endl << std::endl;


  cout << "Running stereo calibration ...\n";

  //  cv::Mat cameraMatrix[2], distCoeffs[2];
  cameraMatrix[0] = cv::Mat::eye(3, 3, CV_64F);
  cameraMatrix[1] = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat R, T, E, F;

  double rms = stereoCalibrate(worldPoints, imagePoints[0], imagePoints[1],
			       cameraMatrix[0], distCoeffs[0],
			       cameraMatrix[1], distCoeffs[1],
			       rgb[0].size(), R, T, E, F,
			       cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
			       CV_CALIB_FIX_ASPECT_RATIO +
			       CV_CALIB_ZERO_TANGENT_DIST +
			       CV_CALIB_SAME_FOCAL_LENGTH +
			       CV_CALIB_RATIONAL_MODEL +
			       CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);
  cout << "done with RMS error=" << rms << endl;

// CALIBRATION QUALITY CHECK
// because the output fundamental matrix implicitly
// includes all the output information,
// we can check the quality of calibration using the
// epipolar geometry constraint: m2^t*F*m1=0
    double err = 0;
    int npoints = 0;
    cv::vector<cv::Vec3f> lines[2];
    for(int i = 0; i < fileNum; i++ )
    {
        int npt = (int)imagePoints[0][i].size();
	cv::Mat imgpt[2];
        //for(int k = 0; k < 2; k++ )
        {
	  imgpt[0] = cv::Mat(imagePoints[0][i]);
	  cv::undistortPoints(imgpt[0], imgpt[0], cameraMatrix[0], distCoeffs[0], cv::Mat(), cameraMatrix[0]);
	  cv::computeCorrespondEpilines(imgpt[0], 1, F, lines[0]);
        }
	{
	  imgpt[1] = cv::Mat(imagePoints[1][i]);
	  cv::undistortPoints(imgpt[1], imgpt[1], cameraMatrix[1], distCoeffs[1], cv::Mat(), cameraMatrix[1]);
	  cv::computeCorrespondEpilines(imgpt[1], 2, F, lines[1]);
        }

        for(int j = 0; j < npt; j++ )
        {
	  double errij = std::fabs(imagePoints[0][i][j].x*lines[1][j][0] +
                                imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
                           fabs(imagePoints[1][i][j].x*lines[0][j][0] +
                                imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    cout << "average reprojection err = " <<  err/npoints << endl;

  cv::waitKey(0);

  // save intrinsic parameters
  cv::FileStorage fs("intrinsics.yml", CV_STORAGE_WRITE);
    if( fs.isOpened() )
    {
        fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
	  "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
        fs.release();
    }
    else
        cout << "Error: can not save the intrinsic parameters\n";

    cv::Mat R1, R2, P1, P2, Q;
    cv::Rect validRoi[2];

    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  rgb[0].size(), R, T, R1, R2, P1, P2, Q,
                  cv::CALIB_ZERO_DISPARITY, 1, rgb[0].size(), &validRoi[0], &validRoi[1]);

    fs.open("extrinsics.yml", CV_STORAGE_WRITE);
    if( fs.isOpened() )
    {
        fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
        fs.release();
    }
    else
        cout << "Error: can not save the intrinsic parameters\n";

    //カメラ内部パラメータの計算
        double apertureWidth = 0, apertureHeight = 0 ;  // センサの物理的な幅・高さ
        double fovx = 0, fovy = 0;                                              // 出力される水平（垂直）軸にそった画角（度単位）
        double focalLength = 0;                                                 // mm単位で表されたレンズの焦点距離
        cv::Point2d principalPoint(0,0);                             // ピクセル単位で表されたレンズの焦点距離
        double aspectRatio = 0;        


    cv::calibrationMatrixValues( cameraMatrix[0], rgb[0].size(), apertureWidth, apertureHeight,
                fovx, fovy, focalLength, principalPoint, aspectRatio );

        cout << "Calc Camera Param" << endl;

        // XMLファイルへ結果を出力する
        cout << "Start Output Xml File" << endl;

        cv::FileStorage wfs("./cameraparam.xml", cv::FileStorage::WRITE);
        //wfs << XMLTAG_CAMERAMAT << cameraMatrix;
        //wfs << XMLTAG_DISTCOEFFS << distCoeffs;

        wfs << "aperture_Width" << apertureWidth;
        wfs << "aperture_Height" << apertureHeight;
        wfs << "fov_x" << fovx;
        wfs << "fov_y" << fovy;
        wfs << "focal_Length" << focalLength;
        wfs << "principal_Point" << principalPoint;
        wfs << "aspect_Ratio" << aspectRatio;
        wfs.release();

        cout << "Finish Output Xml File" << endl;


    // OpenCV can handle left-right
    // or up-down camera arrangements
    //bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

// COMPUTE AND DISPLAY RECTIFICATION
    // if( !showRectified )
    //     return;

    cv::Mat rmap[2][2];
// IF BY CALIBRATED (BOUGUET'S METHOD)
    // if( useCalibrated )
    // {
    //     // we already computed everything
    // }
// OR ELSE HARTLEY'S METHOD
//    else
 // use intrinsic parameters of each camera, but
 // compute the rectification transformation directly
 // from the fundamental matrix
    // {
    //   cv::vector<cv::Point2f> allimgpt[2];
    //   //for(int k = 0; k < 2; k++ )
    //     {
    //         for(int i = 0; i < fileNum; i++ )
    // 	      std::copy(imagePoints[i].begin(), imagePoints[i].end(), std::back_inserter(allimgpt[0]));
    //     }
    //     {
    //         for(int i = 0; i < fileNum; i++ )
    // 	      std::copy(imagePoints[1][i].begin(), imagePoints[1][i].end(), std::back_inserter(allimgpt[1]));
    //     }

    //     F = cv::findFundamentalMat(cv::Mat(allimgpt[0]), cv::Mat(allimgpt[1]), cv::FM_8POINT, 0, 0);
    // 	cv::Mat H1, H2;
    // 	cv::stereoRectifyUncalibrated(cv::Mat(allimgpt[0]), cv::Mat(allimgpt[1]), F, rgb[0].size(), H1, H2, 3);

    //     R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
    //     R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
    //     P1 = cameraMatrix[0];
    //     P2 = cameraMatrix[1];
    // }

    //Precompute maps for cv::remap()
    cv::initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, rgb[0].size(), CV_16SC2, rmap[0][0], rmap[0][1]);
    cv::initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, rgb[0].size(), CV_16SC2, rmap[1][0], rmap[1][1]);

    cv::Mat canvas;
    double sf;
    int w, h;
    // if( !isVerticalStereo )
    // {
    sf = 600./MAX(rgb[0].size().width, rgb[0].size().height);
    w = cvRound(rgb[0].size().width*sf);
    h = cvRound(rgb[0].size().height*sf);
    canvas.create(h, w*2, CV_8UC3);
    // }
    // else
    // {
    //     sf = 300./MAX(rgb[0].size().width, rgb[0].size().height);
    //     w = cvRound(rgb[0].size().width*sf);
    //     h = cvRound(rgb[0].size().height*sf);
    //     canvas.create(h*2, w, CV_8UC3);
    // }

  depth.clear();
  rgb.clear();

  // load chess board images
  std::cout << "load image for quality check." << std::endl;
  for(int i = 0; i < fileNum; ++i){
    stringstream rgbfilename, depthfilename;
    rgbfilename << FLAGS_color << i << FLAGS_type;
    depthfilename << FLAGS_depth << i << FLAGS_type;
    
    cout << "loading : " << rgbfilename.str() << " and " << depthfilename.str() << endl;

    // load RGB image
    cv::Mat tempRGB = cv::imread(rgbfilename.str(), 0);
    //std::cout << tempRGB.channels() << std::endl;
    //cv::cvtColor(tempRGB, tempRGB, CV_RGB2GRAY,3);
    
    rgb.push_back(tempRGB);


    // load depth image
    cv::Mat tempDepth = cv::imread(depthfilename.str(), CV_LOAD_IMAGE_ANYDEPTH);
    tempDepth.convertTo(tempDepth, CV_8U, 255.0/1000.0);
    //cv::cvtColor(tempDepth, tempDepth, CV_GRAY2RGB, 3);
    cv::Mat maxDist = cv::Mat::ones(tempDepth.rows, tempDepth.cols, CV_8U) * MAX_DEPTH;
    cv::Mat minDist = cv::Mat::ones(tempDepth.rows, tempDepth.cols, CV_8U) * MIN_DEPTH;
    cv::min(tempDepth, maxDist, tempDepth);
    tempDepth -= minDist;
    cv::resize(tempDepth, tempDepth, cv::Size(), 2.0,2.0);

    depth.push_back(tempDepth);

    std::cout << "loaded" << std::endl;

    // cv::imshow("rgb",rgb[i]);
    // cv::imshow("depth",depth[i]);
    //cv::waitKey(0);
  }

  cv::namedWindow("rectified");

    for(int i = 0; i < fileNum; i++ )
      {
        for(int k = 0; k < 2; k++ )
	  {
	    if(k == 0){
	      cv::Mat img = rgb[i].clone(), rimg, cimg;

	      cv::remap(img, rimg, rmap[k][0], rmap[k][1], CV_INTER_LINEAR);
	      cv::cvtColor(rimg, cimg, CV_GRAY2BGR);


	      cv::Mat canvasPart = canvas(cv::Rect(w * k, 0, w, h));
	      cv::resize(cimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
	      // if( useCalibrated )
	      	{
		  cv::Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
	      		    cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
		  cv::rectangle(canvasPart, vroi, cv::Scalar(0,0,255), 3, 8);
	      	}
	    }else{
	      cv::Mat img = depth[i].clone(), rimg, cimg;
	      cv::remap(img, rimg, rmap[k][0], rmap[k][1], CV_INTER_LINEAR);
	      cvtColor(rimg, cimg, CV_GRAY2BGR);
	      cv::Mat canvasPart = canvas(cv::Rect(w * k, 0, w, h));
	      cv::resize(cimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
	      	{
		  cv::Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
	      		    cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
		  cv::rectangle(canvasPart, vroi, cv::Scalar(0,0,255), 3, 8);
	      	}
	    }
	  }

        //if( !isVerticalStereo )
            for(int j = 0; j < canvas.rows; j += 16 )
	      cv::line(canvas, cv::Point(0, j), cv::Point(canvas.cols, j), cv::Scalar(0, 255, 0), 1, 8);
	    //else
            //for(int j = 0; j < canvas.cols; j += 16 )
	    //cv::line(canvas, cv::Point(j, 0), cv::Point(j, canvas.rows), cv::Scalar(0, 255, 0), 1, 8);
	    cv::imshow("rectified", canvas);
        char c = (char)cv::waitKey(0);
        if( c == 27 || c == 'q' || c == 'Q' )
            break;
    }

  // 歪み補正した画像を表示
  // std::cout << "Undistorted images" << std::endl;
  // cv::Mat	undistorted, depthUndistorted;
  // cv::namedWindow( "result" );
  // cv::namedWindow( "result_depth" );
  // for( int i = 0; i < fileNum; i++ ) {
  //   cv::undistort( rgb[i], undistorted, cameraMatrix, distCoeffs );
  //   cv::undistort( depth[i], depthUndistorted, depthCameraMatrix, depthDistCoeffs );

  //   cv::imshow( "result", undistorted );
  //   cv::imshow( "rgb", rgb[i] );

  //   cv::imshow( "result_depth", depthUndistorted );
  //   cv::imshow( "depth", depth[i] );

  //   cv::waitKey( 0 );
  // }

  cv::destroyAllWindows();

  return 0;

}
