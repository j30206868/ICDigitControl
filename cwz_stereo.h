#ifndef CWZ_STEREO_H
#define CWZ_STEREO_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <fstream>
#include <sstream>

#include "cwz_dual_icdigit.h"

class SAM_STEREO_READER{
public:
	int w, h;
	bool recording;
	void init();
	void stop();
	int start(DUAL_VCD_CALLBACK *callback);
};

class CALIB_PROC{
public:
	bool valid_calib_param;
	cv::Size imageSize;
	cv::Mat cameraMatrix[2];
	cv::Mat distCoeffs[2];
	cv::Mat R;
	cv::Mat T;
	cv::Mat R1;
	cv::Mat R2; 
	cv::Mat P1; 
	cv::Mat P2;
	cv::Mat Q;
	cv::Mat rmap[2][2];
	cv::Rect validRoi[2];
	cv::Rect commonRoi;
	//
	bool valid_proj_calib_param;
	std::string proj_calib_file_path;
	cv::Mat cam_K;
	cv::Mat cam_kc;
	cv::Mat proj_K;
	cv::Mat proj_kc;
	cv::Mat proj_R;
	cv::Mat proj_T;
	//
	cv::Mat CamProjHomography;
	//
	std::string calib_img_path;
	std::string img_list_xml_path;
	std::string img_list_xml_fname;
	std::string img_ext;
	int calib_frame_number;
	//
	bool img_list_fstream_opened;
	std::ofstream fs;
	
	void init();
	void readCameraIntrAndExtr();
	void readProjIntrAndExtr();

	void openImgListFileStream();
	void writeToImgListFileStream(std::string fname);
	void saveNewCalibImg(cv::Mat left, cv::Mat right);
	void closeImgListFileStream();

	void stereoCalibAndRectify();

	bool checkIfFileExist (const std::string& name);

	void remapAndDrawRoi(cv::Mat &left, cv::Mat &right);
	void remapAndCutRoi(cv::Mat &left, cv::Mat &right);
	void remapping(cv::Mat &left, cv::Mat &right);
	void cutRoi(cv::Mat &left, cv::Mat &right);
};


#endif