#ifndef H_ICDIGITCONFIG_H

#define _WIN32_WINNT 0x0500

#include <iostream>
#include <conio.h>

#include <tisudshl.h>

#include "CmdHelper.h"
#include "Listener.h"

#include <thread>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace _DSHOWLIB_NAMESPACE;

// Specify the number of buffers to be used.
#define NUM_BUFFERS 1

const int AcqImgWidth  = 1280;
const int AcqImgHeight = 960;

//use img buffer to contain received images to avoid memory conflict between read and right on the same buffer
const int imgBufferSize = 5;

void snapImg(smart_ptr<FrameHandlerSink> pSink, bool &running){
	while(running){
		pSink ->snapImagesAsync( NUM_BUFFERS );	// Grab NUM_BUFFERS images.
	}
}

inline void saveLeftRightImg(cv::Mat left, cv::Mat right, int frameCount){
	char filename[MAX_PATH];
	std::stringstream sstm;
	sprintf( filename, "%02i.bmp", frameCount );
	sstm << "raw/left" << filename; 
	cv::imwrite(sstm.str(), left);
	sstm.str("");
	sstm << "raw/right" << filename; 
	cv::imwrite(sstm.str(), right);
}

#endif //H_ICDIGITCONFIG_H