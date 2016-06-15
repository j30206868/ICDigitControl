#ifndef CWZ_VCD_CALLBACK_H
#define CWZ_VCD_CALLBACK_H

//all used by main function
#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/cl.h>
#endif

#include "PxlMatch/cwz_cl_data_type.h"
#include "PxlMatch/cwz_cl_cpp_functions.h"
#include "TreeFilter/cwz_mst.h"
#include "TreeFilter/cwz_disparity_generation.h"
#include "GuidedFilter/cwz_integral_img.h"
#include "EdgeMatch/cwz_edge_detect.h"

#include "cwz_dual_icdigit.h"
#include "cwz_stereo.h"


class MyVCDCallback : public DUAL_VCD_CALLBACK {
public:
	MyVCDCallback();
	~MyVCDCallback();
	void imgProc (cv::Mat left, cv::Mat right);
	void highPass(cv::Mat input, int channel, int r);
	void histogramEqualize(cv::Mat input, int channel);

	void toggleCalibProc(cv::Mat left, cv::Mat right);
	void showLeftRight(cv::Mat left, cv::Mat right);
	void showLeftRightMerged(cv::Mat left, cv::Mat right);

	bool runningLogger;
	char inputKey;
	std::thread keyLogger;
	CALIB_PROC calib_proc;
private:
	int frameCount;
	//calibration
	bool isCalibrationOn; 
	//disparity
	int subsample_divider;
	cl_int err;
	cl_context context;
	cl_device_id device;
	cl_program program;
	match_info *info;
	bool isDispInit;
	dmap_gen dmap_generator;
	dmap_refine dmap_ref;
	//
};


#endif