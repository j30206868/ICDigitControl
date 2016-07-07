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

#define MY_VCD_CALLBACK_KEY_LOG_AMT 36
class MyVCDCallback : public DUAL_VCD_CALLBACK {
public:
	MyVCDCallback();
	~MyVCDCallback();
	void imgProc (cv::Mat left, cv::Mat right);
	void highPass(cv::Mat input, int channel, int r);
	void histogramEqualize(cv::Mat input, int channel);

	void toggleCalibProc(cv::Mat left, cv::Mat right, char inputKeyAtBegin);
	void showLeftRight(std::string pre_title, cv::Mat left, cv::Mat right);
	void showLeftRightMerged(cv::Mat left, cv::Mat right);

	void flipLeftRight(cv::Mat &left, cv::Mat &right);

	void applyHomography(cv::Mat left, int subsample_divider);
	bool convertLeftToProj(cv::Mat left, cv::Mat right, CWZDISPTYPE *dmap, cv::Mat proj, int subsample_divider);

	bool runningLogger;
	bool isKeyPressed[MY_VCD_CALLBACK_KEY_LOG_AMT];
	bool isKeyClicked[MY_VCD_CALLBACK_KEY_LOG_AMT];
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
	int disp_fcount;
	//
	bool is_left_2d_points_init;
	cv::Mat left_2d_points;
	cv::Mat left_3d_points;
	double *left_2d_points_data;
	double *left_3d_points_data;
	//
};

inline float getFocalLengthInMM(float fx, float pixel_size_x_in_mm){
	return fx * pixel_size_x_in_mm;
}

inline float getZInMM(CWZDISPTYPE disp, float focal_length_in_mm, float baseline_in_mm, float pixel_size_x_in_mm, int dispScaleRatio = 1){
	return focal_length_in_mm * baseline_in_mm / ((disp * dispScaleRatio) * pixel_size_x_in_mm);
}

inline float getYInMM(int y_index, int oy, float z_div_focal_length_in_mm, float pixel_size_y_in_mm){
	return ((y_index - oy) * pixel_size_y_in_mm) * z_div_focal_length_in_mm;
}

inline float getXInMM(int x_index, int ox, float z_div_focal_length_in_mm, float focal_length_in_mm, float pixel_size_x_in_mm){
	return ((x_index - ox) * pixel_size_x_in_mm) * z_div_focal_length_in_mm;
}

inline float getBaselineInMM(float baseline, float chessboard_unit_x_in_mm){
	return baseline * chessboard_unit_x_in_mm;
}

#endif