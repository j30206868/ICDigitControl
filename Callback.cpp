

#include "cwz_dual_icdigit.h"
#include "cwz_stereo.h"
#include "cwz_vcd_callback.h"


using namespace cv;

int main(int argc, char* argv[])
{

	/*Mat img = imread("raw/s_left01.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	Mat blurImg;
	cv::blur(img, blurImg, cv::Size(5, 5));

	highPass(blurImg, 60);

	histogramEqualize(blurImg);

	namedWindow("Result", CV_WINDOW_NORMAL);
	imshow("Result", blurImg);

	namedWindow("Output", CV_WINDOW_NORMAL);
	imshow("Output", img);
	cvWaitKey(0);*/

	MyVCDCallback *mycallback = new MyVCDCallback();
	//mycallback->calib_proc.stereoCalibAndRectify();

	/*SAM_STEREO_READER reader;
	reader.init();
	reader.start(mycallback);*/
	DUAL_VCD_READER myreader;
	myreader.init();
	myreader.start(mycallback);
	/*
	cv::Mat left = cv::imread("raw/left1.bmp");
	cv::Mat right = cv::imread("raw/right1.bmp");

	int multiplier = 2;
	cv::Mat left_b, right_b;
	cv::resize(left, left_b, cv::Size(left.cols * multiplier, left.rows * multiplier));
	cv::resize(right, right_b, cv::Size(left.cols * multiplier, left.rows * multiplier));

	mycallback->flipLeftRight(left_b, right_b);
	mycallback->imgProc(left_b, right_b);
	*/
	std::cout << "Press any key to continue!" << std::endl;
	std::cin.get();
	return 0;
}

