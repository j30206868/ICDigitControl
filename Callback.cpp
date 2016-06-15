

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
	myreader.init(false);
	myreader.start(mycallback);

	std::cout << "Press any key to continue!" << std::endl;
	std::cin.get();
	return 0;
}

