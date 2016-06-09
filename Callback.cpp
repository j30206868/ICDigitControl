#include "cwz_dual_icdigit.h"

class MyVCDCallback : public DUAL_VCD_CALLBACK {
public:
	MyVCDCallback();
	void imgProc (cv::Mat left, cv::Mat right);
private:
	bool isKeyPressed;
	int frameCount;
};

void highPass(cv::Mat input, int r){
	cv::Mat blur;
	cv::blur(input, blur, cv::Size(r, r));
	//GaussianBlur(img, blurImg, Size(30, 30), 0, 0);

	for(int i=0 ; i<input.rows * input.cols ; i++){
		int result = input.data[i] - blur.data[i] + 128;
		if(result < 0)
			result = 0;
		else if(result > 255)
			result = 255;
		input.data[i] = result;
	}
}

void histogramEqualize(cv::Mat input){
	int totalNode = input.rows * input.cols;

	int histogram[256];
	memset(histogram, 0, sizeof(int) * 256);
	for(int i=0 ; i<totalNode; i++){
		histogram[input.data[i]]++;
	}

	uchar lookup[256];
	for(int i=0 ; i<256 ; i++){
		if(i > 0)
			histogram[i] += histogram[i-1];
		lookup[i] = histogram[i] / (double)totalNode * 255;
	}

	for(int i=0 ; i<totalNode; i++){
		input.data[i] = lookup[ input.data[i] ];
	}
}

using namespace cv;

int main(int argc, char* argv[])
{
	Mat img = imread("raw/s_left01.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	Mat blurImg;
	cv::blur(img, blurImg, cv::Size(5, 5));

	highPass(blurImg, 60);

	histogramEqualize(blurImg);

	namedWindow("Result", CV_WINDOW_NORMAL);
	imshow("Result", blurImg);

	namedWindow("Output", CV_WINDOW_NORMAL);
	imshow("Output", img);
	cvWaitKey(0);

	std::cout << "Press any key to continue!" << std::endl;
	std::cin.get();
	return 0;
}

MyVCDCallback::MyVCDCallback(){
	isKeyPressed = false;
	frameCount = 1;
}
void MyVCDCallback::imgProc(cv::Mat left, cv::Mat right){
	//left
	cv::namedWindow("left", CV_WINDOW_FREERATIO);
	cv::imshow("left", left);
	cvWaitKey(1);
	//right
	cv::namedWindow("right", CV_WINDOW_FREERATIO);
	cv::imshow("right", right);
	cvWaitKey(1);

	SHORT upKeyState = GetAsyncKeyState(VK_UP);
	if( ( 1 << 16 ) & upKeyState){
		if(!isKeyPressed){
			saveLeftRightImg(left, right, frameCount);
			std::cout << "frameCount: "  << frameCount  << std::endl;
			frameCount++;
		}
		isKeyPressed = true;
	}else{
		isKeyPressed = false;
	}
}

