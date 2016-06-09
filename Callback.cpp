#include "cwz_dual_icdigit.h"

class MyVCDCallback : public DUAL_VCD_CALLBACK {
public:
	MyVCDCallback();
	void imgProc (cv::Mat left, cv::Mat right);
private:
	bool isKeyPressed;
	int frameCount;
};

int main(int argc, char* argv[])
{
	DShowLib::InitLibrary();

	atexit( ExitLibrary );

	DUAL_VCD_READER myreader;
	myreader.init();

	DUAL_VCD_CALLBACK *mycallback = new MyVCDCallback();

	myreader.start(mycallback);

	delete mycallback;

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

