#include "cwz_dual_icdigit.h"

class MyVCDCallback : public DUAL_VCD_CALLBACK {
public:
	MyVCDCallback();
	void imgProc (cv::Mat left, cv::Mat right);
private:
	bool isKeyPressed;
	int frameCount;
};

using namespace cv;

inline int get1DCoord(int x, int y, int w){
	return y*w + x;
}

inline void fillCircle(float *img, int cx, int cy, int width, int r){
	int r2 = r * r;
	for(int y=r ; y>=-r ; y--){
		int x = sqrt(r2 - y*y);
		memset(&img[ get1DCoord(cx-x, cy-y, width) ], 0, sizeof(*img) * (x+x));
	}
}

inline void fillSector(float *img, int cx, int cy, int width, int height, int r){
	int ex = width - 1;
	int ey = height - 1;

	int r2 = r * r;
	for(int y=0 ; y<r ; y++){
		int x = sqrt(r2 - y*y);
		memset(&img[ get1DCoord(ex-x, y   , width) ], 0, sizeof(*img) * x);//第一象限
		memset(&img[ get1DCoord(0   , y   , width) ], 0, sizeof(*img) * x);//第二象限
		memset(&img[ get1DCoord(0   , ey-y, width) ], 0, sizeof(*img) * x);//第三象限
		memset(&img[ get1DCoord(ex-x, ey-y, width) ], 0, sizeof(*img) * x);//第四象限
	}
}

void highPassFilter(cv::Mat img, int r){
	int cx = img.cols/2;
    int cy = img.rows/2;

	//fillCircle((float *)img.data, cx, cy, img.cols, r);
	fillSector((float *)img.data, cx, cy, img.cols, img.rows, r);
}

int main(int argc, char* argv[])
{
	Mat I = cv::imread("raw/s_right01.jpg", CV_LOAD_IMAGE_GRAYSCALE);

	namedWindow("Input Image", CV_WINDOW_NORMAL);
    imshow("Input Image"       , I);

	if( I.empty())
        return -1;

    Mat padded;                            //expand input image to optimal size
    int m = getOptimalDFTSize( I.rows );
    int n = getOptimalDFTSize( I.cols ); // on the border add zero values
    copyMakeBorder(I, padded, 0, m - I.rows, 0, n - I.cols, BORDER_CONSTANT, Scalar::all(0));

    Mat planes[] = {Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F)};
    Mat complexI;
    merge(planes, 2, complexI);         // Add to the expanded another plane with zeros

    dft(complexI, complexI);            // this way the result may fit in the source matrix

    // compute the magnitude and switch to logarithmic scale
    // => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
    split(complexI, planes);                   // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))

	highPassFilter(planes[0], 1);
	highPassFilter(planes[1], 1);

	merge(planes, 2, complexI);

	Mat invDFT, invDFTcvt;
    idft(complexI, invDFT, DFT_SCALE | DFT_REAL_OUTPUT ); // Applying IDFT
    invDFT.convertTo(invDFTcvt, CV_8U); 
	namedWindow("Output", CV_WINDOW_NORMAL);
    imshow("Output", invDFTcvt);
	imwrite("test.bmp", invDFTcvt);
	/*
    magnitude(planes[0], planes[1], planes[0]);// planes[0] = magnitude
    Mat magI = planes[0];


    magI += Scalar::all(1);                    // switch to logarithmic scale
    log(magI, magI);
	
    // crop the spectrum, if it has an odd number of rows or columns
    magI = magI(Rect(0, 0, magI.cols & -2, magI.rows & -2));

    // rearrange the quadrants of Fourier image  so that the origin is at the image center
    int cx = magI.cols/2;
    int cy = magI.rows/2;

    Mat q0(magI, Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
    Mat q1(magI, Rect(cx, 0, cx, cy));  // Top-Right
    Mat q2(magI, Rect(0, cy, cx, cy));  // Bottom-Left
    Mat q3(magI, Rect(cx, cy, cx, cy)); // Bottom-Right

    Mat tmp;                           // swap quadrants (Top-Left with Bottom-Right)
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);

    q1.copyTo(tmp);                    // swap quadrant (Top-Right with Bottom-Left)
    q2.copyTo(q1);
    tmp.copyTo(q2);

    normalize(magI, magI, 0, 1, CV_MINMAX); // Transform the matrix with float values into a
                                            // viewable image form (float between values 0 and 1).
	
	//先歸零右半圓
	
	namedWindow("spectrum magnitude", CV_WINDOW_NORMAL);
    imshow("spectrum magnitude", magI);*/
    waitKey();

	/*DShowLib::InitLibrary();

	atexit( ExitLibrary );

	DUAL_VCD_READER myreader;
	if( !myreader.init() )
		return -1;

	DUAL_VCD_CALLBACK *mycallback = new MyVCDCallback();

	myreader.start(mycallback);

	delete mycallback;*/

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

