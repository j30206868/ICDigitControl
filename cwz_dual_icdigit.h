#ifndef CWZ_DUAL_ICDIGIT_H

#include "ICDigitConfig.h"

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

class DUAL_VCD_CALLBACK;
class DUAL_VCD_READER;

class DUAL_VCD_CALLBACK{
public:
	virtual void imgProc (cv::Mat left, cv::Mat right) = 0;
};

class DUAL_VCD_READER{
public:
	Grabber leftGrabber;
	Grabber rightGrabber;

	bool recording;

	int init(bool openWithXML = true);
	void start(DUAL_VCD_CALLBACK *callback);
	void stop();
};

void snapImg(smart_ptr<FrameHandlerSink> pSink, bool *running);

#endif