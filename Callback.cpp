///////////////////////////////////////////////////////////////////////////////
//
// This example demonstrates how to use a GrabberListener derived
// callback handler object to process events
//
// A class CListener is derived from GrabberListener. It is used to handle callbacks.
// The method CListener::frameReady() simulates a time expensive processing. Therefore,
// the method CListener::frameReady() is not called for every captured buffer. 
// The member CListener::m_pBufferWritten is used to record, which buffers were processed
// by CListener::frameReady().
// After snapImages() returns, the main function will save the buffers that were not
// processed. This sample shows that all buffers have been copied correctly to the
// MembufferCollection, although CListener::frameReady() was not called for every buffer.
//

#define _WIN32_WINNT 0x0500

#include <iostream>
#include <conio.h>

#include <tisudshl.h>

#include "CmdHelper.h"
#include "Listener.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <thread>
#include <mutex>

using namespace _DSHOWLIB_NAMESPACE;

// Specify the number of buffers to be used.
#define NUM_BUFFERS 1

const int AcqImgWidth  = 1280;
const int AcqImgHeight = 960;

//use img buffer to contain received images to avoid memory conflict between read and right on the same buffer
const int imgBufferSize = 10;

void snapImg(smart_ptr<FrameHandlerSink> pSink, bool &running){
	while(running){
		pSink ->snapImages( NUM_BUFFERS );	// Grab NUM_BUFFERS images.
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

int main(int argc, char* argv[])
{

	DShowLib::InitLibrary();

	atexit( ExitLibrary );

	Grabber leftGrabber;
	Grabber rightGrabber;

	// Create the GrabberListener object.
	// CListener is derived from GrabberListener.
	CListener *pLeftListener = new CListener();
	pLeftListener->imgBufHeadIdx = 0;
	pLeftListener->imgBufferSize = imgBufferSize;
	CListener *pRightListener = new CListener();
	pRightListener->imgBufHeadIdx = 0;
	pRightListener->imgBufferSize = imgBufferSize;
								
	//讀影像資料
	if( !setupDeviceFromFile( leftGrabber ) )
	{
		return -1;
	}
	if( !setupDeviceFromFile( rightGrabber ) )
	{
		return -1;
	}
	
	// Assign the number of buffers to the cListener object.
	pLeftListener->setBufferSize( NUM_BUFFERS );
	pRightListener->setBufferSize( NUM_BUFFERS );

	// Enable the overlay bitmap to display the frame counter in the live video.
	leftGrabber.getOverlay()->setEnable( true );
	rightGrabber.getOverlay()->setEnable( true );

	// Register the pListener object for the frame ready and 
	// the overlay callback event.
	leftGrabber.addListener ( pLeftListener , GrabberListener::eFRAMEREADY|
											  GrabberListener::eOVERLAYCALLBACK );
	rightGrabber.addListener( pRightListener, GrabberListener::eFRAMEREADY|
											  GrabberListener::eOVERLAYCALLBACK );


	// Create a FrameTypeInfoArray data structure describing the allowed color formats.
	FrameTypeInfo leftTypeInfo ( eRGB8, AcqImgWidth, AcqImgHeight );
	FrameTypeInfo rightTypeInfo( eRGB8, AcqImgWidth, AcqImgHeight );

	// Create the frame handler sink
	smart_ptr<FrameHandlerSink> pLeftSink  = FrameHandlerSink::create( leftTypeInfo , NUM_BUFFERS );
	smart_ptr<FrameHandlerSink> pRightSink = FrameHandlerSink::create( rightTypeInfo, NUM_BUFFERS );

	// enable snap mode (formerly tFrameGrabberMode::eSNAP).
	pLeftSink ->setSnapMode( true );
	pRightSink->setSnapMode( true );

	// Apply the sink to the grabber.
	leftGrabber .setSinkType( pLeftSink );
	rightGrabber.setSinkType( pRightSink );

	leftGrabber .startLive(false);				// Start the grabber.
	rightGrabber.startLive(false);				// Start the grabber.

	cv::Mat left;
	cv::Mat right;
	bool isKeyPressed = false;
	bool recording = true;
	int frameCount = 1;

	//
	smart_ptr<MemBuffer> pLeftBuffer  = pLeftSink ->getMemBufferCollection()->getBuffer( 0 );
	smart_ptr<MemBuffer> pRightBuffer = pRightSink->getMemBufferCollection()->getBuffer( 0 );
	left  = cv::Mat(pLeftBuffer->getSize().cy, pLeftBuffer->getSize().cx, CV_8UC1);
	right = cv::Mat(pRightBuffer->getSize().cy, pRightBuffer->getSize().cx, CV_8UC1);

	//init img buffer
	pLeftListener->imgBuffer    = new BYTE*[imgBufferSize];
	pRightListener->imgBuffer   = new BYTE*[imgBufferSize];
	for(int i=0 ; i < pLeftListener->imgBufferSize ; i++){
		pLeftListener->imgBuffer[i]  = new BYTE[pLeftBuffer->getBufferSize()];
		pRightListener->imgBuffer [i] = new BYTE[pRightBuffer->getBufferSize()];
	}
	//

	//show information
	std::cout << "==== Image Information ====" << std::endl;
	std::cout << "Image Dimension     : (" << pLeftBuffer->getSize().cx << ", " << pLeftBuffer->getSize().cy << ")" << std::endl;
	std::cout << "Bytes for each frame: " << pLeftBuffer->getBufferSize() << std::endl;
	std::cout << "Bits per pixel      : " << pLeftBuffer->getBitsPerPixel() << std::endl;
	

	//loop to get newest image
	std::thread leftThread  (snapImg, pLeftSink , recording);
	std::thread rightThread (snapImg, pRightSink, recording);

	while(recording){
		//index - 1 才是最新的一張
		int L_Idx = pLeftListener->imgBufHeadIdx - 1;
		int R_Idx = pRightListener->imgBufHeadIdx - 1;
		if(L_Idx < 0)
			L_Idx = imgBufferSize - 1;
		if(R_Idx < 0)
			R_Idx = imgBufferSize - 1;
		
		memcpy(left.data, pLeftListener->imgBuffer[L_Idx], pLeftBuffer->getBufferSize());
		memcpy(right.data, pRightListener->imgBuffer[R_Idx], pRightBuffer->getBufferSize());

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

	//wait for the end of thread
	leftThread .join();
	rightThread.join();

	//pSink->snapImages( NUM_BUFFERS );	// Grab NUM_BUFFERS images.

	leftGrabber.stopLive();					// Stop the grabber.
	rightGrabber.stopLive();					// Stop the grabber.
	
	// The CListener object must be unregistered for all events
	// before it may be destroyed.
	leftGrabber .removeListener( pLeftListener );
	rightGrabber.removeListener( pRightListener );

	// Now, it must be checked whether the CListener object has been unregistered
	// for all events.
	while( leftGrabber.isListenerRegistered( pLeftListener ) || rightGrabber.isListenerRegistered( pRightListener ) )
	{
		Sleep( 0 ); // Wait and give pending callbacks a chance to complete.
	}

	// Now, the application can be sure that no callback methods of pListener
	// will be called anymore. It is now safe to delete pListener.
	delete pLeftListener;
	delete pRightListener;

	std::cout << "Press any key to continue!" << std::endl;
	std::cin.get();
	return 0;
}

