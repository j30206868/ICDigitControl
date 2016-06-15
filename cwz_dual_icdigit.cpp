#include "cwz_dual_icdigit.h"

int DUAL_VCD_READER::init(bool openWithXML){
	if(openWithXML){
		leftGrabber.loadDeviceStateFromFile("ICDigitXML/leftDeviceState.xml");
		rightGrabber.loadDeviceStateFromFile("ICDigitXML/rightDeviceState.xml");
	}
			
	if(!leftGrabber.isDevValid() || !rightGrabber.isDevValid()){
		if( !setupDeviceFromFile( leftGrabber, "ICDigitXML/leftDeviceState.xml") )
		{
			return 0;
		}
		if( !setupDeviceFromFile( rightGrabber, "ICDigitXML/rightDeviceState.xml" ) )
		{
			return 0;
		}
	}

	// Enable the overlay bitmap to display the frame counter in the live video.
	leftGrabber.getOverlay()->setEnable( true );
	rightGrabber.getOverlay()->setEnable( true );

	return 1;
}

void DUAL_VCD_READER::stop(){
	recording = false;
}

void DUAL_VCD_READER::start(DUAL_VCD_CALLBACK *callback){
	// Create a FrameTypeInfoArray data structure describing the allowed color formats.
	FrameTypeInfo leftTypeInfo ( DUAL_VCD_COLOR_TYPE, AcqImgWidth, AcqImgHeight );
	FrameTypeInfo rightTypeInfo( DUAL_VCD_COLOR_TYPE, AcqImgWidth, AcqImgHeight );

	// Create the frame handler sink
	smart_ptr<FrameHandlerSink> pLeftSink  = FrameHandlerSink::create( leftTypeInfo , NUM_BUFFERS );
	smart_ptr<FrameHandlerSink> pRightSink = FrameHandlerSink::create( rightTypeInfo, NUM_BUFFERS );

	// set snap mode (用snap mode 多跑幾個執行序, 搶在一有影像輸出時 馬上就copy到)
	pLeftSink ->setSnapMode( true );
	pRightSink->setSnapMode( true );

	// Apply the sink to the grabber.
	leftGrabber .setSinkType( pLeftSink );
	rightGrabber.setSinkType( pRightSink );

	leftGrabber .startLive(false);				// Start the grabber.
	rightGrabber.startLive(false);				// Start the grabber.

	cv::Mat left;
	cv::Mat right;
	int frameCount = 1;

	//
	smart_ptr<MemBuffer> pLeftBuffer  = pLeftSink ->getMemBufferCollection()->getBuffer( 0 );
	smart_ptr<MemBuffer> pRightBuffer = pRightSink->getMemBufferCollection()->getBuffer( 0 );
	left  = cv::Mat(pLeftBuffer->getSize().cy, pLeftBuffer->getSize().cx, CV_8UC1);
	right = cv::Mat(pRightBuffer->getSize().cy, pRightBuffer->getSize().cx, CV_8UC1);

	//show information
	std::cout << "==== Image Information ====" << std::endl;
	std::cout << "Image Dimension     : (" << pLeftBuffer->getSize().cx << ", " << pLeftBuffer->getSize().cy << ")" << std::endl;
	std::cout << "Bytes for each frame: " << pLeftBuffer->getBufferSize() << std::endl;
	std::cout << "Bits per pixel      : " << pLeftBuffer->getBitsPerPixel() << std::endl;


	const int poolSize = 500;
	std::thread leftThreads[poolSize];
	std::thread rightThreads[poolSize];
	for(int i=0 ; i<poolSize ; i++){
		leftThreads[i]  = std::thread(snapImg, pLeftSink , &recording);
		rightThreads[i] = std::thread(snapImg, pRightSink, &recording);
		//Sleep(2);
	}
	//loop to get newest image
	//std::thread leftThread  (snapImg, pLeftSink , &recording);
	//std::thread rightThread (snapImg, pRightSink, &recording);
	
	recording = true;
	while(recording){
		memcpy(left.data, pLeftSink->getLastAcqMemBuffer()->getPtr(), pLeftBuffer->getBufferSize());
		memcpy(right.data, pRightSink->getLastAcqMemBuffer()->getPtr(), pRightBuffer->getBufferSize());

		callback->imgProc(left, right);
	}

	//wait for the end of thread
	for(int i=0 ; i<poolSize ; i++){
		leftThreads[i].join();
		rightThreads[i].join();
	}
	//leftThread .join();
	//rightThread.join();

	//pSink->snapImages( NUM_BUFFERS );	// Grab NUM_BUFFERS images.
	leftGrabber.stopLive();					// Stop the grabber.
	rightGrabber.stopLive();					// Stop the grabber.
}

void snapImg(smart_ptr<FrameHandlerSink> pSink, bool *running){
	while(*running){
		//pSink ->snapImagesAsync( NUM_BUFFERS );	// Grab NUM_BUFFERS images.
		//Sleep(10);
		pSink ->snapImages( 1, 70UL );
	}
}
