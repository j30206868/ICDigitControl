// Listener.cpp: implementation of the CListener class.
//
//////////////////////////////////////////////////////////////////////

#define _WIN32_WINNT 0x0500

#include <iostream>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Listener.h"

using namespace DShowLib;

//////////////////////////////////////////////////////////////////////////
/*! The overlayCallback() method draws the number of the current frame. The
	frame count is a member of the tsMediaSampleDesc structure that is passed
	to overlayCallback() by the Grabber.
*/
void	CListener::overlayCallback( Grabber& caller, smart_ptr<OverlayBitmap> pBitmap, 
								   const tsMediaSampleDesc& MediaSampleDesc)
{
	char szText[25];
	if( pBitmap->getEnable() == true ) // Draw only, if the overlay bitmap is enabled.
	{
		sprintf( szText,"%04d ", MediaSampleDesc.FrameNumber);
		pBitmap->drawText( RGB(255,0,0), 0, 0, szText );
	}
}

//////////////////////////////////////////////////////////////////////////
/*! The frameReady() method calls the saveImage method to save the image buffer to disk.
*/

void	CListener::frameReady( Grabber& caller, smart_ptr<MemBuffer> pBuffer, DWORD currFrame)
{
	/*cv::Mat img = cv::Mat(pBuffer->getSize().cy, pBuffer->getSize().cx, CV_8UC1);
	img.data = pBuffer->getPtr();
	//std::cout << "Bits per pixel: " << pBuffer->getBitsPerPixel() << std::endl;
	//std::cout << "getBufferSize: " << pBuffer->getBufferSize() << std::endl;
	//std::cout << "getSize: (" << pBuffer->getSize().cx << ", " << pBuffer->getSize().cy << ")" << std::endl;
	//std::cout << "Buffer " << currFrame << " processed in CListener::frameReady()." << std::endl;

	if( !frameSaved ){
		saveImage( pBuffer, frameCout ); // Do the buffer processing.
		frameSaved = true;
	}

	cv::namedWindow(prefix, CV_WINDOW_NORMAL);
	cv::imshow(prefix, img);
	cvWaitKey(1);*/

	memcpy(imgBuffer[imgBufHeadIdx], pBuffer->getPtr(), pBuffer->getBufferSize());
	//increase buffer head pointer
	if((imgBufHeadIdx+1) == imgBufferSize)
		imgBufHeadIdx = 0;
	else
		imgBufHeadIdx++;

	//::Sleep(250); // Simulate a time expensive processing.
}

//////////////////////////////////////////////////////////////////////////
/*! Initialize the array of bools that is used to memorize, which buffers were processed in 
	the frameReady() method. The size of the array is specified by the parameter NumBuffers.
	It should be equal to the number of buffers in the FrameHandlerSink.
	All members of m_BufferWritten are initialized to false.
	This means that no buffers have been processed.
*/
void	CListener::setBufferSize( unsigned long NumBuffers )
{
	m_BufferWritten.resize( NumBuffers, false );
}

//////////////////////////////////////////////////////////////////////////
/*! The image passed by the MemBuffer pointer is saved to a BMP file.
*/
void	CListener::saveImage( smart_ptr<MemBuffer> pBuffer, DWORD currFrame)
{
	char filename[MAX_PATH];
	//if( currFrame < m_BufferWritten.size() )
	//{
		sprintf( filename, "%02i.bmp", currFrame );
		saveToFileBMP( *pBuffer, filename );

		//m_BufferWritten.at( currFrame ) = true;
	//}
}
//>>

double  cwz_timer::m_pc_frequency = 0;
__int64 cwz_timer::m_counter_start = 0;
double  cwz_timer::t_pc_frequency = 0;
__int64 cwz_timer::t_counter_start = 0;

void cwz_timer::start()
{
    //m_begin=clock();

    LARGE_INTEGER li;
    if(!QueryPerformanceFrequency(&li))
        std::cout << "QueryPerformanceFrequency failed!\n";

    m_pc_frequency = double(li.QuadPart);///1000.0;

    QueryPerformanceCounter(&li);
    m_counter_start = li.QuadPart;
}
void cwz_timer::t_start()
{
    //m_begin=clock();

    LARGE_INTEGER li;
    if(!QueryPerformanceFrequency(&li))
        std::cout << "QueryPerformanceFrequency failed!\n";

    t_pc_frequency = double(li.QuadPart);///1000.0;

    QueryPerformanceCounter(&li);
    t_counter_start = li.QuadPart;
}
double cwz_timer::t_stop(){
    LARGE_INTEGER li;
    QueryPerformanceCounter(&li);
    return double(li.QuadPart-t_counter_start)/t_pc_frequency;
}
double cwz_timer::stop()
{
    //m_end=clock(); return ( double(m_end-m_begin)/CLOCKS_PER_SEC );
    LARGE_INTEGER li;
    QueryPerformanceCounter(&li);
    return double(li.QuadPart-m_counter_start)/m_pc_frequency;
}
void cwz_timer::time_display(char *disp,int nr_frame){
    printf("Running time (%s) is: %5.5f Seconds.",disp,stop()/nr_frame);
    //std::cout << "Running time (" << disp << ") is: " << stop()/nr_frame << "Seconds." << std::endl;
    std::cout << std::endl;
}
void cwz_timer::t_time_display(char *disp,int nr_frame){ printf("Running time (%s) is: %5.5f Seconds.\n",disp,t_stop()/nr_frame);}

