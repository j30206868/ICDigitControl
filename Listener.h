// Listener.h: interface for the CListener class.
// 
// The CListener class is derived from GrabberListener. It overwrites
// the "frameReady()" method. In the frameReady method the member method
// "saveImage()" is called.
// "saveImage()" saves the specified buffer to a BMP file and calls a "Sleep(250)" 
// to simulate a time expensive image processing. "saveImage()" is also called
// by the main() function of this example to save all buffers, that have
// not been processed in the frameReady method.#
//
// This class also overwrites the overlayCallback method to draw a 
// frame counter.
//
// The CListener object is registered to a Grabber with the parameter 
// eFRAMEREADY|eOVERLAYCALLBACK . 
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_LISTENER_H__3E017E1D_6B0A_472C_9F9C_0C5F9A8DFB23__INCLUDED_)
#define AFX_LISTENER_H__3E017E1D_6B0A_472C_9F9C_0C5F9A8DFB23__INCLUDED_

#pragma once

#include <tisudshl.h>

class CListener : public DShowLib::GrabberListener  
{
public:
	// Overwrite the GrabberListener methods we need
	virtual void overlayCallback( DShowLib::Grabber& caller, smart_ptr<DShowLib::OverlayBitmap> pBitmap, const DShowLib::tsMediaSampleDesc& MediaSampleDesc ); 
	virtual void frameReady( DShowLib::Grabber& caller, smart_ptr<DShowLib::MemBuffer> pBuffer, DWORD FrameNumber ); 

	// Save one image and mark it as saved
	void		saveImage( smart_ptr<DShowLib::MemBuffer> pBuffer, DWORD currFrame );
	// Setup the buffersize. 
	void		setBufferSize( unsigned long NumBuffers );

	std::vector<bool>	m_BufferWritten;	// array of flags which buffers have been saved.

	int imgBufferSize;
	int imgBufHeadIdx;
	BYTE **imgBuffer;
};

class cwz_timer{
public:
    static void start();
    static void t_start();
    static double stop();
    static double t_stop();
    static void time_display(char *disp,int nr_frame=1);
    static void t_time_display(char *disp,int nr_frame=1);
private:
    static double m_pc_frequency;
    static __int64 m_counter_start;
    static double t_pc_frequency;
    static __int64 t_counter_start;
};


#endif // !defined(AFX_LISTENER_H__3E017E1D_6B0A_472C_9F9C_0C5F9A8DFB23__INCLUDED_)
