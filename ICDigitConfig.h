#ifndef H_ICDIGITCONFIG_H

#define _WIN32_WINNT 0x0500

#include <iostream>
#include <conio.h>

#include <tisudshl.h>

#include "CmdHelper.h"

#include <thread>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace _DSHOWLIB_NAMESPACE;

// Specify the number of buffers to be used.
#define NUM_BUFFERS 15

static const int AcqImgWidth  = 1280;
static const int AcqImgHeight = 960;

#endif //H_ICDIGITCONFIG_H