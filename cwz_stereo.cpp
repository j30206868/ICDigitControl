#include "cwz_stereo.h"

using namespace cv;
using namespace std;

void SAM_STEREO_READER::init(){
	w = 640;
	h = 480;
	recording = false;
}
void SAM_STEREO_READER::stop(){
	recording = false;
}
int SAM_STEREO_READER::start(DUAL_VCD_CALLBACK *callback){
	recording = true;

	cv::VideoCapture leftCap(0); // open the default camera
	cv::VideoCapture rightCap(1); // open the default camera
	if(!rightCap.isOpened())  // check if we succeeded
		return 0;
	if(!leftCap.isOpened())  // check if we succeeded
		return 0;

	bool hasShowedImgInfo = false;

	while(recording){

		cv::Mat left, right, leftC, rightC;
        rightCap >> leftC;
		leftCap >> rightC;
		if(leftC.empty() || rightC.empty())
			continue;
		
		if(!hasShowedImgInfo){
			printf("Image Dimension(%d, %d)\n", leftC.cols, leftC.rows);
			hasShowedImgInfo = true;
		}

		//cv::cvtColor(leftC , left , cv::COLOR_BGR2GRAY);
		//cv::cvtColor(rightC, right, cv::COLOR_BGR2GRAY);

		callback->imgProc(leftC, rightC);
	}

	return 1;
}

void CALIB_PROC::init(){
	calib_img_path     = "Calibration/";
	img_list_xml_path  = "Calibration/";
	img_list_xml_fname = "StereoCalibImageList.xml";
	img_ext            = ".bmp";
	//
	proj_calib_file_path = "Projector/";
	//
	calib_frame_number = 0;
	img_list_fstream_opened = false;
	valid_calib_param = false;
	valid_proj_calib_param = false;
	readCameraIntrAndExtr();
	readProjIntrAndExtr();
}
void CALIB_PROC::openImgListFileStream(){
	std::stringstream sstm;
	sstm << img_list_xml_path << img_list_xml_fname;
	fs.open(sstm.str());
	fs << "<?xml version=\"1.0\"?>" << std::endl;
	fs << "<opencv_storage>" << std::endl;
	fs << "<imagelist>" << std::endl;
	img_list_fstream_opened = true;
	valid_calib_param = false;
	printf("CALIB_PROC Stream On ; valid_calib_param -> false\n");
}
void CALIB_PROC::writeToImgListFileStream(std::string fname){
	fs << "\"" << fname << "\"" << std::endl;
}
/*void CALIB_PROC::saveNewStereoAndDisp(cv::Mat left, cv::Mat right, cv::Mat disp){

}*/
void CALIB_PROC::saveNewCalibImg(cv::Mat left, cv::Mat right){
	char fileNumberStr[20];
	sprintf(fileNumberStr, "%02i", calib_frame_number+1);
	std::stringstream sstm;
	//
	sstm << calib_img_path << "left" << fileNumberStr << img_ext;
	std::cout << "CALIB_PROC write new file: " << sstm.str() << std::endl;
	cv::imwrite(sstm.str(), left);
	writeToImgListFileStream(sstm.str());
	//
	sstm.str("");
	//
	sstm << calib_img_path << "right" << fileNumberStr << img_ext;
	std::cout << "CALIB_PROC write new file: " << sstm.str() << std::endl;
	cv::imwrite(sstm.str(), right);
	writeToImgListFileStream(sstm.str());

	calib_frame_number++;
}
void CALIB_PROC::closeImgListFileStream(){
	if(img_list_fstream_opened){
		fs << "</imagelist>" << std::endl;
		fs << "</opencv_storage>" << std::endl;
		fs.close();
		printf("CALIB_PROC Stream Off\n");
		calib_frame_number = 0;
		img_list_fstream_opened = false;
	}
}
inline  bool readStringList( const std::string& filename, cv::vector<std::string>& l )
{
    l.resize(0);
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    cv::FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != cv::FileNode::SEQ )
        return false;
    cv::FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((std::string)*it);
    return true;
}
static void
StereoCalib(std::string calib_yml_path, const vector<string>& imagelist, Size boardSize, bool useCalibrated=true, bool showRectified=true)
{
    if( imagelist.size() % 2 != 0 )
    {
        cout << "Error: the image list contains odd (non-even) number of elements\n";
        return;
    }

    bool displayCorners = false;//true;
    const int maxScale = 2;
    const float squareSize = 0.5f;  // Set this to your actual square size
    // ARRAY AND VECTOR STORAGE:

    vector<vector<Point2f> > imagePoints[2];
    vector<vector<Point3f> > objectPoints;
    Size imageSize;

    int i, j, k, nimages = (int)imagelist.size()/2;

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    vector<string> goodImageList;

    for( i = j = 0; i < nimages; i++ )
    {
        for( k = 0; k < 2; k++ )
        {
            const string& filename = imagelist[i*2+k];

			std::cout << "filename: " << filename << std::endl;

            Mat img = imread(filename, 0);
            if(img.empty())
                break;
            if( imageSize == Size() )
                imageSize = img.size();
            else if( img.size() != imageSize )
            {
                cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
                break;
            }
            bool found = false;
            vector<Point2f>& corners = imagePoints[k][j];
            for( int scale = 1; scale <= maxScale; scale++ )
            {
                Mat timg;
                if( scale == 1 )
                    timg = img;
                else
                    resize(img, timg, Size(), scale, scale);
                found = findChessboardCorners(timg, boardSize, corners,
                    CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FILTER_QUADS);
                if( found )
                {
                    if( scale > 1 )
                    {
                        Mat cornersMat(corners);
                        cornersMat *= 1./scale;
                    }
                    break;
                }
            }
            if( displayCorners )
            {
                cout << filename << endl;
                Mat cimg, cimg1;
                cvtColor(img, cimg, COLOR_GRAY2BGR);
                drawChessboardCorners(cimg, boardSize, corners, found);
                double sf = 640./MAX(img.rows, img.cols);
                resize(cimg, cimg1, Size(), sf, sf);
                imshow("corners", cimg1);
                char c = (char)waitKey(0);
                if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
                    exit(-1);
            }
            else
                putchar('.');
            if( !found )
                break;
            cornerSubPix(img, corners, Size(11,11), Size(-1,-1),
                         TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
                                      30, 0.01));
        }
        if( k == 2 )
        {
            goodImageList.push_back(imagelist[i*2]);
            goodImageList.push_back(imagelist[i*2+1]);
            j++;
        }
    }
    cout << j << " pairs have been successfully detected.\n";
    nimages = j;
    if( nimages < 2 )
    {
        cout << "Error: too little pairs to run the calibration\n";
        return;
    }

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    objectPoints.resize(nimages);

    for( i = 0; i < nimages; i++ )
    {
        for( j = 0; j < boardSize.height; j++ )
            for( k = 0; k < boardSize.width; k++ )
                objectPoints[i].push_back(Point3f(j*squareSize, k*squareSize, 0));
    }

    cout << "Running stereo calibration ...\n";

    Mat cameraMatrix[2], distCoeffs[2];
    cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
    cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
    Mat R, T, E, F;

	double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                    cameraMatrix[0], distCoeffs[0],
                    cameraMatrix[1], distCoeffs[1],
                    imageSize, R, T, E, F,
                    TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                    CV_CALIB_FIX_ASPECT_RATIO +
                    CV_CALIB_ZERO_TANGENT_DIST +
                    CV_CALIB_SAME_FOCAL_LENGTH +
                    CV_CALIB_RATIONAL_MODEL +
                    CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);
    /*double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                    cameraMatrix[0], distCoeffs[0],
                    cameraMatrix[1], distCoeffs[1],
                    imageSize, R, T, E, F,
                    TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                    CV_CALIB_FIX_ASPECT_RATIO +
                    CV_CALIB_ZERO_TANGENT_DIST +
                    CV_CALIB_SAME_FOCAL_LENGTH +
                    CV_CALIB_RATIONAL_MODEL +
                    CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);*/
    cout << "done with RMS error=" << rms << endl;

// CALIBRATION QUALITY CHECK
// because the output fundamental matrix implicitly
// includes all the output information,
// we can check the quality of calibration using the
// epipolar geometry constraint: m2^t*F*m1=0
    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];
    for( i = 0; i < nimages; i++ )
    {
        int npt = (int)imagePoints[0][i].size();
        Mat imgpt[2];
        for( k = 0; k < 2; k++ )
        {
            imgpt[k] = Mat(imagePoints[k][i]);
            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }
        for( j = 0; j < npt; j++ )
        {
            double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
                                imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
                           fabs(imagePoints[1][i][j].x*lines[0][j][0] +
                                imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    cout << "average reprojection err = " <<  err/npoints << endl;

    // save intrinsic parameters
	std::stringstream sstm;
	sstm << calib_yml_path << "intrinsics.yml";
    FileStorage fs(sstm.str(), CV_STORAGE_WRITE);
    if( fs.isOpened() )
    {
        fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
            "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
        fs.release();
    }
    else
        cout << "Error: can not save the intrinsic parameters\n";

    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];

    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
	sstm.str("");
	sstm << calib_yml_path << "extrinsics.yml";
    fs.open(sstm.str(), CV_STORAGE_WRITE);
    if( fs.isOpened() )
    {
        fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs << "ROI1" << validRoi[0] << "ROI2" << validRoi[1];
		fs << "ImageSize" << imageSize;
        fs.release();
    }
    else
        cout << "Error: can not save the intrinsic parameters\n";

    // OpenCV can handle left-right
    // or up-down camera arrangements
    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

// COMPUTE AND DISPLAY RECTIFICATION
    if( !showRectified )
        return;

    Mat rmap[2][2];
// IF BY CALIBRATED (BOUGUET'S METHOD)
    if( useCalibrated )
    {
		printf("BOUGUET'S METHOD\n");
        // we already computed everything
    }
// OR ELSE HARTLEY'S METHOD
    else
 // use intrinsic parameters of each camera, but
 // compute the rectification transformation directly
 // from the fundamental matrix
    {
		printf("HARTLEY'S METHOD\n");
        vector<Point2f> allimgpt[2];
        for( k = 0; k < 2; k++ )
        {
            for( i = 0; i < nimages; i++ )
                std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
        }
        F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
        Mat H1, H2;
        stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

        R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
        R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
        P1 = cameraMatrix[0];
        P2 = cameraMatrix[1];
    }

    //Precompute maps for cv::remap()
    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    Mat canvas;
    double sf;
    int w, h;
    if( !isVerticalStereo )
    {
        sf = 600./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h, w*2, CV_8UC3);
    }
    else
    {
        sf = 300./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h*2, w, CV_8UC3);
    }

	std::cout << "show rectified images, nimages = " << nimages << std::endl;
    for( i = 0; i < nimages; i++ )
    {
        for( k = 0; k < 2; k++ )
        {
            Mat img = imread(goodImageList[i*2+k], 0), rimg, cimg;

            remap(img, rimg, rmap[k][0], rmap[k][1], CV_INTER_LINEAR);
			
			cvtColor(rimg, cimg, COLOR_GRAY2BGR);

            Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
            resize(cimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
            if( useCalibrated )
            {
                Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
                          cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
                rectangle(canvasPart, vroi, Scalar(0,0,255), 3, 8);
            }
        }

        if( !isVerticalStereo )
            for( j = 0; j < canvas.rows; j += 16 )
                line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
        else
            for( j = 0; j < canvas.cols; j += 16 )
                line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
        imshow("rectified", canvas);
        char c = (char)waitKey();
        if( c == 27 || c == 'q' || c == 'Q' )
            break;
    }
}
void CALIB_PROC::readCameraIntrAndExtr(){
	std::stringstream isstm, esstm;
	isstm << img_list_xml_path << "intrinsics.yml";
	esstm << img_list_xml_path << "extrinsics.yml";

	if( checkIfFileExist(isstm.str()) && checkIfFileExist(esstm.str())){
		FileStorage fsi(isstm.str(), FileStorage::READ);
		fsi["M1"] >> cameraMatrix[0];
		fsi["M2"] >> cameraMatrix[1];
		fsi["D1"] >> distCoeffs[0];
		fsi["D2"] >> distCoeffs[1];
		fsi.release();

		FileStorage fse(esstm.str(), FileStorage::READ);
		fse["R"] >> R;
		fse["T"] >> T;
		fse["R1"] >> R1;
		fse["R2"] >> R2;
		fse["P1"] >> P1;
		fse["P2"] >> P2;
		fse["Q"] >> Q;
		fse["ROI1"] >> validRoi[0];
		fse["ROI2"] >> validRoi[1];
		fse["ImageSize"] >> imageSize;
		fse.release();
	}

	if(R.empty() || T.empty() || Q.empty() || R1.empty() || R2.empty() || P1.empty() || P2.empty()){
		valid_calib_param = false;
	}else{
		initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
		initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
		
		int maxSX = max(validRoi[0].x, validRoi[1].x);
		int minEX = min(validRoi[0].x + validRoi[0].width - 1, validRoi[1].x + validRoi[1].width - 1);
		int maxSY = max(validRoi[0].y, validRoi[1].y);
		int minEY = min(validRoi[0].y + validRoi[0].height - 1, validRoi[1].y + validRoi[1].height - 1);

		commonRoi = cv::Rect(maxSX, maxSY, minEX-maxSX+1, minEY-maxSY+1);
		printf("commonRoi(%d, %d, %d, %d)\n", commonRoi.x, commonRoi.y, commonRoi.width, commonRoi.height);
		
		valid_calib_param = true;
		printf("CALIB_PROC: valid_calib_param = true\n");
	}
}
void CALIB_PROC::readProjIntrAndExtr(){
	std::stringstream sstm;
	sstm << proj_calib_file_path << "calibration.yml";

	if( checkIfFileExist(sstm.str()) ){
		FileStorage fsi(sstm.str(), FileStorage::READ);
		fsi["cam_K"]			 >> cam_K;
		fsi["cam_kc"]		 	 >> cam_kc;
		fsi["proj_K"]			 >> proj_K;
		fsi["proj_kc"]			 >> proj_kc;
		fsi["R"]				 >> proj_R;
		fsi["T"]				 >> proj_T;
		fsi["CamProjHomography"] >> CamProjHomography;
		fsi.release();
	}
	if( cam_K.empty() || cam_kc.empty() || proj_K.empty() || proj_kc.empty() || proj_R.empty() || proj_T.empty() ){
		valid_proj_calib_param = false;
	}else{
		valid_proj_calib_param = true;
		printf("CALIB_PROC: valid_proj_calib_param = true\n");
	}
}
void CALIB_PROC::stereoCalibAndRectify(){
	cv::Size boardSize = cv::Size(17, 13);
    bool showRectified = true;

	std::stringstream sstm;
	sstm << img_list_xml_path << img_list_xml_fname;

	cv::vector<std::string> imagelist;
    bool ok = readStringList(sstm.str(), imagelist);
    if(!ok || imagelist.empty())
    {
        std::cout << "can not open " << sstm.str() << " or the string list is empty" << std::endl;
    }else
		StereoCalib(img_list_xml_path, imagelist, boardSize, true, showRectified);

	readCameraIntrAndExtr();
}
bool CALIB_PROC::checkIfFileExist (const std::string& name) {
    if (FILE *file = fopen(name.c_str(), "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }   
}
void CALIB_PROC::remapAndDrawRoi(cv::Mat &left, cv::Mat &right){
	if( valid_calib_param ){
		cv::Mat l_t = left.clone();
		cv::Mat r_t = right.clone();
		remap(l_t, left, rmap[0][0], rmap[0][1], CV_INTER_LINEAR);
		remap(r_t, right, rmap[1][0], rmap[1][1], CV_INTER_LINEAR);

		rectangle(left, validRoi[0], cv::Scalar(255), 3, 8);
		rectangle(right, validRoi[1], cv::Scalar(255), 3, 8);
	}
}
void CALIB_PROC::remapAndCutRoi(cv::Mat &left, cv::Mat &right){
	if( valid_calib_param ){
		cv::Mat l_t = left.clone();
		cv::Mat r_t = right.clone();
		remap(l_t, left, rmap[0][0], rmap[0][1], CV_INTER_LINEAR);
		remap(r_t, right, rmap[1][0], rmap[1][1], CV_INTER_LINEAR);
		//left  = left(commonRoi);
		//right = right(commonRoi);

		cv::Mat ltmp = left.clone();
		cv::Mat rtmp = right.clone();
		left  = cv::Mat(commonRoi.height, commonRoi.width, l_t.type());
		right = cv::Mat(commonRoi.height, commonRoi.width, l_t.type());
		int channel;
		if(l_t.type() == CV_8UC3)
			channel = 3;
		else
			channel = 1;
		int newid = 0;
		int sid   = commonRoi.y * ltmp.cols * channel + commonRoi.x * channel;
		int len   = (commonRoi.y+commonRoi.height) * ltmp.cols * channel;
		int step  = ltmp.cols * channel;
		int cpybytes = sizeof(uchar) * commonRoi.width * channel;
		int newid_step = left.cols * channel;
		for(int i=sid ; i<len ; i+=step){
			memcpy(&left.data[newid] , &ltmp.data[i], cpybytes);
			memcpy(&right.data[newid], &rtmp.data[i], cpybytes);
			newid+=newid_step;
		}
	}
}
void CALIB_PROC::remapping(cv::Mat &left, cv::Mat &right){
	if( valid_calib_param ){
		cv::Mat l_t = left.clone();
		cv::Mat r_t = right.clone();
		remap(l_t, left, rmap[0][0], rmap[0][1], CV_INTER_LINEAR);
		remap(r_t, right, rmap[1][0], rmap[1][1], CV_INTER_LINEAR);
	}
}
void CALIB_PROC::cutRoi(cv::Mat &left, cv::Mat &right){
	if( valid_calib_param ){
		//left  = left(commonRoi);
		//right = right(commonRoi);
		cv::Mat ltmp = left.clone();
		cv::Mat rtmp = right.clone();
		left  = cv::Mat(commonRoi.height, commonRoi.width, ltmp.type());
		right = cv::Mat(commonRoi.height, commonRoi.width, ltmp.type());
		int channel;
		if(ltmp.type() == CV_8UC3)
			channel = 3;
		else
			channel = 1;
		int newid = 0;
		int sid   = commonRoi.y * ltmp.cols * channel + commonRoi.x * channel;
		int len   = (commonRoi.y+commonRoi.height) * ltmp.cols * channel;
		int step  = ltmp.cols * channel;
		int cpybytes = sizeof(uchar) * commonRoi.width * channel;
		int newid_step = left.cols * channel;
		for(int i=sid ; i<len ; i+=step){
			memcpy(&left.data[newid] , &ltmp.data[i], cpybytes);
			memcpy(&right.data[newid], &rtmp.data[i], cpybytes);
			newid+=newid_step;
		}
	}
	//left  = remapLeft(validRoi[0]);
	//right = remapRight(validRoi[1]);
}