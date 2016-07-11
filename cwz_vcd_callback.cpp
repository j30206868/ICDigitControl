#include "cwz_vcd_callback.h"

void keyLog(char *inputKey, bool *isKeyPressed, bool *isKeyClicked, bool *running){
	//�ԲӪ�Virtual Key Code��Ӫ�Ш�https://msdn.microsoft.com/en-us/library/windows/desktop/dd375731(v=vs.85).aspx
	//Virtual Key Code�S���j�p�g����, �u�Ophysically mapping����L
	//�]�i�H����ƹ�, ���L�ƹ������k��Ophysically mapping, �i��|�A��
	char chType[]="ABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890";
	while(*running)
	{
		if(*inputKey == '\0')
			for (int i=0; i<MY_VCD_CALLBACK_KEY_LOG_AMT; i++){
				if( GetAsyncKeyState(chType[i]) ){
					if(isKeyPressed[i] == false){
						//isKeyClicked[i] = true;
						*inputKey = chType[i];
						//printf("%c clicked\n", *inputKey);
					}
					
					isKeyPressed[i] = true;
				}else{
					isKeyPressed[i] = false;
				}
			}
		//every 100 ms
		Sleep(100);
	}
}

MyVCDCallback::MyVCDCallback(){
	frameCount = 1;
	isCalibrationOn = false;
	calib_proc.init();

	runningLogger = true;
	inputKey = '\0';
	memset(isKeyPressed, false, sizeof(bool) * MY_VCD_CALLBACK_KEY_LOG_AMT);
	memset(isKeyClicked, false, sizeof(bool) * MY_VCD_CALLBACK_KEY_LOG_AMT);
	keyLogger = std::thread(keyLog, &this->inputKey, this->isKeyPressed, this->isKeyClicked, &this->runningLogger);
	

	//disparity
	isDispInit = false;
	device = setup_opencl(context, err);
	program = load_program(context, "clkernel/test.cl");
	if(program == 0) { std::cerr << "Can't load or build program\n"; clReleaseContext(context); system("PAUSE"); }
	disp_fcount = 0;

	//proj
	is_left_2d_points_init = false;
}
MyVCDCallback::~MyVCDCallback(){
	runningLogger = false;
	keyLogger.join();
}

static bool highPassOn = false;
void MyVCDCallback::imgProc(cv::Mat left, cv::Mat right){
	char inputKeyAtBegin = inputKey;

	flipLeftRight(left, right);

	if(inputKeyAtBegin == 'H'){
		highPassOn = highPassOn? false : true;
	}

	if(highPassOn){
		cv::Mat blurLeft  = left.clone();
		cv::Mat blurRight = right.clone();
		cv::blur(blurLeft, left, cv::Size(20, 20));
		cv::blur(blurRight, right, cv::Size(20, 20));
		int ch;
		if(left.type() == CV_8UC3)
			ch = 3;
		else
			ch = 1;

		highPass(left, ch, 50);
		histogramEqualize(left, ch);
		highPass(right, ch, 50);
		histogramEqualize(right, ch);
	}	

	showLeftRight("Raw_", left, right);

	//calib_proc.remapAndDrawRoi(left, right);
	calib_proc.remapping(left, right);
	//calib_proc.remapAndCutRoi(left, right);

	showLeftRightMerged(left, right);
	//showLeftRight("Rectified_", left, right);

	//printf("img cut dimension(%d, %d)\n", left.cols, left.rows);
	if(calib_proc.valid_calib_param){
		if(!isDispInit){			
			for(subsample_divider = 1 ;; subsample_divider*=2){
				if(left.type() == CV_8UC3)
					info = createMatchInfo(left.cols/subsample_divider, left.rows/subsample_divider, 3);
				else
					info = createMatchInfo(left.cols/subsample_divider, left.rows/subsample_divider, 1);

				if(info->max_x_d < 230)
					break;
				else
					delete info;
			}
			info->printf_match_info("Subsample Info");
			printf("max_x_disparity: %d\n", info->max_x_d);
			printf("subsample_divider: %d\n", subsample_divider);

			cv::Mat left_b = left.clone();
			cv::Mat right_b = right.clone();
			cv::resize(left_b, left, cv::Size(left_b.cols/subsample_divider, left_b.rows/subsample_divider));
			cv::resize(right_b, right, cv::Size(right_b.cols/subsample_divider, right_b.rows/subsample_divider));
			printf("resize left right dimension(%d, %d)\n", left.cols, left.rows);

			dmap_generator.init(context, device, program, err, left, right, *info);
			dmap_ref.init(dmap_generator.mst_L, *info, dmap_generator.left_dmap, dmap_generator.right_dmap);

			printf("MyVCDCallback: disp init finished\n");
			isDispInit = true;

			if(inputKeyAtBegin == 'D')
				inputKeyAtBegin = '\0';
		}

		if(subsample_divider > 1){
			cv::Mat left_b = left.clone();
			cv::Mat right_b = right.clone();
			cv::resize(left_b, left, cv::Size(left_b.cols/subsample_divider, left_b.rows/subsample_divider));
			cv::resize(right_b, right, cv::Size(right_b.cols/subsample_divider, right_b.rows/subsample_divider));
		}

		//
		//inputKeyAtBegin='D';
		//left = cv::imread("raw/left1.bmp");
		//right = cv::imread("raw/right1.bmp");
		//

		if(inputKeyAtBegin == 'D' && isDispInit==true){
			dmap_generator.set_left_right(left, right);

			dmap_generator.filtering();
			dmap_generator.compute_cwz_img();

			CWZDISPTYPE *left_dmap;
			CWZDISPTYPE *right_dmap;
			CWZDISPTYPE *refined_dmap;

			if( !(left_dmap = dmap_generator.generate_left_dmap()) )
			{printf( "cwz_dmap_generate left_dmap failed...!" );system("PAUSE");}

			if( !(right_dmap = dmap_generator.generate_right_dmap()) )
			{printf( "cwz_dmap_generate right_dmap failed...!" );system("PAUSE");}

			cwz_mst::updateSigma(cwz_mst::sigma / 2);
			refined_dmap = dmap_ref.refinement(dmap_refine::MODE_TREE);
			cwz_mst::updateSigma(cwz_mst::sigma * 2);

			for(int i=0 ; i<info->node_c ; i++){
				if(dmap_generator.left_gray_1d_arr[i] >= 150){
					refined_dmap[i] = 0;
				}
			}

			//applyHomography(left, subsample_divider);
			cv::Mat proj = cv::Mat(left.rows, left.cols, CV_8UC1);
			memset(proj.data, 255, sizeof(uchar) * left.rows * left.cols);
			convertLeftToProj(left, right, refined_dmap, proj, subsample_divider);

			if(false)
				show_cv_img("left_dmap", left_dmap, info->img_height, info->img_width, 1, false);
			if(false)
				show_cv_img("right_dmap", right_dmap, info->img_height, info->img_width, 1, false);

			show_cv_img(0, "Disparity Map", refined_dmap, info->img_height, info->img_width, 1, true);

			write_cv_img(disp_fcount+1, "raw/left", left);
			write_cv_img(disp_fcount+1, "raw/right", right);
			write_cv_img(disp_fcount+1, "raw/disp", refined_dmap, info->img_height, info->img_width, CV_8UC1);
			disp_fcount++;
	
			dmap_generator.mst_L.reinit();
			dmap_generator.mst_R.reinit();
		}
	}
	
	toggleCalibProc(left, right, inputKeyAtBegin);

	if(inputKeyAtBegin != '\0')
		inputKey = '\0';
}

void MyVCDCallback::highPass(cv::Mat input, int channel, int r){
	cv::Mat blur;
	cv::blur(input, blur, cv::Size(r, r));
	//GaussianBlur(img, blurImg, Size(30, 30), 0, 0);

	for(int i=0 ; i<input.rows * input.cols * channel ; i++){
		int result = input.data[i] - blur.data[i] + 128;
		if(result < 0)
			result = 0;
		else if(result > 255)
			result = 255;
		input.data[i] = result;
	}
}
void MyVCDCallback::histogramEqualize(cv::Mat input, int channel){
	int totalNode = input.rows * input.cols;

	int histogram[256];
	memset(histogram, 0, sizeof(int) * 256);
	for(int i=0 ; i<totalNode*channel; i+=channel){
		histogram[input.data[i]]++;
	}

	uchar lookup[256];
	for(int i=0 ; i<256 ; i++){
		if(i > 0)
			histogram[i] += histogram[i-1];
		lookup[i] = histogram[i] / (double)totalNode * 255;
	}

	for(int i=0 ; i<totalNode*channel; i++){
		input.data[i] = lookup[ input.data[i] ];
	}
}
void MyVCDCallback::toggleCalibProc(cv::Mat left, cv::Mat right, char inputKeyAtBegin){
	//��C�}�l�����s���ե��v��
		//��S�x�s�s���ե��v��
	//�A��C������� �éI�sstereoCalibAndRectify
	if(inputKeyAtBegin == 'C'){
		if( isCalibrationOn ){
			calib_proc.closeImgListFileStream();
			calib_proc.stereoCalibAndRectify();
			isCalibrationOn = false;
		}else{
			calib_proc.openImgListFileStream();
			isCalibrationOn = true;
			isDispInit      = false;
		}
	}

	if(isCalibrationOn){
		if(inputKeyAtBegin == 'S'){
			//cv::Mat left_b = left.clone();
			//cv::Mat right_b = right.clone();
			//cv::resize(left_b, left, cv::Size(left_b.cols/2, left_b.rows/2));
			//cv::resize(right_b, right, cv::Size(right_b.cols/2, right_b.rows/2));

			calib_proc.saveNewCalibImg(left, right);
			printf("Calibration Proc: save new calibration file number %d\n", calib_proc.calib_frame_number);
		}
	}

	//��T toggle�O�_�n�ϥ�calibration process
	if(inputKeyAtBegin == 'T'){
		calib_proc.valid_calib_param = calib_proc.valid_calib_param ? false : true;
		calib_proc.valid_calib_param ? printf("calib_proc.valid_calib_param: true, is calibrated\n") : printf("calib_proc.valid_calib_param: false, not calibrated\n");
	}
}
void MyVCDCallback::showLeftRight(std::string pre_title, cv::Mat left, cv::Mat right){
	std::stringstream sstm;
	sstm << pre_title << "left";
	cv::namedWindow(sstm.str(), CV_WINDOW_KEEPRATIO);
	cv::imshow(sstm.str(), left);
	cvWaitKey(1);

	sstm.str("");
	sstm << pre_title << "right";
	cv::namedWindow(sstm.str(), CV_WINDOW_KEEPRATIO);
	cv::imshow(sstm.str(), right);
	cvWaitKey(1);
}
void MyVCDCallback::showLeftRightMerged(cv::Mat left, cv::Mat right){
	cv::Mat frame;
	frame.create(left.rows, left.cols + right.cols, left.type());
	cv::Mat frameLeftPart = frame(cv::Rect(0, 0, left.cols, left.rows));
	resize(left, frameLeftPart, frameLeftPart.size(), 0, 0, CV_INTER_AREA);
	cv::Mat frameRightPart = frame(cv::Rect(left.cols, 0, right.cols, right.rows));
	resize(right, frameRightPart, frameRightPart.size(), 0, 0, CV_INTER_AREA);

	int lineAmt = 30;
	int step = frame.rows / (lineAmt+1);
	for( int j = step; j <= step * lineAmt; j += step)
        line(frame, cv::Point(0, j), cv::Point(frame.cols, j), cv::Scalar(0), 2, 8);

	cv::namedWindow("Merged", CV_WINDOW_FREERATIO);
	cv::imshow("Merged", frame);
	cvWaitKey(1);
}

void MyVCDCallback::flipLeftRight(cv::Mat &left, cv::Mat &right){
	cv::Mat fl = left.clone();
	cv::Mat fr = right.clone();
	cv::flip(fl, left, 0);
	cv::flip(fr, right, 0);
}

void showEularAngleOfRotMat(cv::Mat rot_mat){

}

void MyVCDCallback::applyHomography(cv::Mat left, int subsample_divider){
	/*cv::Mat grayLeft = cv::Mat(left.rows, left.cols, CV_8UC1);
	cv::cvtColor(left, grayLeft, CV_RGB2GRAY);
	cv::Mat doubleLeft = cv::Mat(grayLeft.rows, grayLeft.cols, CV_64FC1);
	grayLeft.convertTo(doubleLeft, CV_64FC1);*/
	if(!calib_proc.CamProjHomography.empty()){
		cv::Mat left_b;
		cv::resize(left, left_b, cv::Size(left.rows * subsample_divider, left.cols * subsample_divider));

		cv::Mat result;
		cv::warpPerspective(left_b, result, calib_proc.CamProjHomography, cv::Size(1280, 960), CV_INTER_LINEAR);
		show_cv_img("ApplyHomography", result.data, result.rows, result.cols, 3, false);
	}
}

inline void normalizeHomoCoord_64FC1(cv::Mat &coords){
	double *x, *y, *z, *w;
	
	x = (double *)coords.data;
	y = (double *)&x[coords.cols];
	if(coords.rows == 3){
		w = (double *)&y[coords.cols];

		for(int i=0 ; i<coords.cols ; i++){
			x[i] = x[i] / w[i];
			y[i] = y[i] / w[i];
		}
	}else if(coords.rows == 4){
		z = (double *)&y[coords.cols];
		w = (double *)&z[coords.cols];

		for(int i=0 ; i<coords.cols * coords.rows ; i++){
			x[i] = x[i] / w[i];
			y[i] = y[i] / w[i];
			z[i] = z[i] / w[i];
		}
	}else{
		printf("homoCoordToNormal_64FC1 Invalid rows of cv::Mat: %d\n", coords.rows);
	}
}
void getUndoRectifyLeftCoord(cv::Mat &left_2d_points, CALIB_PROC &calib_proc, CWZDISPTYPE *dmap, int w, int h, int subsample_divider){
	int node_c = w * h;

	double *left_2d_points_data = (double *)left_2d_points.data;
	double *left_2d_points_data_x = &left_2d_points_data[0];
	double *left_2d_points_data_y = &left_2d_points_data[node_c];
	double *left_2d_points_data_z = &left_2d_points_data[node_c*2];
	double *left_2d_points_data_w = &left_2d_points_data[node_c*3];
	int x = 0;
	int y = 0;
	cv::Mat rectified_2d_homo = cv::Mat(3, 1, CV_64FC1);
	cv::Mat R_inverse = calib_proc.R1.inv();
	double *rectified_2d_homo_data = (double *)rectified_2d_homo.data;

	//newCameraMatrix為3x4矩陣
	double new_fx = calib_proc.P1.at<double>(0, 0);
	double new_cx = calib_proc.P1.at<double>(0, 2);
	double new_fy = calib_proc.P1.at<double>(1, 1);
	double new_cy = calib_proc.P1.at<double>(1, 2);
	//CameraMatrix為3x3矩陣
	double fx = calib_proc.cameraMatrix[0].at<double>(0, 0);
	double cx = calib_proc.cameraMatrix[0].at<double>(0, 2);
	double fy = calib_proc.cameraMatrix[0].at<double>(1, 1);
	double cy = calib_proc.cameraMatrix[0].at<double>(1, 2);
	double focal_length = fx;

	double cx_right = calib_proc.cameraMatrix[1].at<double>(0, 2);

	for(int idx = 0 ; idx < node_c ; idx++){
		rectified_2d_homo_data[0] = (x - new_cx) / new_fx;
		rectified_2d_homo_data[1] = (y - new_cy) / new_fy;
		rectified_2d_homo_data[2] = 1;

		cv::Mat real_homo_coord = R_inverse * rectified_2d_homo;
		
		double real_x_coord = real_homo_coord.at<double>(0) / real_homo_coord.at<double>(2);
		double real_y_coord = real_homo_coord.at<double>(1) / real_homo_coord.at<double>(2);

		double real_x = real_x_coord * fx + cx;
		double real_y = real_y_coord * fy + cy;

		left_2d_points_data_x[idx] = real_x    * subsample_divider;
		left_2d_points_data_y[idx] = real_y    * subsample_divider;
		left_2d_points_data_z[idx] = dmap[idx] * subsample_divider;

		x++;
		if(x >= w){
			x = 0;
			y++;
		}
	}
}
void getRectifiedLeftCoord(cv::Mat &left_2d_points, CWZDISPTYPE *dmap, int w, int h, int subsample_divider){
	int node_c = w * h;

	double *left_2d_points_data = (double *)left_2d_points.data;
	double *left_2d_points_data_x = &left_2d_points_data[0];
	double *left_2d_points_data_y = &left_2d_points_data[node_c];
	double *left_2d_points_data_z = &left_2d_points_data[node_c*2];
	double *left_2d_points_data_w = &left_2d_points_data[node_c*3];
	int x = 0;
	int y = 0;

	for(int idx = 0 ; idx < node_c ; idx++){
		left_2d_points_data_x[idx] = x    * subsample_divider;
		left_2d_points_data_y[idx] = y    * subsample_divider;
		left_2d_points_data_z[idx] = dmap[idx] * subsample_divider;

		x++;
		if(x >= w){
			x = 0;
			y++;
		}
	}
}
void initQMatrixNoRectify(cv::Mat &q_mat, CALIB_PROC &calib_proc){
	//CameraMatrix為3x3矩陣
	double fx = calib_proc.cameraMatrix[0].at<double>(0, 0);
	double cx = calib_proc.cameraMatrix[0].at<double>(0, 2);
	double fy = calib_proc.cameraMatrix[0].at<double>(1, 1);
	double cy = calib_proc.cameraMatrix[0].at<double>(1, 2);
	double focal_length = fx;

	double cx_right = calib_proc.cameraMatrix[1].at<double>(0, 2);

	for(int i=0 ; i<q_mat.cols*q_mat.rows ; i++)
		q_mat.at<double>(i) = 0.0;
	q_mat.at<double>(0, 0) = 1.0;
	q_mat.at<double>(0, 3) = -cx;
	q_mat.at<double>(1, 1) = 1.0;
	q_mat.at<double>(1, 3) = -cy;
	q_mat.at<double>(2, 3) = focal_length;
	q_mat.at<double>(3, 2) = -1.0 / calib_proc.T.at<double>(0, 0);
	q_mat.at<double>(3, 3) = (cx - cx_right) / calib_proc.T.at<double>(0, 0);
	std::cout << "Q Matrix" << std::endl;
	std::cout << q_mat << std::endl;
}
void initQMatrixRectified(cv::Mat &q_mat, CALIB_PROC &calib_proc){
	for(int i=0 ; i<q_mat.cols * q_mat.rows ; i++)
		q_mat.at<double>(i) = calib_proc.Q.at<double>(i);
}
void applyQMatrx(cv::Mat left_2d_points, double *left_3d_points_data, cv::Mat Q, int  node_c){
	cv::Mat left_3d_homo_points = Q * left_2d_points;
	double *left_3d_homo_points_data = (double *)left_3d_homo_points.data;
	double *left_3d_homo_points_data_x = &left_3d_homo_points_data[0];
	double *left_3d_homo_points_data_y = &left_3d_homo_points_data[node_c];
	double *left_3d_homo_points_data_z = &left_3d_homo_points_data[node_c*2];
	double *left_3d_homo_points_data_w = &left_3d_homo_points_data[node_c*3];

	double *left_3d_points_data_x      = &left_3d_points_data[0];
	double *left_3d_points_data_y      = &left_3d_points_data[node_c];
	double *left_3d_points_data_z      = &left_3d_points_data[node_c*2];
	for(int idx = 0 ; idx < node_c ; idx++){
		left_3d_points_data_x[idx] = left_3d_homo_points_data_x[idx] / left_3d_homo_points_data_w[idx];
		left_3d_points_data_y[idx] = left_3d_homo_points_data_y[idx] / left_3d_homo_points_data_w[idx];
		left_3d_points_data_z[idx] = left_3d_homo_points_data_z[idx] / left_3d_homo_points_data_w[idx];
	}
}
void apply3DCoordToRTMatrix(cv::Mat left_3d_points, cv::Mat &result_3d_points, cv::Mat R, cv::Mat T, int w, int h, int node_c){
	result_3d_points = R * left_3d_points;

	double *proj_T_data = (double *)T.data;
	double *result_3d_points_x = (double *)result_3d_points.data;
	double *result_3d_points_y = &result_3d_points_x[node_c];
	double *result_3d_points_z = &result_3d_points_y[node_c];
	for(int idx=0 ; idx < node_c ; idx++){
		result_3d_points_x[idx] += proj_T_data[0];
		result_3d_points_y[idx] += proj_T_data[1];
		result_3d_points_z[idx] += proj_T_data[2];
		/*
		int ox = idx % w;
		int oy = idx / w;

		if(ox >= 337 && ox <= 339 && (oy == 211 || oy == 214)){
			printf("X:%d Y:%d = left_3d(%f, %f, %f) proj_3d(%f, %f, %f)\n", ox, oy, 
														left_3d_points_data_x[idx], left_3d_points_data_y[idx], left_3d_points_data_z[idx],
														result_3d_points_x[idx], result_3d_points_y[idx], result_3d_points_z[idx]);
		}
		*/
	}
}
bool MyVCDCallback::convertLeftToProj(cv::Mat left, cv::Mat right, CWZDISPTYPE *dmap, cv::Mat proj, int subsample_divider){
	if( !calib_proc.valid_proj_calib_param )
		return false;

	printf("convertLeftToProj: subsample_divider = %d\n", subsample_divider);
	
	bool GUESS_RIGHT        = false;
	bool UNDO_RECTIFICATION = true;
	std::string title       = "Guessed Projector View";

	cv::Mat q_mat = calib_proc.Q;
	cv::Mat r_mat = calib_proc.proj_R;
	cv::Mat t_mat = calib_proc.proj_T;
	cv::Mat target_camera_mat = calib_proc.proj_K;

	if(GUESS_RIGHT){
		r_mat = calib_proc.R;
		t_mat = calib_proc.T;
		target_camera_mat = cv::Mat(3, 3, CV_64FC1);
		if(UNDO_RECTIFICATION)
			for(int i=0 ; i<3; i++)for(int j=0 ; j<3 ; j++)
				target_camera_mat.at<double>(i,j) = calib_proc.cameraMatrix[1].at<double>(i,j);
		else
			for(int i=0 ; i<3; i++)for(int j=0 ; j<3 ; j++)
				target_camera_mat.at<double>(i,j) = calib_proc.P2.at<double>(i,j);
		title = "Guessed Right Camera View";
	}

	int w = left.cols;
	int h = left.rows;
	int node_c = w * h;
	
	if(!is_left_2d_points_init)
	{
		left_2d_points = cv::Mat(4, node_c, CV_64FC1);
		left_2d_points_data = (double *)left_2d_points.data;
		is_left_2d_points_init = true;

		double *left_2d_points_data_w = &left_2d_points_data[node_c*3];
		for(int idx = 0 ; idx < node_c ; idx++)
			left_2d_points_data_w[idx] = 1;
		//
		left_3d_points = cv::Mat(3, node_c, CV_64FC1);
		left_3d_points_data = (double *) left_3d_points.data;
	}

	if(UNDO_RECTIFICATION){
		initQMatrixNoRectify(q_mat, calib_proc);
		getUndoRectifyLeftCoord(left_2d_points, calib_proc, dmap, w, h, subsample_divider);
	}else{
		getRectifiedLeftCoord(left_2d_points, dmap, w, h, subsample_divider);
	}

	applyQMatrx(left_2d_points, left_3d_points_data, q_mat, node_c);
	cv::Mat result_3d_points;
	apply3DCoordToRTMatrix(left_3d_points, result_3d_points, r_mat, t_mat, w, h, node_c);
	//reulst_2d_points = calib_proc.proj_K * result_3d_points;
	cv::Mat reulst_2d_points = target_camera_mat * result_3d_points;

	int x_shift = 40;
	int y_shift = 61;

	normalizeHomoCoord_64FC1(reulst_2d_points);

	double *reulst_2d_points_data_x = (double *) reulst_2d_points.data;
	double *reulst_2d_points_data_y = &reulst_2d_points_data_x[node_c];

	uchar *proj_data = proj.data;
	uchar *left_data = left.data;
	for(int i = 0 ; i < node_c ; i++){
		//if(ox < 300 || ox > 480 || oy < 160 || oy > 360)
		//	continue;
		if(dmap[i] <= 50)
			continue;

		double _x = (reulst_2d_points_data_x[i]) / subsample_divider;
		double _y = (reulst_2d_points_data_y[i]) / subsample_divider;

		uchar origin_pixel = left_data[i*3];

		int __x = floor(_x) + x_shift;
		int __y = floor(_y) + y_shift;
		int idx = abs(__y * w + __x);
		if(idx > 0 && idx+w+1 < node_c){
			proj_data[idx]    = origin_pixel;
			proj_data[idx+1]  = origin_pixel;
			proj_data[idx+w]  = origin_pixel;
			proj_data[idx+w+1] = origin_pixel;
		}
	}
	
	cv::Mat proj_tmp = proj.clone();
	cv::resize(proj_tmp, proj, cv::Size(proj.cols*subsample_divider, proj.rows*subsample_divider));
	cv::Size proj_size = cv::Size(1024, 768);

	if(GUESS_RIGHT){
		proj_size = cv::Size(right.cols * subsample_divider, right.rows * subsample_divider);
	}

	proj = proj(cv::Rect(0, 0, proj_size.width, proj_size.height));
	cv::namedWindow(title, CV_WINDOW_FREERATIO);
	cv::imshow(title, proj);
	cv::waitKey(10);
	//show_cv_img("proj.data", proj.data, proj.rows, proj.cols, 1, true);
	show_cv_img_fullscreen("FullScreenProjection", proj, false);

	return true;
}