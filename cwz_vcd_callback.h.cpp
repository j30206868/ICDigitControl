#include "cwz_vcd_callback.h"

void keyLog(char *inputKey, bool *running){
	//詳細的Virtual Key Code對照表請見https://msdn.microsoft.com/en-us/library/windows/desktop/dd375731(v=vs.85).aspx
	//Virtual Key Code沒有大小寫之分, 只是physically mapping到鍵盤
	//也可以控制滑鼠, 不過滑鼠的左右鍵是physically mapping, 可能會顛倒
	char chType[]="ABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890";
	while(*running)
	{
		if(*inputKey == '\0')
			for (int i=0; i<36; i++){
				if( GetAsyncKeyState(chType[i]) ){
					*inputKey = chType[i];
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
	keyLogger = std::thread(keyLog, &this->inputKey, &this->runningLogger);

	//disparity
	isDispInit = false;
	device = setup_opencl(context, err);
	program = load_program(context, "clkernel/test.cl");
	if(program == 0) { std::cerr << "Can't load or build program\n"; clReleaseContext(context); system("PAUSE"); }
}
MyVCDCallback::~MyVCDCallback(){
	runningLogger = false;
	keyLogger.join();
}

void MyVCDCallback::imgProc(cv::Mat left, cv::Mat right){
	//calib_proc.remapAndDrawRoi(left, right);
	//calib_proc.remapping(left, right);
	//calib_proc.remapAndCutRoi(left, right);

	//showLeftRightMerged(left, right);

	showLeftRight(left, right);

	//printf("img cut dimension(%d, %d)\n", left.cols, left.rows);
	/*if(calib_proc.valid_calib_param){
		if(!isDispInit){
			
			for(subsample_divider = 1 ;; subsample_divider*=2){
				if(left.type() == CV_8UC3)
					info = createMatchInfo(left.cols/subsample_divider, left.rows/subsample_divider, 3);
				else
					info = createMatchInfo(left.cols/subsample_divider, left.rows/subsample_divider, 1);

				if(info->max_x_d < 255)
					break;
				else
					delete info;
			}
			info->printf_match_info("Subsample Info");
			printf("subsample_divider: %d\n", subsample_divider);

			cv::Mat left_b = left.clone();
			cv::Mat right_b = right.clone();
			cv::resize(left_b, left, cv::Size(left_b.cols/subsample_divider, left_b.rows/subsample_divider));
			cv::resize(right_b, right, cv::Size(right_b.cols/subsample_divider, right_b.rows/subsample_divider));
			printf("resize left right dimension(%d, %d)\n", left.cols, left.rows);

			dmap_generator.init(context, device, program, err, left, right, *info);
			dmap_ref.init(dmap_generator.mst_L, *info, dmap_generator.left_dmap, dmap_generator.right_dmap);

			isDispInit = true;
			printf("MyVCDCallback: disp init finished");
		}

		if(subsample_divider > 1){
			cv::Mat left_b = left.clone();
			cv::Mat right_b = right.clone();
			cv::resize(left_b, left, cv::Size(left_b.cols/subsample_divider, left_b.rows/subsample_divider));
			cv::resize(right_b, right, cv::Size(right_b.cols/subsample_divider, right_b.rows/subsample_divider));
		}

		

		if(inputKey == 'D'){
			//cv::Mat blurLeft  = left.clone();
			//cv::Mat blurRight = right.clone();
			//cv::blur(blurLeft, left, cv::Size(5, 5));
			//highPass(left, 3, 60);
			//histogramEqualize(left, 3);
			//cv::blur(blurRight, right, cv::Size(5, 5));
			//highPass(right, 3, 60);
			//histogramEqualize(right, 3);

			dmap_generator.set_left_right(left, right);

			dmap_generator.filtering();
			dmap_generator.compute_cwz_img();

			uchar *left_dmap;
			uchar *right_dmap;
			uchar *refined_dmap;

			if( !(left_dmap = dmap_generator.generate_left_dmap()) )
			{printf( "cwz_dmap_generate left_dmap failed...!" );system("PAUSE");}

			if( !(right_dmap = dmap_generator.generate_right_dmap()) )
			{printf( "cwz_dmap_generate right_dmap failed...!" );system("PAUSE");}

			cwz_mst::updateSigma(cwz_mst::sigma * 4);
			refined_dmap = dmap_ref.refinement(dmap_refine::MODE_TREE);
			cwz_mst::updateSigma(cwz_mst::sigma / 4);

			//把黑色之外地方的深度全部歸零
			//for(int i=0 ; i<info->node_c ; i++){
			//	if(dmap_generator.left_gray_1d_arr[i] >= 170){
			//		refined_dmap[i] = 0;
			//	}
			//}

			if(true)
				show_cv_img("left_dmap", left_dmap, info->img_height, info->img_width, 1, false);
			if(true)
				show_cv_img("right_dmap", right_dmap, info->img_height, info->img_width, 1, false);
			//顯示深度影像 並在window標題加上frame_count編號
			show_cv_img(0, "深度影像", refined_dmap, info->img_height, info->img_width, 1, true);

			dmap_generator.mst_L.reinit();
			dmap_generator.mst_R.reinit();
		}
	}*/
	
	toggleCalibProc(left, right);

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
void MyVCDCallback::toggleCalibProc(cv::Mat left, cv::Mat right){
	//按C開始收集新的校正影像
		//按S儲存新的校正影像
	//再按C結束收集 並呼叫stereoCalibAndRectify
	if(inputKey == 'C'){
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
		if(inputKey == 'S'){
			calib_proc.saveNewCalibImg(left, right);
			printf("Calibration Proc: save new calibration file number %d\n", calib_proc.calib_frame_number);
		}
	}

	//按T toggle是否要使用calibration process
	if(inputKey == 'T'){
		calib_proc.valid_calib_param = calib_proc.valid_calib_param ? false : true;
		calib_proc.valid_calib_param ? printf("calib_proc.valid_calib_param: true, is calibrated\n") : printf("calib_proc.valid_calib_param: false, not calibrated\n");
	}
}
void MyVCDCallback::showLeftRight(cv::Mat left, cv::Mat right){
	cv::namedWindow("left", CV_WINDOW_KEEPRATIO);
	cv::imshow("left", left);
	cvWaitKey(1);
	cv::namedWindow("right", CV_WINDOW_KEEPRATIO);
	cv::imshow("right", right);
	cvWaitKey(1);

}
void MyVCDCallback::showLeftRightMerged(cv::Mat left, cv::Mat right){
	cv::Mat frame;
	frame.create(left.rows, left.cols + right.cols, left.type());
	cv::Mat frameLeftPart = frame(cv::Rect(0, 0, left.cols, left.rows));
	resize(left, frameLeftPart, frameLeftPart.size(), 0, 0, CV_INTER_AREA);
	cv::Mat frameRightPart = frame(cv::Rect(left.cols, 0, right.cols, right.rows));
	resize(right, frameRightPart, frameRightPart.size(), 0, 0, CV_INTER_AREA);

	int lineAmt = 9;
	int step = frame.rows / (lineAmt+1);
	for( int j = step; j <= step * lineAmt; j += step)
        line(frame, cv::Point(0, j), cv::Point(frame.cols, j), cv::Scalar(0), 2, 8);

	cv::namedWindow("Combined", CV_WINDOW_FREERATIO);
	cv::imshow("Combined", frame);
	cvWaitKey(1);
}