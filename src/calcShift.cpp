#include <opencv2/opencv.hpp>
#include <iostream>
#include <cassert>
#include <cmath>

/**
* @brief オプティカルフローを計算することで動画の動きを検出します。
* @param [in] filename 動画のファイル名
* @param [in] calcPeriod オプティカルフローを計算するフレーム数[ ]で正の整数。0未満を指定すると全域で相関を計算。
**/
std::vector<cv::Vec3d> CalcShiftFromVideo(char *filename, int calcPeriod){
	//動画を開く
	cv::VideoCapture cap(filename);
    assert(cap.isOpened());

    cv::Mat cur, cur_grey;
    cv::Mat prev, prev_grey;

	//リサイズor切り抜き関係
	const int processWidth = 640;	//処理時の画像サイズ
	double mag = 1.0;				//処理画像倍率
	mag = processWidth/cap.get(CV_CAP_PROP_FRAME_WIDTH);
	printf("Mag:%f\n",mag);
	cv::Mat rimg;

	//動画から最初のフレームを取得
    cap >> prev;

	//リサイズ関係
#if 0	//リサイズするとき
	cv::resize(prev,rimg,cv::Size(),mag ,mag ,cv::INTER_NEAREST);
	prev = rimg;
#else  //中心部だけ切り抜くとき
	prev = cv::Mat(prev,cv::Rect((prev.cols-640)/2,(prev.rows-480)/2,640,480));
#endif
	
	//カラーからモノクロへ色変換
    cv::cvtColor(prev, prev_grey, cv::COLOR_BGR2GRAY);
    
    // Step 1 - Get previous to current frame transformation (dx, dy, da) for all frames
    std::vector <cv::Vec3d> prev_to_cur_transform; // previous to current
    int k=1;
    int max_frames = cap.get(CV_CAP_PROP_FRAME_COUNT);
    cv::Mat last_T;
	
	//繰り返し処理	
	while(true) {
        cap >> cur;

        if(cur.data == NULL) {
            break;
        }

		//リサイズ
#if 0	//リサイズするとき
		cv::resize(cur,rimg,cv::Size(),mag ,mag ,cv::INTER_NEAREST));
		cur = rimg;
#else  //中心部だけ切り抜くとき
		cur = cv::Mat(cur,cv::Rect((cur.cols-640)/2,(cur.rows-480)/2,640,480));
#endif


        cv::cvtColor(cur, cur_grey, cv::COLOR_BGR2GRAY);

        // vector from prev to cur
        std::vector <cv::Point2f> prev_corner, cur_corner;
        std::vector <cv::Point2f> prev_corner2, cur_corner2;
        std::vector <uchar> status;
        std::vector <float> err;

        cv::goodFeaturesToTrack(prev_grey, prev_corner, 200, 0.01, 30);
        cv::calcOpticalFlowPyrLK(prev_grey, cur_grey, prev_corner, cur_corner, status, err);

        // weed out bad matches
        for(size_t i=0; i < status.size(); i++) {
            if(status[i]) {
                prev_corner2.push_back(prev_corner[i]);
                cur_corner2.push_back(cur_corner[i]);
            }
        }

        // translation + rotation only
        cv::Mat T = cv::estimateRigidTransform(prev_corner2, cur_corner2, false); // false = rigid transform, no scaling/shearing

        // in rare cases no transform is found. We'll just use the last known good transform.
        if(T.data == NULL) {
            last_T.copyTo(T);
        }

        T.copyTo(last_T);

        // decompose T
        double dx = T.at<double>(0,2)/mag;
        double dy = T.at<double>(1,2)/mag;
        double da = atan2(T.at<double>(1,0), T.at<double>(0,0));

        prev_to_cur_transform.push_back(cv::Vec3d(dx, dy, da));

        //~ out_transform << k << " " << dx << " " << dy << " " << da << endl;

        cur.copyTo(prev);
        cur_grey.copyTo(prev_grey);

        std::cout << "Frame: " << k << "/" << max_frames << " - good optical flow: " << prev_corner2.size() << std::endl;
        k++;
        
        if(calcPeriod <= 0){continue;}	//長さ0ならビデオ全域で継続
        else if(calcPeriod < k){break;}	//指定された長さで処理を終了
    }
	return prev_to_cur_transform;
}
