#include <opencv2/opencv.hpp>
#include <iostream>
#include <cassert>
#include <cmath>
#include <Eigen/Dense>
#include <memory>
/**
* @brief オプティカルフローを計算することで動画の動きを検出します。
* @param [in] filename 動画のファイル名
* @param [in] calcPeriod オプティカルフローを計算するフレーム数[ ]で正の整数。0未満を指定すると全域で相関を計算。
**/
std::vector<cv::Vec3d> CalcShiftFromVideo(const char *filename, int calcPeriod){
    //動画を開く
    cv::VideoCapture cap(filename);
    assert(cap.isOpened());

    cv::Mat cur, cur_grey;
    cv::Mat prev, prev_grey;

    //動画から最初のフレームを取得
    cap >> prev;

    //中心部を取り出してモノクロへ色変換
    int width  = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    if((width >= 640) && (height >= 480))
    {
        cv::cvtColor(prev(cv::Rect((prev.cols-640)/2,(prev.rows-480)/2,640,480)), prev_grey, cv::COLOR_BGR2GRAY);
    }else
    {
        cv::cvtColor(prev, prev_grey, cv::COLOR_BGR2GRAY);
    }
    
    // Step 1 - Get previous to current frame transformation (dx, dy, da) for all frames
    std::vector <cv::Vec3d> prev_to_cur_transform; // previous to current
    int k=1;
    int max_frames = cap.get(cv::CAP_PROP_FRAME_COUNT);
    cv::Mat last_T;

    //繰り返し処理
    while(true) {
        cap >> cur;

        if(cur.data == NULL) {
            break;
        }

        //中心部だけ切り抜く
        // cur = cv::Mat(cur,cv::Rect((cur.cols-640)/2,(cur.rows-480)/2,640,480));
        // cv::cvtColor(cur, cur_grey, cv::COLOR_BGR2GRAY);
        if((width >= 640) && (height >= 480))
        {
            cv::cvtColor(cur(cv::Rect((cur.cols-640)/2,(cur.rows-480)/2,640,480)), cur_grey, cv::COLOR_BGR2GRAY);
        }else
        {
            cv::cvtColor(cur, cur_grey, cv::COLOR_BGR2GRAY);
        }


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

        double dx = T.at<double>(0,2);
        double dy = T.at<double>(1,2);
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

void CalcShiftFromVideo(const char *filename, int calcPeriod, Eigen::MatrixXd &optical_shift, Eigen::MatrixXd &confidence){
    //動画を開く
    cv::VideoCapture cap(filename);
    assert(cap.isOpened());

    optical_shift = Eigen::MatrixXd::Zero(cap.get(cv::CAP_PROP_FRAME_COUNT),3);
    confidence = Eigen::MatrixXd::Zero(cap.get(cv::CAP_PROP_FRAME_COUNT),1);

    cv::Mat cur, cur_grey;
    cv::Mat prev, prev_grey;

    //動画から最初のフレームを取得
    cap >> prev;

    //中心部を取り出してモノクロへ色変換
    int width  = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    if((width >= 640) && (height >= 480))
    {
        cv::cvtColor(prev(cv::Rect((prev.cols-640)/2,(prev.rows-480)/2,640,480)), prev_grey, cv::COLOR_BGR2GRAY);
    }else
    {
        cv::cvtColor(prev, prev_grey, cv::COLOR_BGR2GRAY);
    }
    
    // Step 1 - Get previous to current frame transformation (dx, dy, da) for all frames
    std::vector <cv::Vec3d> prev_to_cur_transform; // previous to current
    int k=1;
    int max_frames = cap.get(cv::CAP_PROP_FRAME_COUNT);
    cv::Mat last_T;

    //繰り返し処理
    for(int frame=0;frame<max_frames;++frame) {
        cap >> cur;

        if(cur.data == NULL) {
            break;
        }

        //中心部だけ切り抜く
        // cur = cv::Mat(cur,cv::Rect((cur.cols-640)/2,(cur.rows-480)/2,640,480));
        // cv::cvtColor(cur, cur_grey, cv::COLOR_BGR2GRAY);
        if((width >= 640) && (height >= 480))
        {
            cv::cvtColor(cur(cv::Rect((cur.cols-640)/2,(cur.rows-480)/2,640,480)), cur_grey, cv::COLOR_BGR2GRAY);
        }else
        {
            cv::cvtColor(cur, cur_grey, cv::COLOR_BGR2GRAY);
        }


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
        try{
            cv::Mat T = cv::estimateRigidTransform(prev_corner2, cur_corner2, false); // false = rigid transform, no scaling/shearing

            // in rare cases no transform is found. We'll just use the last known good transform.
            if(T.data == NULL) {
                last_T.copyTo(T);
            }

            T.copyTo(last_T);

            double dx = T.at<double>(0,2);
            double dy = T.at<double>(1,2);
            double da = atan2(T.at<double>(1,0), T.at<double>(0,0));
            // prev_to_cur_transform.push_back(cv::Vec3d(dx, dy, da));
            optical_shift.row(frame) << dx, dy, da;
            confidence.row(frame) << 1.0;

        }catch(...){
            optical_shift.row(frame) << 0.0, 0.0, 0.0;
            confidence.row(frame) << 0.0;
        }
        //~ out_transform << k << " " << dx << " " << dy << " " << da << endl;

        cur.copyTo(prev);
        cur_grey.copyTo(prev_grey);

        std::cout << "Frame: " << k << "/" << max_frames << " - good optical flow: " << prev_corner2.size() << std::endl;
        k++;
        
        if(calcPeriod <= 0){continue;}	//長さ0ならビデオ全域で継続
        else if(calcPeriod < k){break;}	//指定された長さで処理を終了
    }
    // return prev_to_cur_transform;
    return;
}

/**
* @brief オプティカルフローを計算することで動画の動きを検出します。
* @param [in] filename 動画のファイル名
* @param [in] calcPeriod オプティカルフローを計算するフレーム数[ ]で正の整数。0未満を指定すると全域で相関を計算。
**/
// void calcShiftFromVideo(std::shared_ptr<cv::VideoCapture> capture, int calc_length, Eigen::MatrixXd &dst){
//     assert(capture->isOpened());
//     calc_length = capture->get(cv::CAP_PROP_FRAME_COUNT) > calc_length ? calc_length : capture->get(cv::CAP_PROP_FRAME_COUNT);
//     dst.resize(calc_length,3);

//     cv::Mat cur, cur_grey;
//     cv::Mat prev, prev_grey;

//     //動画から最初のフレームを取得
//     *capture >> prev;

//     //中心部を取り出してモノクロへ色変換
//     cv::cvtColor(prev(cv::Rect((prev.cols-640)/2,(prev.rows-480)/2,640,480)), prev_grey, cv::COLOR_BGR2GRAY);

//     // Step 1 - Get previous to current frame transformation (dx, dy, da) for all frames
// //    std::vector <cv::Vec3d> prev_to_cur_transform; // previous to current

//     int max_frames = capture->get(cv::CAP_PROP_FRAME_COUNT);
//     cv::Mat last_T;

//     //繰り返し処理
//     for(int frame=0;frame<calc_length;++frame) {
//         *capture >> cur;

//         if(cur.data == NULL) {
//             break;
//         }

//         //中心部だけ切り抜く
//         cur = cv::Mat(cur,cv::Rect((cur.cols-640)/2,(cur.rows-480)/2,640,480));
//         cv::cvtColor(cur, cur_grey, cv::COLOR_BGR2GRAY);

//         // vector from prev to cur
//         std::vector <cv::Point2f> prev_corner, cur_corner;
//         std::vector <cv::Point2f> prev_corner2, cur_corner2;
//         std::vector <uchar> status;
//         std::vector <float> err;

//         cv::goodFeaturesToTrack(prev_grey, prev_corner, 200, 0.01, 30);
//         cv::calcOpticalFlowPyrLK(prev_grey, cur_grey, prev_corner, cur_corner, status, err);

//         // weed out bad matches
//         for(size_t i=0; i < status.size(); i++) {
//             if(status[i]) {
//                 prev_corner2.push_back(prev_corner[i]);
//                 cur_corner2.push_back(cur_corner[i]);
//             }
//         }

//         // translation + rotation only
//         cv::Mat T = cv::estimateRigidTransform(prev_corner2, cur_corner2, false); // false = rigid transform, no scaling/shearing

//         // in rare cases no transform is found. We'll just use the last known good transform.
//         if(T.data == NULL) {
//             last_T.copyTo(T);
//         }

//         T.copyTo(last_T);

//         double dx = T.at<double>(0,2);
//         double dy = T.at<double>(1,2);
//         double da = atan2(T.at<double>(1,0), T.at<double>(0,0));

// //        prev_to_cur_transform.push_back(cv::Vec3d(dx, dy, da));
//         dst(frame,0) = dx;
//         dst(frame,1) = dy;
//         dst(frame,2) = da;
//         //~ out_transform << k << " " << dx << " " << dy << " " << da << endl;

//         cur.copyTo(prev);
//         cur_grey.copyTo(prev_grey);

//         std::cout << "Frame: " << frame << "/" << max_frames << " - good optical flow: " << prev_corner2.size() << std::endl;


//     }
// //    return prev_to_cur_transform;
//     return;
// }
