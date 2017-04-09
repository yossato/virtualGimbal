#include <iostream>

#include "Eigen/Dense"
#include "unsupported/Eigen/NonLinearOptimization"
#include "unsupported/Eigen/NumericalDiff"
#include <boost/math/quaternion.hpp> //四元数使う
#include <opencv2/opencv.hpp>
#include "calcShift.hpp"
using namespace Eigen;
using namespace boost::math;

template <typename _Tp> extern double SSD_Quaternion(double ti, double ts, std::vector<cv::Mat> &vecFrames, std::vector<quaternion<_Tp>> &Qa, double T, int Len, int cLen, cv::Mat IK, cv::Mat matIntrinsic);
template <typename _Tp> extern quaternion<_Tp> Slerp(quaternion<_Tp> Qfrom, quaternion<_Tp> Qto, _Tp t);
template <typename T_num> extern cv::Vec3d Quaternion2Vector(quaternion<T_num> q);
template <typename T_num> extern cv::Vec3d Quaternion2Vector(quaternion<T_num> q, cv::Vec3d prev);
template <typename T_num> extern void Quaternion2Matrix(quaternion<T_num> q, cv::Mat &det);
// Generic functor
template<typename _Scalar, int NX=Dynamic, int NY=Dynamic>
struct Functor
{
  typedef _Scalar Scalar;
  enum {
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
  };
  typedef Matrix<Scalar,InputsAtCompileTime,1> InputType;
  typedef Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
  typedef Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;
  
  Functor() : inputs_(InputsAtCompileTime), values_(ValuesAtCompileTime){}
  Functor(int inputs, int values) : inputs_(inputs), values_(values) {}
  
  int inputs() const { return inputs_; }
  int values() const { return values_; }
  
  const int inputs_;
  const int values_;

  
};

/**
 * @brief クォータニオンによる角度と、動画のフレームから、ビデオとジャイロセンサのタイミングを同期します。
 * @brief 動画とジャイロのタイミングを、フレーム間の輝度値の差の二乗和を最小化することで計算します。
 * @param [in]	ti	時間差の初期値、ビデオにおける秒[s]
 * @param [in]	ts	ローリングシャッターの全行取得時間[sec]
 //~ * @param [in]	Cap	ビデオのcapture[]
 * @param [in]	Qa	ジャイロの角速度から計算したカメラの方向を表す回転クォータニオン時系列データ
 * @param [in] 	m_strTI	ジャイロとビデオのサンプリング周期[s]を記録した構造体
 * @param [in]	cLen 相関を取る前後フレームの数[frame]
 * @param [in]	IK	"逆"歪係数(k1,k2,p1,p2)
 * @param [in]	matIntrinsic	カメラ行列(fx,fy,cx,cy) [pixel]
 * @param [in]	inputs	最適化したい変数の数[]
 * @param [in]	values	誤差式の数 = 自乗誤差を求めるフレームの長さ
 * @retval				タイミング[s]
 **/
template <typename _Tp> struct calc_timing_from_image : Functor<double>
{
	calc_timing_from_image(int inputs, int values, std::vector<cv::Mat> &m_vecFrames, std::vector<quaternion<_Tp>> &m_Qa, strTimingInformation &m_strTI, /*int Len,*/ int m_cLen, cv::Mat m_IK, cv::Mat m_matIntrinsic)
	//初期化はここに書く
	: Qa(m_Qa), vecFrames(m_vecFrames), strTI(m_strTI),  cLen(m_cLen), IK(m_IK), matIntrinsic(m_matIntrinsic), Functor(inputs, values) {}
	
	std::vector<cv::Mat> &vecFrames;
	//~ cv::VideoCapture &m_Cap;
	std::vector<quaternion<_Tp>> &Qa;
	strTimingInformation &strTI;
	//~ int		m_Len;
	int		cLen;
	cv::Mat IK;
	cv::Mat matIntrinsic;
	
	
	int operator()(const VectorXd& times, VectorXd& fvec) const	//times[0]がti、times[1]がtsである。
	{
		assert(IK.type() == CV_64F);
		assert(matIntrinsic.type() == CV_64F);
		
		//~ double scale = 160.0/1920.0;	//任意のスケール。小さいほうが処理が早い
		//~ matIntrinsic = scale * matIntrinsic;
		double ti = times[0];
		double ts = times[1];
		double fx = matIntrinsic.at<double>(0, 0);
		double fy = matIntrinsic.at<double>(1, 1);
		double cx = matIntrinsic.at<double>(0, 2);
		double cy = matIntrinsic.at<double>(1, 2);
		double k1 = IK.at<double>(0,0);
		double k2 = IK.at<double>(0,1);
		double p1 = IK.at<double>(0,2);
		double p2 = IK.at<double>(0,3);
		
		cv::namedWindow("TEST",cv::WINDOW_NORMAL);
		cv::namedWindow("TEST2",cv::WINDOW_NORMAL);
		cv::namedWindow("DIFF",cv::WINDOW_NORMAL);

		//2.for文でひたすら輝度の差の二乗和を計算→LM法へ後で適用
		//~ std::vector<double> vecSumSq;
		cv::Mat pixelBuff(vecFrames[0].size(),CV_8UC3);
		cv::Mat pixelBuff2(vecFrames[0].size(),CV_8UC3);
		for(int i=0;i<values_;++i){
			std::vector<double> vecSumSq;
			//比較するためにフレームiにおける逆歪補正画像を生成
			for(int v=0;v<vecFrames[0].rows;++v){
				//W(t1,t2)を計算
				cv::Mat R;
				double tiy = ti + strTI.TV*i*cLen + ts*v/vecFrames[0].rows;	//ローリングシャッターの読み込みを考慮した各行毎のサンプル時間[sec]
				double tin = ti + strTI.TV*i*cLen;
				
				//値域制限
				if(tiy < 0.0){
					tiy = 0.0;
					printf("limit:1\r\n");
				 }
				if(strTI.TG*(Qa.size()-1) < tiy){
					tiy = strTI.TG*(Qa.size()-1);
					printf("limit:2\r\n");
				}
				if(tin < 0.0){
					tin = 0.0;
					printf("limit:3\r\n");
				 }
				if(strTI.TG*(Qa.size()-1) < tin){
					tiy = strTI.TG*(Qa.size()-1);
					printf("limit:4\r\n");
				}
				
				unsigned int fi = (int)floor(tiy/strTI.TG);	//ローリングシャッター補正を含むフレーム数の整数部[ ]
				double ff = tiy/strTI.TG - (double)fi;			//ローリングシャッター補正を含むフレーム数の浮動小数点数部[ ]
				auto SQa = Slerp(Qa[fi],Qa[fi+1],ff);	//オリジナルの角度クウォータニオンに関して球面線形補間
				
				unsigned int gi = (int)floor(tin/strTI.TG);		//フレーム数の整数部[ ]
				double gf = tin/strTI.TG - (double)gi;			//フレーム数の浮動小数点数部[ ]
				auto SQf = Slerp(Qa[gi],Qa[gi+1],gf);	//フィルタ済みの角度クウォータニオンに関して球面線形補間
				
				Quaternion2Matrix(conj(SQf)*SQa,R);		//ローリングシャッター補正を含む回転行列を計算

				for(int u=0;u<vecFrames[0].cols;++u){
					//~ double u = (double)i/n*imgSize.width;
					//後々の行列演算に備えて、画像上の座標を同次座標で表現しておく。(x座標、y座標,1)T
					cv::Mat p = (cv::Mat_<double>(3,1) << (u- cx)/fx, (v - cy)/fy, 1.0);	//1のポリゴン座標に、K^-1を掛けた結果の３x１行列
					//2
					cv::Mat XYW = R * p;//inv()なし
					
					double x1 = XYW.at<double>(0, 0)/XYW.at<double>(2, 0);
					double y1 = XYW.at<double>(1, 0)/XYW.at<double>(2, 0);
					
					double r = sqrt(x1*x1+y1*y1);
					
					double x2 = x1*(1.0+k1*r*r+k2*r*r*r*r)+2.0*p1*x1*y1+p2*(r*r+2.0*x1*x1);
					double y2 = y1*(1.0+k1*r*r+k2*r*r*r*r)+p1*(r*r+2.0*y1*y1)+2.0*p2*x1*y1;
					double mapx = x2*fx+cx;
					double mapy = y2*fy+cy;
					
					//結果をmapに保存
					//~ map.at<cv::Vec2d>(j,i)[0] = mapx;
					//~ map.at<cv::Vec2d>(j,i)[1] = mapy;
					int imapx = round(mapx);
					int imapy = round(mapy);
					//もしも画像の範囲内にあれば
					if((0<=imapx)&&(imapx<vecFrames[0].cols)&&(0<=imapy)&&(imapy<vecFrames[0].rows)){
						pixelBuff.at<cv::Vec3b>(imapy,imapx) = vecFrames[i*cLen].at<cv::Vec3b>(v,u);
					}
				}
			}
			
			for(int j=i*cLen+1;j<(i*cLen+cLen);++j){
				//比較するためにフレームiにおける逆歪補正画像を生成
				for(int v=0;v<vecFrames[0].rows;++v){
					//W(t1,t2)を計算
					cv::Mat R;
					double tiy = ti + strTI.TV*j + ts*v/vecFrames[0].rows;	//ローリングシャッターの読み込みを考慮した各行毎のサンプル時間[sec]
					//~ double tin = ti + T*j;
					double tin = ti + strTI.TV*i*cLen;//jではなく、iである。
					
					unsigned int fi = (int)floor(tiy/strTI.TG);	//ローリングシャッター補正を含むフレーム数の整数部[ ]
					double ff = tiy/strTI.TG - (double)fi;			//ローリングシャッター補正を含むフレーム数の浮動小数点数部[ ]
					auto SQa = Slerp(Qa[fi],Qa[fi+1],ff);	//オリジナルの角度クウォータニオンに関して球面線形補間
					
					unsigned int gi = (int)floor(tin/strTI.TG);		//フレーム数の整数部[ ]
					double gf = tin/strTI.TG - (double)gi;			//フレーム数の浮動小数点数部[ ]
					auto SQf = Slerp(Qa[gi],Qa[gi+1],gf);	//フィルタ済みの角度クウォータニオンに関して球面線形補間
					
					Quaternion2Matrix(conj(SQf)*SQa,R);		//ローリングシャッター補正を含む回転行列を計算

					for(int u=0;u<vecFrames[0].cols;++u){
						//~ double u = (double)i/n*imgSize.width;
						//後々の行列演算に備えて、画像上の座標を同次座標で表現しておく。(x座標、y座標,1)T
						cv::Mat p = (cv::Mat_<double>(3,1) << (u- cx)/fx, (v - cy)/fy, 1.0);	//1のポリゴン座標に、K^-1を掛けた結果の３x１行列
						//2
						cv::Mat XYW = R * p;//inv()なし
						
						double x1 = XYW.at<double>(0, 0)/XYW.at<double>(2, 0);
						double y1 = XYW.at<double>(1, 0)/XYW.at<double>(2, 0);
						
						double r = sqrt(x1*x1+y1*y1);
						
						double x2 = x1*(1.0+k1*r*r+k2*r*r*r*r)+2.0*p1*x1*y1+p2*(r*r+2.0*x1*x1);
						double y2 = y1*(1.0+k1*r*r+k2*r*r*r*r)+p1*(r*r+2.0*y1*y1)+2.0*p2*x1*y1;
						double mapx = x2*fx+cx;
						double mapy = y2*fy+cy;
						
						//結果をmapに保存
						//~ map.at<cv::Vec2d>(j,i)[0] = mapx;
						//~ map.at<cv::Vec2d>(j,i)[1] = mapy;
						int imapx = round(mapx);
						int imapy = round(mapy);
						//もしも画像の範囲内にあれば
						if((0<=imapx)&&(imapx<vecFrames[0].cols)&&(0<=imapy)&&(imapy<vecFrames[0].rows)){
							pixelBuff2.at<cv::Vec3b>(imapy,imapx) = vecFrames[j].at<cv::Vec3b>(v,u);//ここもiではなくjである。
						}
					}
				}
				cv::imshow("TEST",pixelBuff);
				cv::imshow("TEST2",pixelBuff2);
				
				cv::Mat Diff = cv::Mat(pixelBuff.size(),CV_8UC3,cv::Scalar(128,128,128)) + 0.5*pixelBuff - 0.5*pixelBuff2;
				cv::imshow("DIFF",Diff);
				
				cv::waitKey(1);
				
				//自乗和を計算
				unsigned long long sumSq = 0;
				unsigned int pixelNum = 0;
				auto src = pixelBuff.begin<cv::Vec3b>();
				auto dst = pixelBuff2.begin<cv::Vec3b>();
				auto v3z = cv::Vec3b(0,0,0);
				for(auto end = pixelBuff.end<cv::Vec3b>();src!=end;++src,++dst){
					if(((*src) != v3z)&&((*dst) != v3z)){//双方の画素値が0でなければ計算
						int nm = norm(*src - *dst);
						sumSq += nm * nm;
						++pixelNum;
						
					}else{
						continue;
					}
				}//各画素に関するループ
				vecSumSq.push_back(sqrt((double)sumSq)/(double)pixelNum);
				
				pixelBuff2 = cv::Mat::zeros(vecFrames[0].size(),CV_8UC3);
				
			}//cLenのループ
			pixelBuff = cv::Mat::zeros(vecFrames[0].size(),CV_8UC3);
			
			fvec[i] = std::accumulate(vecSumSq.begin(),vecSumSq.end(),0.0)/vecSumSq.size();
		}//values_のループ
		//~ double retval = std::accumulate(vecSumSq.begin(),vecSumSq.end(),0.0)/vecSumSq.size();
		double retval = 0.0;
		for(int i=0;i<values_;++i){
			retval += fvec[i];
		}
		retval /= values_;
		printf("ti = %f ts=%f\tSSD Average = %4.6f\r\n",ti,ts,retval);
		return 0;
		//~ fvec[0] = SSD_Quaternion(times[0], times[1], m_Frames, m_Qa, m_T, m_Len, m_cLen, m_IK, m_matIntrinsic);
		//~ return 0;
	}
	
};

/**
 * @brief OpenCV等の歪補正の係数に関して逆方向に変換する逆係数を計算します。
 * @param [in] undistortedPoints	OpenCVなどで歪補正した、歪補正済み画像上の点の座標の組
 * @param [in] refPoints			歪補正前の歪んだ画像上の点の座標の組
 **/
struct calc_invert_distortion_coeff : Functor<double>
{
	calc_invert_distortion_coeff(int inputs, int values, std::vector<double> &undistortedPointsX, std::vector<double> &undistortedPointsY, std::vector<double> &refPointsX, std::vector<double> &refPointsY, Matrix3d &intrinsicCoeff)
	: m_undistortedPointsX(undistortedPointsX),m_undistortedPointsY(undistortedPointsY), m_refPointsX(refPointsX), m_refPointsY(refPointsY), m_intrinsicCoeff(intrinsicCoeff), Functor(inputs, values) {}
	
	std::vector<double> m_undistortedPointsX;
	std::vector<double> m_undistortedPointsY;
	std::vector<double> m_refPointsX;
	std::vector<double> m_refPointsY;
	Matrix3d m_intrinsicCoeff;

	int operator()(const VectorXd& K, VectorXd& fvec) const
    {
		double fx = m_intrinsicCoeff(0, 0);
		double fy = m_intrinsicCoeff(1, 1);
		double cx = m_intrinsicCoeff(0, 2);
		double cy = m_intrinsicCoeff(1, 2);
		
		double k1 = K[0];
		double k2 = K[1];
		double p1 = K[2];
		double p2 = K[3];
		
		for(int i=0,e=values_;i<e;i++){
			
			double u = m_undistortedPointsX[i];
			double v = m_undistortedPointsY[i];

			
			//後々の行列演算に備えて、画像上の座標を同次座標で表現しておく。(x座標、y座標,1)T
			//~ cv::Mat p = (cv::Mat_<double>(3,1) << (u - cx)/fx, (v - cy)/fy, 1.0);	//1のポリゴン座標に、K^-1を掛けた結果の３x１行列
			//~ cv::Mat XYW = R.inv() * p;
			
			//~ double x1 = XYW.at<double>(0, 0)/XYW.at<double>(2, 0);
			//~ double y1 = XYW.at<double>(1, 0)/XYW.at<double>(2, 0);
			double x1 = (u - cx)/fx;
			double y1 = (v - cy)/fy;
			
			double r = sqrt(x1*x1+y1*y1);
			
			double x2 = x1*(1.0+k1*r*r+k2*r*r*r*r)+2.0*p1*x1*y1+p2*(r*r+2.0*x1*x1);
			double y2 = y1*(1.0+k1*r*r+k2*r*r*r*r)+p1*(r*r+2.0*y1*y1)+2.0*p2*x1*y1;
			double mapx = x2*fx+cx;
			double mapy = y2*fy+cy;
			
			fvec[i] 	= (mapx - m_refPointsX[i])*(mapx - m_refPointsX[i]) + (mapy - m_refPointsY[i])*(mapy - m_refPointsY[i]);
			//~ fvec[i] 	= mapx - m_refPointsX[i];
			//~ fvec[i+1] 	= mapy - m_refPointsY[i];
		}

        return 0;
    }
};
