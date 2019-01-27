#ifndef _CALCSHIFT_HPP_
#define _CALCSHIFT_HPP_


#include <opencv2/opencv.hpp>
#include <iostream>
#include <cassert>
#include <cmath>
#include <numeric>
//#include "mFibonacci.hpp"

typedef struct{
	double TG;		//ジャイロセンサの周期[second]
	double TV;		//ビデオの周期[second]
	double TG2B;	//ジャイロセンサとビデオのタイミング[second]、ジャイロの時間にこの値を足すと、ビデオの時間に変換できる
}strTimingInformation;

std::vector<cv::Vec3d> CalcShiftFromVideo(const char *filename, int calcPeriod);

/**
 * @brief 波形を線形補間します。 
 * @param [in] waveform	波形
 * @param [in] index	フレーム[ ]
 * @retval 線形補間された値
 **/
template <typename Ta> Ta getIDP(std::vector<Ta> &waveform, double index){
	assert(index >= 0.0);
	assert(index <= (waveform.size()-1));
	
	int i = floor(index);
	double decimalPart = index - (double)i;
	return waveform[i]*(1.0-decimalPart)+waveform[i+1]*decimalPart;
}

/**
 * @brief ベクトル同士の相関を小数点以下の精度で計算します。
 * @param waveform		[in]	入力波形
 * @param window 		[in]	入力波形との相関を計算したい波形
 * @param index			[in]	相関を計算するビデオのフレーム[ ]
 * @param ti			[in]	タイミング情報
 * @retval 相関係数(-1から1の値で0は相関が無い、-1は負の相関、1は正の相関があることを示す)
  **/
template <typename Ta> Ta getCorrelationCoffecientInDecimal(std::vector<Ta> waveform, std::vector<Ta> window, double index, strTimingInformation &ti){
	assert(waveform.size()>=window.size());//ウィンドウサイズより波形データのほうが長くなければいけない。
	assert(waveform.size()!=0);
	assert(window.size()!=0);
	assert(index>=0.0);
	assert(index<(waveform.size()-window.size()-1));
	
	double cc;//相関係数
	auto avewindow = std::accumulate(window.begin(),window.end(),Ta())*(1.0/window.size());//windowの平均

	cv::Vec3d avewaveform;
	double dindex = index;
	for(int i=0,e=window.size();i<e;++i){//波形の平均値を計算
		avewaveform += getIDP(waveform,dindex*ti.TV/ti.TG);
		dindex+=1.0;
	}
	avewaveform*=(1.0/window.size());
	
	Ta sum1,sum2,sum3;
	dindex = index;

	for(int i=0,e=window.size();i<e;++i){
		sum1 += (window[i] - avewindow).mul(getIDP(waveform,dindex*ti.TV/ti.TG) - avewaveform);
		sum2 += (window[i] - avewindow).mul(window[i] - avewindow);
		sum3 += (getIDP(waveform,dindex*ti.TV/ti.TG) - avewaveform).mul(getIDP(waveform,dindex*ti.TV/ti.TG)- avewaveform);
		
		dindex += 1.0;
	}

	
	return Ta(sum1[0]/(sqrt(sum2[0]*sum3[0])),sum1[1]/(sqrt(sum2[1]*sum3[1])),sum1[2]/(sqrt(sum2[2]*sum3[2])));
}

/**
 * @brief 最も相関が高いフレームを線形補間により小数点以下の精度で求めます。ただし計算には画面内の回転しか使用しません。
 * @param waveform		[in]	入力波形
 * @param window		[in]	入力波形との相関を計算したい波形
 * @param is			[in]	フィボナッチ探索を開始するフレーム
 * @param ie			[in]	フィボナッチ探索を終了するフレーム
 * @param ti			[in]	タイミング情報
 * @retval 最も相関が高いフレーム[ ]
 **/
//template <typename Ta> double getHighestCorrelationIndex(std::vector<Ta> waveform, std::vector<Ta> window, double is, double ie, strTimingInformation &ti){
//	//フィボナッチ探索法の準備
//	const int mFIBONACCI_N = 12;				//フィボナッチ探索の探索回数
//	float RatioA[mFIBONACCI_N+1];			//ここの+1を忘れると動作しなくなる。。。。非常に重要。
//	float One_RatioA[mFIBONACCI_N+1];
//	GetRatio(mFIBONACCI_N,RatioA,One_RatioA);//比を予め計算しておく
//	//フィボナッチ探索法の準備ここまで
	
//	double F = is;
//	double T = ie;
//	double L,R;

//	int N = mFIBONACCI_N;
//	for(;N>0;N--){//繰り返し計算
//		//フレームを計算
//		L=One_RatioA[N]*F+RatioA[N]*T;//定数のテーブルから拾ってくる
//		R=RatioA[N]*F+One_RatioA[N]*T;
//		printf("L:%4.3f R;%4.3f ",L,R);
//		if(fabs(getCorrelationCoffecientInDecimal(waveform,window,L,ti)[2])<fabs(getCorrelationCoffecientInDecimal(waveform,window,R,ti)[2])){//画面内の回転しか計算に使っていないことに注意
//			F=L;
//			printf("F=L %4.3f\r\n",L);
//		}else{
//			T=R;
//			printf("T=R %4.3f\r\n",R);
//		}
//	}
	
//	//相関の最大値を表示。
//	std::cout << "maximum correlation:" << getCorrelationCoffecientInDecimal(waveform,window,(L+R)*0.5,ti) << std::endl;
	
//	return (F+T)*0.5;
//}

/**
 * @brief 波形間の粗い相関を全範囲に渡って計算
 * @param waveform		[in]	入力波形
 * @param window 			[in]	入力波形とのの相関を計算したい波形
 * @param ch				[in]	windowの特定の軸のみを計算に使う場合はx軸なら0を、y軸なら1を、z軸なら2を指定。負数で3軸を計算に使用。
 * @param ti				[in]	タイミング情報
 * @retval 相関[ ]
  **/
template <typename Ta> std::vector<Ta> getCorrelationCoffecient(std::vector<Ta> waveform, std::vector<Ta> window, int ch, strTimingInformation &ti){
	assert((waveform.size()*ti.TG/ti.TG)>=window.size());//ウィンドウサイズより波形データのほうが長くなければいけない。
	assert(waveform.size()!=0);
	assert(window.size()!=0);
	
	//ビデオのサンプリング周期に合わせたジャイロの波形を生成する
	std::vector<Ta> waveformV;
	for(double t=0.0,te=(waveform.size()-1)*ti.TG;t<=te;t+=ti.TV){
		waveformV.push_back(getIDP(waveform,t/ti.TG));
	}
	
	std::vector<Ta> cc;//相関係数
	auto avewindow = std::accumulate(window.begin(),window.end(),Ta())*(1.0/window.size());//windowの平均

	if(ch>=0){//チャンネルの指定があるとき
		for(auto it=waveformV.begin(),e=waveformV.end()-window.size()+1;it!=e;){
			auto avewaveform = std::accumulate(it,it+window.size(),Ta())*(1.0/window.size());//計算区間でのwaveformVの平均を計算
			
			auto itwindow = window.begin();
			Ta sum1,sum3;
			double sum2=0.0;
			for(auto it2 = it,e2=it+window.size()+1;it2!=e2;){//相関係数を計算するために必要な総和を計算
				sum1 += ((*itwindow)[ch] - avewindow[ch])*(*it2 - avewaveform);				//Vec3d
				sum2 += ((*itwindow)[ch] - avewindow[ch])*((*itwindow)[ch] - avewindow[ch]);//scholar
				sum3 += (*it2 - avewaveform).mul(*it2 - avewaveform);						//Vec3d
				++it2;
				++itwindow;
			}
			//~ std::cout << "sum1:" << sum1 << "sum2" << sum2 << "sum3" << sum3 << std::endl;
			//~ std::cout << sum1 << sum2 << sum3 << std::endl;
			//~ cc.push_back(Ta(sum1[0]/(sqrt(sum2[0]*sum3[0])),sum1[1]/(sqrt(sum2[1]*sum3[1])),sum1[2]/(sqrt(sum2[2]*sum3[2]))));
			cc.push_back(Ta(sum1[0]/(sqrt(sum2*sum3[0])),sum1[1]/(sqrt(sum2*sum3[1])),sum1[2]/(sqrt(sum2*sum3[2]))));
			++it;
		}
	}else{//3軸の相関を取るとき
		for(auto it=waveformV.begin(),e=waveformV.end()-window.size()+1;it!=e;){
			auto avewaveform = std::accumulate(it,it+window.size(),Ta())*(1.0/window.size());//計算区間でのwaveformVの平均を計算
			auto itwindow = window.begin();
			Ta sum1,sum2,sum3;
			for(auto it2 = it,e2=it+window.size()+1;it2!=e2;){//相関係数を計算するために必要な総和を計算
				sum1 += (*itwindow - avewindow).mul(*it2 - avewaveform);
				sum2 += (*itwindow - avewindow).mul(*itwindow - avewindow);
				sum3 += (*it2 - avewaveform).mul(*it2 - avewaveform);
				++it2;
				++itwindow;
			}
			cc.push_back(Ta(sum1[0]/(sqrt(sum2[0]*sum3[0])),sum1[1]/(sqrt(sum2[1]*sum3[1])),sum1[2]/(sqrt(sum2[2]*sum3[2]))));
			++it;
		}
	}
	
	return cc;
}

#endif //_CALCSHIFT_HPP_
