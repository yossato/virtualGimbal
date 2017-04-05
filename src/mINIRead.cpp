#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include "mINIRead.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <sstream> 

#ifdef __cplusplus
extern "C"
{
#endif


const int  INICheckerBoardParamNum = sizeof(strCheckerBoardParams)/sizeof(double);							//!<チェッカーボード設定項目数
const char *INICheckerBoardValueNames[INICheckerBoardParamNum] = {	//!<チェッカーボード設定
	//~ "ImageWidth",
	//~ "ImageHeight",
	"NumberOfCaptureImage",
	"SizeOfQuadsX_mm",
	"SizeOfQuadsY_mm",
	"NumberOfInnerCornersX",
	"NumberOfInnerCornersY"
};




//実体を定義しておく
strCheckerBoardParams Dcbp;


/**
* @brief カメラ行列設定を記録したテキストファイルを読み込みます
* @param [in] FileName		テキストファイルの名前。例:"intrinsic.txt"
* @param [out] Values		読み取った値を書き込む構造体。double[3][3]かdouble[9]の配列とすること
* @retval 0で正常
**/
int ReadStereoExtrinsicParams(const char FileName[], cv::Mat &Values){
	FILE *fp;
	const int Num = 16;
	double valuesd[16];// = (double*)(void*)&Values;
	// テキストファイルを読み込み用に開く
	if ((fp=fopen(FileName, "r")) != 0){
		// ファイルオープンに失敗
		//printf("File Open Error.\n");
		printf("File Open Error. \"%s\" is not found.\n", FileName);
		return 1;
	}

	for (int i = 0; i < 4; i++){
		if (fscanf(fp, "%lf %lf %lf %lf", &valuesd[i * 4], &valuesd[i * 4 + 1], &valuesd[i * 4 + 2], &valuesd[i * 4 + 3]) == -1){//1行ずつ行列を読み込み
			//エラー処理
			printf("エラー:固有パラメータファイルの読み込みに失敗しました。\r\n");
			printf("Check format of \"%s\".\n", FileName);
			fclose(fp);
			return 1;
		}
	}
	int i;
	printf("%s\r\n", FileName);
	for (i = 0; i < Num; i++){

		Values.at<double>(i / 4, i % 4) = valuesd[i];//Matに保存
	}
	std::cout << Values << std::endl;
	printf("\r\n");
	fclose(fp);
	return 0;
}
/**
* @brief カメラ行列設定を記録したテキストファイルを読み込みます
* @param [in] FileName		テキストファイルの名前。例:"intrinsic.txt"
* @param [out] Values		読み取った値を書き込む行列。
* @retval 0で正常
**/
int ReadIntrinsicsParams(const char* FileName, cv::Mat &Values){
	std::ifstream ifs(FileName);//CSVファイルを開く
	if(!ifs){
		printf("エラー：カメラ行列を記録した%sが見つかりません\nプログラムを終了します\n",FileName);//TODO:abotrせずに開けなかったことを上に返すようにして、その場合は推定値を使うようにする
		abort();
	}
	std::string str;
	Values = cv::Mat::zeros(3,3,CV_64F);//matを初期化
	
	int j=0;
	while(getline(ifs,str)){//1行ずつ読み込む
		std::string token;
		std::istringstream stream(str);
		
		int i=0;
		while(getline(stream, token, ' ')){
			Values.at<double>(j,i++) = (double)stof(token);//値を保存
			if(i>3){
				printf("エラー:カメラ行列のパラメータファイルintrinsic.txtの書式が不正なため、プログラムを終了します。\n");
				abort();
			}
		}
		++j;
	}
	
	//~ std::cout << "カメラ行列:" << Values << std::endl;
	return 0;
	
}


/**
* @brief 歪補正パラメータを記録したテキストファイルを読み込みます
* @param [in] FileName		テキストファイルの名前。例:"distortion.txt"
* @param [out] Values		読み取った値を書き込む構造体。
* @retval 0で正常
**/
//template<typename T_dcoef>
int ReadDistortionParams(const char FileName[], cv::Mat &Values){
	std::ifstream ifs(FileName);//CSVファイルを開く
	if(!ifs){
		printf("エラー：歪パラメータを記録した%sが見つかりません\nプログラムを終了します\n",FileName);//TODO:abotrせずに開けなかったことを上に返すようにして、その場合は推定値を使うようにする
		abort();
	}
	std::string str;
	Values = cv::Mat::zeros(1,4,CV_64F);//matを初期化
	
	int j=0;
	while(getline(ifs,str)){//1行ずつ読み込む
		std::string token;
		std::istringstream stream(str);
		
		int i=0;
		while(getline(stream, token, ' ')){
			Values.at<double>(0,i++) = (double)stof(token);//値を保存
		}
	}
	return 0;
	
	
	
	//エラーチェック
	assert(Values.cols == 5);
	assert(Values.rows == 1);

	FILE *fp;
	const int Num = 5;
	//double *valuesd = (double*)(void*)Values.data();
	// テキストファイルを読み込み用に開く
	if (fopen(FileName, "r") != 0){
		// ファイルオープンに失敗
		printf("File Open Error. \"%s\" is not found.\n", FileName);
		return 1;
	}

	double valuesd[5] = { 0.0 };
	if (fscanf(fp, "%lf %lf %lf %lf %lf", &valuesd[0], &valuesd[1], &valuesd[2], &valuesd[3], &valuesd[4]) == -1){//1行読み込み
		//エラー処理
		printf("エラー:歪パラメータファイルの読み込みに失敗しました。\r\n");
		printf("Check format of \"%s\".\n", FileName);
		fclose(fp);
		return 1;
	}
	int i;
	printf("%s\r\n", FileName);
	for (i = 0; i < Num; i++){
		//printf("%f%t\n", valuesd[i]);
		Values.at<double>(0, i) = valuesd[i];//cv::Matに代入
	}
	std::cout << Values << std::endl;
	printf("\r\n");
	fclose(fp);
	return 0;

}

/**
 * @brief FIRフィルタの係数ファイルを読み込み
 **/
void ReadCoeff(std::vector<double> &coeff, const char* filename){
                std::ifstream ifs(filename);//CSVファイルを開く
        if(!ifs){
                std::cout << "エラー：係数が記録されているファイルが見つかりません\n" << std::endl;
                return;
        }
        std::string str;
        coeff.clear();

        while(getline(ifs,str)){//1行ずつ読み込む
                coeff.push_back(stod(str));
                //~ std::cout << coeff.back() << std::endl;
        }
        printf("length of coeff is %ld\n",coeff.size());
        return;
}

/**
 * @brief CSVファイルを読み込んで配列を返す関数
 **/
void ReadCSV(std::vector<cv::Vec3d> &w, const char* filename){
        std::ifstream ifs(filename);//CSVファイルを開く
        if(!ifs){
                std::cout << "エラー：CSVファイルが見つかりません\n" << std::endl;
                return;
        }

        std::string str;
        w.clear();
        cv::Vec3d numl;
        while(getline(ifs,str)){//1行ずつ読み込む
                std::string token;
                std::istringstream stream(str);

                int i=0;
                while(getline(stream, token, ',')){
                        double temp = stof(token);
                        numl[i++] = (double)temp;//値を保存
                }
                w.push_back(numl);
        }

        printf("size of w is %ld\n",w.size());
        return;
}

#ifdef __cplusplus
}
#endif


