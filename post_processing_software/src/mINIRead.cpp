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


#ifdef __cplusplus
}
#endif


