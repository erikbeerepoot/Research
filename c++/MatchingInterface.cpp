//MatchingInterface.cpp

#include <math.h>
#include <matrix.h>
#include <mex.h>

#include <string>
#include <stdio.h>
#include <iostream>

#include "DisplayImage.hpp"

#define IMG_PATH_STRING_1 	prhs[0]
#define IMG_PATH_STRING_2 	prhs[1]
#define BOOL_DOPLOT		  	prhs[2]
#define BOOL_DOSAVE			prhs[3]
#define FEATURE_TYPE_STRING	prhs[4]

#define MATCHING_SCORE_INT		plhs[0]
#define	AVG_TRACK_LENGTH_INT	plhs[1]

std::string GetString(const mxArray *argumentPointer);
bool GetBool(const mxArray *argumentPointer);

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	DisplayImage();
  // if(nlhs < 2) {
  // 	mexErrMsgIdAndTxt("MATLAB:mexcpp:nargout","Wrong number of outputs");
  // }

  // if(nrhs < 5){
  // 	mexErrMsgIdAndTxt("MATLAB:mexcpp:nargin","Wrong number of inputs");
  // }


  // if(!mxIsChar(IMG_PATH_STRING_1)){
  // 	mexErrMsgIdAndTxt("MatchingInterface:ParseArguments","The first argument must be of char* type!");
  // }

  // if(!mxIsChar(IMG_PATH_STRING_2)){
  // 	mexErrMsgIdAndTxt("MatchingInterface:ParseArguments","The first argument must be of char * type!");
  // }

  // if(!mxIsLogical(BOOL_DOSAVE)){
  // 	mexErrMsgIdAndTxt("MatchingInterface:ParseArguments","The third argument must be of logical type!");
  // }

  // if(!mxIsLogical(BOOL_DOPLOT)){
  // 	mexErrMsgIdAndTxt("MatchingInterface:ParseArguments","The fourth argument must be of logical type!");
  // }

  // if(!mxIsChar(FEATURE_TYPE_STRING)){
 	// mexErrMsgIdAndTxt("MatchingInterface:ParseArguments","The fifth argument must be of char * type!");
  // }



  // //parse parameters (image paths,plotting params, feature type)
  // std::string imgPath1,imgPath2,featureType;
  // bool doPlot,doSave;

  // imgPath1 = GetString(IMG_PATH_STRING_1);
  // imgPath2 = GetString(IMG_PATH_STRING_2);
  // featureType = GetString(FEATURE_TYPE_STRING);
  // doPlot = GetBool(BOOL_DOPLOT);
  // doSave = GetBool(BOOL_DOSAVE);

  //  //run algorithm and return output
  // double *matchingScore,*avgTrackLength;
  // MATCHING_SCORE_INT = mxCreateDoubleMatrix(1,1, mxREAL);
  // AVG_TRACK_LENGTH_INT = mxCreateDoubleMatrix(1,1, mxREAL);
  // matchingScore = mxGetPr(MATCHING_SCORE_INT);
  // avgTrackLength = mxGetPr(AVG_TRACK_LENGTH_INT);

  // CompareImagesByDescriptor(imgPath1,imgPath2,doPlot,doSave,featureType,matchingScore,avgTrackLength);
}

// std::string GetString(const mxArray *argumentPointer){
// 	if(!mxIsChar(argumentPointer)){
//   		mexErrMsgIdAndTxt("GetString:CheckType","The first argument must be of char * type!");
//   	}
	
// 	int stringLength = mxGetN(argumentPointer) +1;
// 	if(stringLength < 1){
// 		mexErrMsgIdAndTxt("GetString:CheckValidLength","Empty string passed as argument!");
// 	}

// 	char *ptr_string = (char*)mxCalloc(stringLength,sizeof(char));
// 	if(ptr_string==NULL){
// 		mexErrMsgIdAndTxt("GetString:MemoryError","No space left on the heap!");	
// 	}

// 	mxGetString(argumentPointer,ptr_string,stringLength);
// 	return std::string(ptr_string,stringLength-1);
// }

// bool GetBool(const mxArray *argumentPointer){
// 	if(!mxIsLogical(argumentPointer)){
//   		mexErrMsgIdAndTxt("GetBool:CheckType","Argument given is not of logical type!");
//   	}

//   	return mxGetLogicals(argumentPointer);
// }