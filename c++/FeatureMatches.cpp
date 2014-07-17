#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "FeatureMatches.hpp"



void CompareImagesBySurfFeatures(cv::Mat img1, cv::Mat img2,bool doAHE, bool doPlot, bool doSave, double &score, double &avgTrackLength);
void CompareSURFDescriptors(cv::Mat img1, cv::Mat img2, std::vector<cv::KeyPoint> keypoints1,std::vector<cv::KeyPoint> keypoints2, bool doPlot, bool doSave, double &matchingScore, double &avgTrackLength);

 int main(int argc, char **argv){
	
//	std::string imagePath = "/Users/erikbeerepoot/Datasets/Features/sim/moon/compensated-data/10-deg-s/1.0/0001/0001.asa.comp.png";
//	std::string imagePath2 = "/Users/erikbeerepoot/Datasets/Features/sim/moon/compensated-data/10-deg-s/1.0/0001/0003.asa.comp.png";

 	if(argc < 3){
 		std::cout << "Usage: FeatureMatches [/path/to/img1] [/path/to/img2]" << std::endl;
 		return 1;
 	}

 	const std::string imagePath(argv[1]);
 	const std::string imagePath2(argv[2]);

	cv::Mat compensatedImage = cv::imread(imagePath,CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat compensatedImage2 = cv::imread(imagePath2,CV_LOAD_IMAGE_GRAYSCALE);

	if(compensatedImage.empty()) {
		std::cout << "Image empty!" << std::endl;
		return 1;
	}
	
	//Compare images
	double score=0,avgTrackLength=0;
	//CompareImagesBySurfFeatures(compensatedImage,compensatedImage2,true,true,false,score,avgTrackLength);
	CompareImagesByDescriptor(imagePath, imagePath2, false, false, "SURF",&score, &avgTrackLength);
	
	#ifdef DEBUG
		std::cout << "Matching score: " << score << std::endl;
		std::cout << "avgTrackLength: " << avgTrackLength << std::endl;
	#else
		printf("%f,%f,\n",score,avgTrackLength);
	#endif

	return 0;
 }

void CompareImagesByDescriptor(std::string img1Path, std::string img2Path, bool doPlot, bool doSave, std::string descriptorName, double *score, double *trackLength){
	cv::Mat img1 = cv::imread(img1Path,CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat img2 = cv::imread(img2Path,CV_LOAD_IMAGE_GRAYSCALE);

	if(img1.empty() || img2.empty()){
		std::cout << "Images are empty or invalid path!" << std::endl;
		return;
	}

	if(descriptorName.compare("SURF")==0){
		CompareImagesBySurfFeatures(img1,img2,true,doPlot,doSave,*score,*trackLength);
	} else if(descriptorName.compare("FAST")==0){
		return;
	} else if(descriptorName.compare("Harris")==0){
		return;
	} else if(descriptorName.compare("HarrisAffine")==0){
		return;
	} else if(descriptorName.compare("MSER")==0){
		return;
	} else {
		std::cout << "Could not recognize descriptor type!" << std::endl;
		return;
	}
}

void CompareImagesBySurfFeatures(cv::Mat img1, cv::Mat img2,bool doAHE, bool doPlot, bool doSave, double &score, double &avgTrackLength){
	//SURF parameters
	const int minHessian = 400;

	//function variables
	std::vector<cv::KeyPoint> keypoints1,keypoints2;

	//assign default values for output variables
	score = 0.0;
	avgTrackLength = 0.0;

	cv::Mat im1,im2;

	//apply adaptive histogram equalization
	if(doAHE){
		cv::equalizeHist(img1,im1);
		cv::equalizeHist(img2,im2);
	} else {
		im1 = img1;
		im2 = img2;
	}

	//detect surf features
	cv::SurfFeatureDetector detector(minHessian);
	detector.detect(im1,keypoints1);
	detector.detect(im2,keypoints2);

	if(doPlot){
		cv::Mat im1withKeypoints,im2withKeypoints;

		//draw kp and plot
		cv::drawKeypoints(im1,keypoints1,im1withKeypoints,cv::Scalar::all(-1),cv::DrawMatchesFlags::DEFAULT);
		cv::drawKeypoints(im2,keypoints2,im2withKeypoints,cv::Scalar::all(-1),cv::DrawMatchesFlags::DEFAULT);

		cv::imshow("Image 1",im1withKeypoints);
		cv::imshow("Image 2",im2withKeypoints);
	}

	 CompareSURFDescriptors(im1,im2,keypoints1,keypoints2,true,false,score,avgTrackLength);	
}

void CompareSURFDescriptors(cv::Mat img1, cv::Mat img2, std::vector<cv::KeyPoint> keypoints1,std::vector<cv::KeyPoint> keypoints2, bool doPlot, bool doSave, double &matchingScore, double &avgTrackLength){
	int descriptorIndex = 0;
	double magnitudeSum = 0.0;
	
	//output variables
	int numberOfMatches = 0;

	if(keypoints1.empty() || keypoints2.empty()) return;
	

	//comparison parameters
	const double kDescThreshold = 0.030;		//summed descriptor can vary by no more than this
	const double kDistanceThreshold = 200; 	//200 px
	const int knn = 1;
	//function variables
	cv::Mat descriptors1,descriptors2;

	//compute descriptors
	cv::SurfDescriptorExtractor extractor;
	extractor.compute(img1,keypoints1,descriptors1);
	extractor.compute(img2,keypoints2,descriptors2);
	if(descriptors1.empty() || descriptors2.empty()) return;	

	cv::Mat queryDescriptors,referenceDescriptors;
	std::vector<cv::KeyPoint> queryKeypoints, referenceKeypoints;
	if(descriptors1.rows > descriptors2.rows){
		queryDescriptors = descriptors2;
		queryKeypoints = keypoints2;
		referenceDescriptors = descriptors1;
		referenceKeypoints = keypoints1;
	} else {
		queryDescriptors = descriptors1;
		queryKeypoints = keypoints1;
		referenceDescriptors = descriptors2;
		referenceKeypoints = keypoints2;
	}

	//construct kdtree and associated variables
	cv::Mat indices(knn,2,CV_32F,0);
	cv::Mat dists(knn,2,CV_32F,0);
	cv::flann::Index index(referenceDescriptors,cv::flann::KDTreeIndexParams(8));

	
	

	while(descriptorIndex++ < (queryDescriptors.rows-1)){	
		index.knnSearch(queryDescriptors.row(descriptorIndex),indices,dists,knn,cv::flann::SearchParams(32));

		//grab the nn vector, and dot with the query vector
		double dottedDescriptors = (1 - (queryDescriptors.row(descriptorIndex)).dot(referenceDescriptors.row(indices.at<int>(0,0))));		
		if(dottedDescriptors < kDescThreshold){		
			double magnitude = 0.0;
			cv::Point vector(fabs(queryKeypoints.at(descriptorIndex).pt.x - referenceKeypoints.at(indices.at<int>(0,0)).pt.x),fabs(queryKeypoints.at(descriptorIndex).pt.y - referenceKeypoints.at(indices.at<int>(0,0)).pt.y));
			magnitude = sqrt(vector.x*vector.x + vector.y*vector.y);
			
			#ifdef DEBUG
				std::cout << "magnitude: " << vector.x << std::endl;
			#endif	
			
			if(magnitude > kDistanceThreshold){
				descriptorIndex = descriptorIndex + 1; 
				continue;
			}

			if(doPlot || doSave){
				//plot and/or save
			}
			magnitudeSum += magnitude;
			numberOfMatches++;
		}		
	    
	}
	#ifdef DEBUG
		std::cout << "Number of matches: " << numberOfMatches << std::endl;
	#endif
	matchingScore = static_cast<double>(numberOfMatches) / queryDescriptors.rows;
	avgTrackLength = static_cast<double>(magnitudeSum) / numberOfMatches;	
}