#include <stdio.h>
#include <iostream>
#include <fstream>

// #include <math.h>
// #include <matrix.h>
// #include <mex.h>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "DisplayImage.hpp"

void DisplayImage(const std::string &imagePath){
	cv::Mat compensatedImage = cv::imread(imagePath,CV_LOAD_IMAGE_COLOR);

	if(! compensatedImage.data )                              // Check for invalid input
    {
       std::cout <<  "Could not open or find the image" << std::endl ;
       return;
    }

	cv::imshow("Image 1",compensatedImage);
	cv::waitKey(0);



 }

 int main(int argc, char **argv){
    //parse arguments
    if(argc < 2) {
        std::cout << "Usage: DisplayImage [/path/to/image/]" << std::endl;
    }

	std::string imagePath(argv[1]);
	DisplayImage(imagePath);
}