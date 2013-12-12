#ifndef VISION_OBJECT_COLOR_IMAGE_CONFIG
#define VISION_OBJECT_COLOR_IMAGE_CONFIG

class ColorImageConfig {
	public:
		// radius of blurring applied before edge detection
		double blurRad;
		// threshold for the Canny edge detection algorithm
		double cannyThresh;
		// weight of the color-based object recognition 
		double coefColor;
		// weight of the feature-based object recognition
		double coefKeypoints;
		// weight of the shape-based object recognition
		double coefShape;
		/**
		* Number of bins used for calculating the color
		* distribution function.
		*/
		int histBins;
		/**
		* Minimal hessian value such that a pixel is considered
		* to be a image keypoint.
		*/
		int minHessian;
		/**
		* Maximum total error before a training object is
		* discarded as invalid.
		*/
		double maxTotalError;
		/**
		* Number of keypoints used for finding the
		* homography between training image and object candidate
		int numbMatchesHomography;
};

int colorImageConfigBuild(ColorImageConfig & outConfig);

#endif