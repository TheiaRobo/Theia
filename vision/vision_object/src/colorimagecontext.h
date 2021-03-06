#ifndef VISION_OBJECT_COLOR_IMAGE_CONTEXT
#define VISION_OBJECT_COLOR_IMAGE_CONTEXT

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include "colorimageconfig.h"

class ColorImageContext {
	public:
		/**
		* Wrapper around OpenCV feature detector and descriptor
		* extractor containing all their configuration values
		*/
		cv::SurfFeatureDetector detector;
		cv::SurfDescriptorExtractor extractor;
		cv::BFMatcher matcher;
		
		double blurRad;
		double cannyThresh;
		double coefColor;
		double coefKeypoints;
		double coefShape;
		int histBins;
		double maxTotalError;
		int numbMatchesHomography;

		ColorImageContext(const ColorImageConfig & config);
};

#endif