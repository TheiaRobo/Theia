#ifndef VISION_OBJECT_COLORIMAGEDATA
#define VISION_OBJECT_COLORIMAGEDATA

#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>

class ColorImageConfig {
	public:
		int minHessian;
};

class ColorImageContext {
	public:
		cv::SurfFeatureDetector detector;
		cv::SurfDescriptorExtractor extractor;

		ColorImageContext(const ColorImageConfig & config);
};

class ColorImageData {
	public:
		std::string path;
		cv::Mat image;
		std::vector<cv::KeyPoint> keypoints;
		cv::Mat descriptors;

		int train(const ColorImageContext & context);
};

#endif