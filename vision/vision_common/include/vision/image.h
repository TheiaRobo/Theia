#ifndef VISION_COMMON_IMAGE
#define VISION_COMMON_IMAGE

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>

typedef struct {
	cv::SurfFeatureDetector detector;
	cv::SurfDescriptorExtractor extractor;
} TheiaImageContext;

typedef struct {
	cv::Mat image;
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
} TheiaImageData;

int theiaImageCreateContext(
	int minHessian,
	TheiaImageContext & context
);

int theiaImageDetectKeypoints(
	TheiaImageData & data,
	TheiaImageContext & context
);

int theiaImageShowKeypoints(TheiaImageData & data);

int theiaImageExtractDescriptors(
	TheiaImageData & data,
	TheiaImageContext & context
);

#endif