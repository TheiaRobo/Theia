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
} TheiaImage;

int theiaImageDetectKeypoints(
	TheiaImage & image,
	TheiaImageContext & context
);

int theiaImageShowKeypoints(TheiaImage & image);

int theiaImageExtractDescriptors(
	TheiaImage & image,
	TheiaImageContext & context
);

#endif