#ifndef VISION_OBJECT_DEPTHIMAGEDATA
#define VISION_OBJECT_DEPTHIMAGEDATA

#include <string>
#include <opencv2/core/core.hpp>

class DepthImageConfig {
	public:
		int blurring;
		int contourMode;
		int contourMethod;
		double cannyLevelOne;
		double cannyLevelTwo;
};

class DepthImageContext {
	public:
		int blurring;
		int contourMode;
		int contourMethod;
		double cannyLevelOne;
		double cannyLevelTwo;

		DepthImageContext(const DepthImageConfig & config);
};

class DepthImageResult {
	public:
		static DepthImageResult worst();
		
		double error;

		bool isBetterThan(const DepthImageResult & result);
};

class DepthImageData {
	public:
		std::string path;
		cv::Mat image;
		std::vector< std::vector<cv::Point> > contours;

		int match(
			const DepthImageData & inSample,
			const DepthImageContext & inContext,
			DepthImageResult & outResult
		);
		int show();
		int train(const DepthImageContext & inContext);
		int train(
			const cv::Mat & inImage,
			const DepthImageContext & inContext
		);
};

#endif