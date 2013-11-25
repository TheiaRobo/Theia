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
		double maxMeanSquareError;
		int numbMatchesHomography;
};

class ColorImageContext {
	public:
		cv::SurfFeatureDetector detector;
		cv::SurfDescriptorExtractor extractor;
		cv::BFMatcher matcher;
		double maxMeanSquareError;
		int numbMatchesHomography;

		ColorImageContext(const ColorImageConfig & config);
};

class ColorImageResult {
	public:
		double meanError;
		double meanSquareError;
		double variance;
		cv::Mat homography;
		std::vector<cv::DMatch> matches;

		ColorImageResult();
		int getBestMatches(
			int inNumbMatches,
			std::vector<cv::DMatch> & outMatches
		) const;
		bool isBetterThan(const ColorImageResult & result) const;
		bool isBetterThan(double maxMeanSquareError) const;
};

class ColorImageData {
	public:
		std::string path;
		cv::Mat image;
		std::vector<cv::KeyPoint> keypoints;
		cv::Mat descriptors;

		int match(
			const ColorImageData & inSample,
			const ColorImageContext & inContext,
			ColorImageResult & outResult
		);
		int showHomography(
			const ColorImageData & inSample,
			const ColorImageResult & inResult
		);
		int showKeypoints();
		int showMatches(
			const ColorImageData & inSample,
			const ColorImageResult & inResult
		);
		int train(const ColorImageContext & context);
		int train(
			const cv::Mat & image,
			const ColorImageContext & context
		);

	protected:
		int findHomography(
			const ColorImageData & inSample,
			const ColorImageContext & inContext,
			ColorImageResult & ioResult
		);
		int matchKeypoints(
			const ColorImageData & inSample,
			const ColorImageContext & inContext,
			ColorImageResult & outResult
		);
};

#endif