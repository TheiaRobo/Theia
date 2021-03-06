#ifndef VISION_OBJECT_COLORIMAGEDATA
#define VISION_OBJECT_COLORIMAGEDATA

#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "colorimagecontext.h"

/**
* This class contains all values resulting from object
* detection algorithms applied to the color image.
*/
class ColorImageResult {
	public:
		double colorError;
		double keypointError;
		double shapeError;
		double totalError;
		cv::Mat homography;
		std::vector<cv::DMatch> matches;

		ColorImageResult();
		void calcTotalError(const ColorImageContext & inContext);
		int getBestMatches(
			int inNumbMatches,
			std::vector<cv::DMatch> & outMatches
		) const;
		bool isBetterThan(const ColorImageResult & result) const;
		bool isBetterThan(double maxTotalError) const;
		bool isGoodEnough(const ColorImageContext & inContext) const;
};

/**
* The ColorImageData class contains all data needed to
* perform object detection algorithms on a color image.
*/
class ColorImageData {
	public:
		std::string path;
		// RGB8 version
		cv::Mat color;
		// HLS8 version
		cv::Mat gray;
		// Color distribution function
		cv::MatND hist;
		// all keypoint descriptors
		cv::Mat descriptors;
		// list of keypoints
		std::vector<cv::KeyPoint> keypoints;
		// object shape
		std::vector<cv::Point> shape;
		
		// bool isWall(const ColorImageContext & inContext);
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
		int matchHistogram(
			const ColorImageData & inSample,
			const ColorImageContext & inContext,
			ColorImageResult & outResult
		);
		int matchKeypoints(
			const ColorImageData & inSample,
			const ColorImageContext & inContext,
			ColorImageResult & outResult
		);
		int matchShape(
			const ColorImageData & inSample,
			const ColorImageContext & inContext,
			ColorImageResult & outResult
		);
		int trainHistogram(const ColorImageContext & inContext);
		int trainKeypoints(const ColorImageContext & inContext);
		int trainShape(const ColorImageContext & inContext);
};

#endif