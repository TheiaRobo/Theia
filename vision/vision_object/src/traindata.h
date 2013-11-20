#ifndef VISION_OBJECT_TRAINDATA
#define VISION_OBJECT_TRAINDATA

#include <string>
#include <vector>

#include "colorimagedata.h"
#include "depthimagedata.h"

class TrainConfig {
	public:
		ColorImageConfig colorImage;
		// DepthImageConfig depthImageConfig;
};

class TrainContext {
	public:
		ColorImageContext colorImage;
		// DepthImageContext depthImageContext;

		TrainContext(const TrainConfig & config);
};

class TrainData {
	public:
		static int find(
			const std::string inPath,
			std::vector<TrainData> & outImageDataVect
		);

		int angle;
		std::string path;
		ColorImageData colorImage;
		DepthImageData depthImage;

		int train(const TrainContext & context);
};

#endif