#ifndef VISION_OBJECT_TRAINDATA
#define VISION_OBJECT_TRAINDATA

#include <string>
#include <vector>

#include "colorimagedata.h"
#include "depthimagedata.h"

class TrainData {
	public:
		static int find(
			const std::string inPath,
			std::vector<TrainData> & outImageDataVect
		);

		int angle;
		std::string path;
		ColorImageData colorImageData;
		DepthImageData depthImageData;

		int train();
};

#endif