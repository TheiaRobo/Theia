#ifndef VISION_OBJECT_IMAGEDATA
#define VISION_OBJECT_IMAEGDATA

#include <string>
#include <vector>

#include "colorimagedata.h"
#include "depthimagedata.h"

class ImageData {
	public:
		static int find(
			const std::string inPath,
			std::vector<ImageData> & outImageDataVect
		);

		int angle;
		std::string path;
		ColorImageData colorImageData;
		DepthImageData depthImageData;

		int train();
};

#endif