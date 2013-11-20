#ifndef VISION_OBJECT_OBJECTDATA
#define VISION_OBJECT_OBJECTDATA

#include <string>
#include <vector>

#include "colorimagedata.h"
#include "depthimagedata.h"

class Config {
	public:
		ColorImageConfig colorImage;
		// DepthImageConfig depthImageConfig;
};

class Context {
	public:
		ColorImageContext colorImage;
		// DepthImageContext depthImageContext;

		Context(const Config & config);
};

class ObjectDataResult {
	// nothing
};

class ObjectData {
	public:
		static int find(
			const std::string inPath,
			std::vector<ObjectData> & outObjectDataVect
		);

		int angle;
		std::string path;
		ColorImageData colorImage;
		DepthImageData depthImage;

		int train(const Context & context);
};

#endif