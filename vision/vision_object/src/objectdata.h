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
		// DepthImageContext depthImage;

		Context(const Config & config);
};

class ObjectDataResult {
	public:
		ColorImageResult colorImage;
		// DepthImageResult depthImage

		bool isBetterThan(const ObjectDataResult & result);
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
		int match(
			const ObjectData & inSampleData,
			const Context & inContext,
			ObjectDataResult & outResult
		);
};

#endif