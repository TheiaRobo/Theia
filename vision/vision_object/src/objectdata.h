#ifndef VISION_OBJECT_OBJECTDATA
#define VISION_OBJECT_OBJECTDATA

#include <string>
#include <vector>

#include "colorimagedata.h"
#include "depthimagedata.h"

class Config {
	public:
		std::string path;
		ColorImageConfig colorImage;
		DepthImageConfig depthImage;
};

class Context {
	public:
		std::string path;
		ColorImageContext colorImage;
		DepthImageContext depthImage;

		Context(const Config & config);
};

class ObjectDataResult {
	public:
		int angle;
		ColorImageResult colorImage;
		DepthImageResult depthImage;

		ObjectDataResult();
		bool isBetterThan(const ObjectDataResult & result) const;
		bool isGoodEnough(const Context & inContext) const;
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

		int match(
			const ObjectData & inSample,
			const Context & inContext,
			ObjectDataResult & outResult
		);
		int train(const Context & context);

	protected:
		int matchColors(
			const ObjectData & inSample,
			const Context & inContext,
			ObjectDataResult & ioResult
		);
};

#endif