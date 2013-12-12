#ifndef VISION_OBJECT_OBJECTDATA
#define VISION_OBJECT_OBJECTDATA

#include <string>
#include <vector>

#include "colorimagedata.h"
#include "context.h"

/**
* The ObjectDataResult class groups the detection results
* from all object recgnition algorithms.
* Initially it contained color image result and results
* based on depth image data.
*/
class ObjectDataResult {
	public:
		int angle;
		ColorImageResult colorImage;

		ObjectDataResult();
		bool isBetterThan(const ObjectDataResult & result) const;
		bool isGoodEnough(const Context & inContext) const;
};

/**
* This class serves as container for all data about a
* training object from ONE single perspective.
* If training images from multiple perspectives is needed
* they are grouped in the Object class.
*/
class ObjectData {
	public:
		static int find(
			const std::string inPath,
			std::vector<ObjectData> & outObjectDataVect
		);

		int angle;
		std::string path;
		ColorImageData colorImage;

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