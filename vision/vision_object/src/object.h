#ifndef VISION_OBJECT_OBJECT
#define VISION_OBJECT_OBJECT

#include <string>
#include <vector>

#include "objectdata.h"

class Object {
	public:
		static int find(
			const std::string inPath,
			std::vector<Object> & outObjectVect
		);

		std::string name;
		std::string path;
		std::vector<ObjectData> objectDataVect;

		int train(const Context & context);
		int match(
			const ObjectData & inSampleData,
			const Context & inContext,
			ObjectDataResult & outResult
		);
};

#endif