#ifndef VISION_OBJECT_OBJECT
#define VISION_OBJECT_OBJECT

#include <string>
#include <vector>

#include "context.h"
#include "objectdata.h"

class Object {
	public:
		static int find(
			const std::string inPath,
			std::vector<Object> & outObjectVect
		);

		// object name
		std::string name;
		// path to directory containing all trainig files
		std::string path;
		// list with training data from multiple perspectives
		std::vector<ObjectData> objectDataVect;

		int train(const Context & context);
		int match(
			const ObjectData & inSampleData,
			const Context & inContext,
			ObjectDataResult & outResult
		);
};

#endif