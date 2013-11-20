#ifndef VISION_OBJECT
#define VISION_OBJECT

#include <string>
#include <vector>

#include "traindata.h"

class Object {
	public:
		static int find(
			const std::string inPath,
			std::vector<Object> & outObjectVect
		);

		std::string name;
		std::string path;
		std::vector<TrainData> trainDataVect;

		int train(const TrainContext & context);
};

#endif