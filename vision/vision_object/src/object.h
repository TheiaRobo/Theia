#ifndef VISION_OBJECT
#define VISION_OBJECT

#include <string>
#include <vector>

#include "imagedata.h"

class Object {
	public:
		static int find(
			const std::string inPath,
			std::vector<Object> & outObjectVect
		);

		std::string name;
		std::string path;
		std::vector<ImageData> imageDataVect;

		int train();
};

#endif