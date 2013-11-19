#ifndef VISION_COMMON_FILE
#define VISION_COMMON_FILE

#include <string>
#include <vector>

#define VISION_DIR_SEP '/'

int visionFileFindDirs(
	const std::string inPath,
	std::vector<std::string> & outDirVect
);
bool visionFileExists(const std::string inPath);

#endif