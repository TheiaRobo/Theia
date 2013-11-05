#ifndef VISION_COMMON_FILE
#define VISION_COMMON_FILE

#include <string>
#include <vector>

/**
* Recursively scan a directory for files
*/
int visionFileScanDir(
	std::string dirName,
	std::vector<std::string> & fileVect
);

#endif