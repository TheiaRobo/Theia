#ifndef VISION_COMMON_FILE
#define VISION_COMMON_FILE

#include <string>
#include <vector>

#define VISION_DIR_SEPARATOR '/'

/**
* Recursively scan a directory for files
*/
int visionFileScanDir(
	std::string dirName,
	std::vector<std::string> & fileVect
);

#endif