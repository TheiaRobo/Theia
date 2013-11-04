#ifndef VISION_COMMON_FILE
#define VISION_COMMON_FILE

#include <string>
#include <vector>

typedef struct {
    std::string path;
    std::string objectName;
    int angle;
} VisionTrainingFile_t;

/**
* Recursively scan a directory for files
*/
int visionFileScanDir(
	std::string dirName,
	std::vector<std::string> & fileVect
);

/**
* Find training files from file list
*/
int visionFileFilterTraining(
	std::vector<std::string> & fileVect,
	std::vector<VisionTrainingFile_t> & trainingFileVect
);

#endif