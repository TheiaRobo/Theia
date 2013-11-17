#ifndef VISION_OBJECT_FILE
#define VISION_OBJECT_FILE

#include <string>
#include <vector>

typedef struct {
	std::string path;
	std::string object;
	int rotation;
} ObjectFileTrain_t;

void objectFilesFilter(
	std::vector<std::string> & fileVect,
	std::vector<ObjectFileTrain_t> & trainFileVect
);

void objectFilesShow(
	std::vector<ObjectFileTrain_t> & trainFileVect
);

#endif