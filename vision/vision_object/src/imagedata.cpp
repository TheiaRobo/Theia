#include <cstdlib>
#include <vision/file.h>

#include "imagedata.h"

using namespace std;

int ImageData::find(
	const string inPath,
	vector<ImageData> & outImageDataVect
){
	int errorCode = 0;

	outImageDataVect.clear();

	vector<string> dirVect;
	errorCode = visionFileFindDirs(inPath, dirVect);
	if(errorCode) return errorCode;

	size_t numbFiles = dirVect.size();
	for(size_t i = 0; i < numbFiles; i++){
		ImageData imageData;
		string dirName = dirVect[i];

		// directory name must be a number
		size_t dirNameSize = dirName.size();
		bool onlyDigits = true;
		for(size_t i = 0; i < dirNameSize; i++){
			if(!isdigit(dirName[i])){
				onlyDigits = false;
				break;
			}
		}

		if(!onlyDigits){
			continue;
		}

		imageData.angle = strtoul(dirName.c_str(), NULL, 10);
		imageData.path = inPath + VISION_DIR_SEP + dirName;

		string colorImagePath = imageData.path + VISION_DIR_SEP + "color.png";
		string depthImagePath = imageData.path + VISION_DIR_SEP + "depth.png";

		if(
			!visionFileExists(colorImagePath)
			|| !visionFileExists(depthImagePath)
		){
			continue;
		}

		imageData.colorImageData.path = colorImagePath;
		imageData.depthImageData.path = depthImagePath;

		outImageDataVect.push_back(imageData);
	}

	return errorCode;
}