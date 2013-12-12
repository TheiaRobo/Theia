#ifndef VISION_OBJECT_CONFIG
#define VISION_OBJECT_CONFIG

#include <string>

#include "cameraconfig.h"
#include "candidateconfig.h"
#include "colorimageconfig.h"

class Config {
	public:
		// path to the directory containing training images
		std::string path;
		// camera-related configuration values
		CameraConfig camera;
		// values related to object candidates
		CandidateConfig candidate;
		// configuration values for color image data
		ColorImageConfig colorImage;
};

int configBuild(Config & outConfig);

#endif