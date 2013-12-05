#ifndef VISION_OBJECT_CONFIG
#define VISION_OBJECT_CONFIG

#include <string>

#include "cameraconfig.h"
#include "colorimageconfig.h"

class Config {
	public:
		std::string path;
		CameraConfig camera;
		ColorImageConfig colorImage;
};

int configBuild(Config & outConfig);

#endif