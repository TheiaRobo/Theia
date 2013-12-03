#ifndef VISION_OBJECT_CONTEXT
#define VISION_OBJECT_CONTEXT

#include <string>

#include "cameracontext.h"
#include "config.h"
#include "colorimagecontext.h"

class Context {
	public:
		std::string path;
		CameraContext camera;
		ColorImageContext colorImage;

		Context(const Config & config);
};

#endif