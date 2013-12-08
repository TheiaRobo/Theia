#ifndef VISION_OBJECT_CONTEXT
#define VISION_OBJECT_CONTEXT

#include <string>

#include "cameracontext.h"
#include "candidatecontext.h"
#include "config.h"
#include "colorimagecontext.h"

class Context {
	public:
		std::string path;
		CameraContext camera;
		CandidateContext candidate;
		ColorImageContext colorImage;

		Context(const Config & config);
};

#endif