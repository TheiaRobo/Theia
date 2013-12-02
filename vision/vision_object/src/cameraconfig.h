#ifndef VISION_OBJECT_CAMERA_CONFIG
#define VISION_OBJECT_CAMERA_CONFIG

class CameraConfig {
	public:
		double fovLat;
		double fovLong;
		double validFovLat;
		double validFovLong;
};

int cameraConfigBuild(CameraConfig & outConfig);

#endif