#ifndef VISION_OBJECT_CAMERA_CONFIG
#define VISION_OBJECT_CAMERA_CONFIG

class CameraConfig {
	public:
		double posX;
		double posY;
		double posZ;
		double angle;
		double fovLat;
		double fovLong;
};

int cameraConfigBuild(CameraConfig & outConfig);

#endif