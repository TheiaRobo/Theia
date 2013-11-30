#ifndef VISION_COMMON_CAMERA_CONFIG
#define VISION_COMMON_CAMERA_CONFIG

/**
* Unit for configuration:
* DEGREES
*/
typedef struct  {
	double fovLat;
	double fovLong;
	double validFovLat;
	double validFovLong;
} CameraConfig;

/**
* TODO
*/
typedef CameraConfig CameraContext;

#endif