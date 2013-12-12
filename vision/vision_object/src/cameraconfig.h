#ifndef VISION_OBJECT_CAMERA_CONFIG
#define VISION_OBJECT_CAMERA_CONFIG

class CameraConfig {
	public:
		/**
		* Position of the camera relative to the robot's center
		* of gravity. All units are meters.
		*/
		double posX;
		double posY;
		double posZ;

		// angle to the vertical in degrees 
		double angle;

		// lateral field of view in degrees
		double fovLat;
		
		// longitudinal field of view in degrees
		double fovLong;
};

int cameraConfigBuild(CameraConfig & outConfig);

#endif