#ifndef VISION_OBJECT_COLOR_IMAGE_CONFIG
#define VISION_OBJECT_COLOR_IMAGE_CONFIG

class ColorImageConfig {
	public:
		int minHessian;
		double maxMeanSquareError;
		int numbMatchesHomography;
};

int colorImageConfigBuild(ColorImageConfig & outConfig);

#endif