#ifndef VISION_OBJECT_COLOR_IMAGE_CONFIG
#define VISION_OBJECT_COLOR_IMAGE_CONFIG

class ColorImageConfig {
	public:
		double blurRad;
		double cannyThresh;
		double coefColor;
		double coefKeypoints;
		double coefShape;
		int histBins;
		int minHessian;
		double maxTotalError;
		int numbMatchesHomography;
};

int colorImageConfigBuild(ColorImageConfig & outConfig);

#endif