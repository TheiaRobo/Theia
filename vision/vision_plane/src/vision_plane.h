#ifndef VISION_PLANE
#define VISION_PLANE

class Config {
	public:
		double leafSize;
		double minPercentage;
		int numbIterations;
		double objectSize;
		double planeDistThresh;
		bool planeOptimize;
};

#endif