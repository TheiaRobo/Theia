#ifndef VISION_PLANE
#define VISION_PLANE

class Config {
	public:
		// leaf size used for voxel grid in meters
		double leafSize;

		// minimum number of elements in point cloud for object candidate
		int minClusterSize;

		// minimum fraction of elements in point cloud for plane
		double minPercentage;

		// number of RANSAC iterations
		int numbIterations;

		// 
		double objectSize;
		
		// maximum distance to plane used for RANSAC
		double planeDistThresh;

		// optimize coefficients of plane model
		bool planeOptimize;
};

#endif