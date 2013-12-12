#ifndef VISION_OBJECT_CANDIDATE_CONFIG
#define VISION_OBJECT_CANDIDATE_CONFIG

class CandidateConfig {
	public:
		/**
		* Coordinates of a bounding box for valid object candidates.
		* These values are relative to the robot position and in meters.
		*/
		double minX;
		double maxX;
		double minY;
		double maxY;
		double minZ;
		double maxZ;
};

int candidateConfigBuild(CandidateConfig & outConfig);

#endif