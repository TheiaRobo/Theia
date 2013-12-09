#ifndef VISION_OBJECT_CANDIDATE_CONFIG
#define VISION_OBJECT_CANDIDATE_CONFIG

class CandidateConfig {
	public:
		double minX;
		double maxX;
		double minY;
		double maxY;
		double minZ;
		double maxZ;
};

int candidateConfigBuild(CandidateConfig & outConfig);

#endif