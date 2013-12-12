#ifndef VISION_OBJECT_CANDIDATE
#define VISION_OBJECT_CANDIDATE

#include <vector>
#include <opencv2/core/core.hpp>
#include <vision_plane/Box.h>

#include "cameracontext.h"
#include "candidatecontext.h"

class Candidate {
	public:
		/**
		* Latitude and longitude of the object in rads
		* relative to the camera's facing direction.
		*/
		double camLatMin;
		double camLatMax;
		double camLongMin;
		double camLongMax;
		
		/**
		* Bounding box around the object relative to the
		* robot. All units are meters.
		*/
		double robXMin;
		double robXMax;
		double robYMin;
		double robYMax;
		double robZMin;
		double robZMax;

		Candidate();
		Candidate(
			const vision_plane::Box & inBox,
			const CameraContext & inContext
		);
		bool isValid(const CandidateContext & inContext) const;
		int print() const;
		int toRect(
			const CameraContext & inContext,
			const cv::Mat & inImage,
			cv::Rect & outRect
		) const;

	protected:
		int calcCamCoordsFromBox(
			const vision_plane::Box & inBox,
			const CameraContext & inContext
		);
		int calcRobCoordsFromBox(
			const vision_plane::Box & inBox,
			const CameraContext & inContext
		);
};

int candFilterValid(
	const std::vector<Candidate> & inCands,
	const CandidateContext & inContext,
	std::vector<Candidate> & outCands
);

int candShow(
	const std::vector<Candidate> & inCandVect,
	const CameraContext & inContext,
	const cv::Mat & inImage
);

#endif