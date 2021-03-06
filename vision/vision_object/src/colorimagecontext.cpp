#include "colorimageconfig.h"
#include "colorimagecontext.h"

using namespace cv;

ColorImageContext::ColorImageContext(const ColorImageConfig & config) :
detector(config.minHessian),
extractor(),
matcher(NORM_L2)
{
	blurRad = config.blurRad;
	cannyThresh = config.cannyThresh;
	coefColor = config.coefColor;
	coefKeypoints = config.coefKeypoints;
	coefShape = config.coefShape;
	histBins = config.histBins;
	maxTotalError = config.maxTotalError;
	numbMatchesHomography = config.numbMatchesHomography;
}
