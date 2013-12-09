#include "colorimageconfig.h"
#include "colorimagecontext.h"

using namespace cv;

ColorImageContext::ColorImageContext(const ColorImageConfig & config) :
detector(config.minHessian),
extractor(),
matcher(NORM_L2)
{
	histBins = config.histBins;
	maxTotalError = config.maxTotalError;
	numbMatchesHomography = config.numbMatchesHomography;
}
