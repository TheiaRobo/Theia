#include "colorimageconfig.h"
#include "colorimagecontext.h"

using namespace cv;

ColorImageContext::ColorImageContext(const ColorImageConfig & config) :
detector(config.minHessian),
extractor(),
matcher(NORM_L2)
{
	maxMeanSquareError = config.maxMeanSquareError;
	numbMatchesHomography = config.numbMatchesHomography;
}
