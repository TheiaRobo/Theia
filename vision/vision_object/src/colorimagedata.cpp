#include <algorithm>
#include <iostream>
#include <limits>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "colorimagedata.h"

using namespace cv;

ColorImageResult::ColorImageResult(){
	colorError = std::numeric_limits<double>::infinity();
	keypointError = std::numeric_limits<double>::infinity();
	shapeError = std::numeric_limits<double>::infinity();
	totalError = std::numeric_limits<double>::infinity();
}

void ColorImageResult::calcTotalError(const ColorImageContext & inContext){
	double total = 0;
	total += inContext.coefColor * colorError;
	total += inContext.coefKeypoints * keypointError;
	total += inContext.coefShape * shapeError;

	totalError = total;
}

/**
* Find the n best matching keypoint pairs based on their
* descriptor difference vector.
*/
int ColorImageResult::getBestMatches(
	int inNumbMatches,
	std::vector<cv::DMatch> & outMatches
) const {
	int errorCode = 0;

	std::vector<DMatch> workingCopy(matches);
	sort(workingCopy.begin(), workingCopy.end());

	size_t totalMatches = workingCopy.size();
	size_t numbMatches = inNumbMatches;
	if(numbMatches > totalMatches){
		numbMatches = totalMatches;
	}

	outMatches = std::vector<DMatch>(
		workingCopy.begin(),
		workingCopy.begin() + numbMatches
	);

	return errorCode;
}

bool ColorImageResult::isBetterThan(const ColorImageResult & result) const {
	return isBetterThan(result.totalError);
}

bool ColorImageResult::isBetterThan(double maxTotalError) const {
	return (totalError < maxTotalError);
}

bool ColorImageResult::isGoodEnough(const ColorImageContext & inContext) const {
	return isBetterThan(inContext.maxTotalError);
}

/*
bool ColorImageData::isWall(const ColorImageContext & inContext){
	double sum = 0;
	const size_t minBin = inContext.histBins - 2;
	const size_t maxBin = inContext.histBins;
	for(size_t bin = minBin; bin < maxBin; bin++){
		sum += hist[0][bin];
		sum += hist[1][bin];
		sum += hist[2][bin];
	}
	
	return (sum > 0.8);
}
*/

/**
* Finding the coordinate transformation between a training image and
* the object candidate image. This allows us to determine the precise
* orientation of an object in the scene.
*/  
int ColorImageData::findHomography(
	const ColorImageData & inSample,
	const ColorImageContext & inContext,
	ColorImageResult & ioResult
){
	int errorCode = 0;

	size_t numbMatches = ioResult.matches.size();
	if(numbMatches < 4){
		std::cout << "Error in " << __FUNCTION__ << std::endl;
		std::cout << "Not enough keypoints for finding homography" << std::endl;
		return -1;
	}

	// construct point vectors  	
	std::vector<Point2f> objectPoints;
	std::vector<Point2f> samplePoints;
	for(size_t i = 0; i < numbMatches; i++){
		size_t objectPointIndex = ioResult.matches[i].queryIdx;
		size_t samplePointIndex = ioResult.matches[i].trainIdx;

		objectPoints.push_back(keypoints[objectPointIndex].pt);
		samplePoints.push_back(inSample.keypoints[samplePointIndex].pt);
	}

	Mat homography;
	try{
		homography = cv::findHomography(
			objectPoints,
			samplePoints,
			CV_LMEDS,
			10
		);
	}catch(Exception ex){
		std::cout << "Error in " << __FUNCTION__ << std::endl;
		std::cout << "Could not find homography" << std::endl;
		return -1;
	}

	ioResult.homography = homography;

	return errorCode;
}

/**
* Run all object recognition algorithms and calculate final score.
*/
int ColorImageData::match(
	const ColorImageData & inSample,
	const ColorImageContext & inContext,
	ColorImageResult & outResult
){
	int errorCode = 0;

	errorCode = matchKeypoints(inSample, inContext, outResult);
	if(errorCode) return errorCode;

/*
	errorCode = findHomography(inSample, inContext, outResult);
	if(errorCode) return errorCode;
*/

	errorCode = matchHistogram(inSample, inContext, outResult);
	if(errorCode) return errorCode;

	errorCode = matchShape(inSample, inContext, outResult);
	if(errorCode) return errorCode;

	outResult.calcTotalError(inContext);
	
	return errorCode;
}

/**
* Calculate error between two color distribution functions.
*/
int ColorImageData::matchHistogram(
	const ColorImageData & inSample,
	const ColorImageContext & inContext,
	ColorImageResult & outResult
){
	int errorCode = 0;

	int method = CV_COMP_HELLINGER;
	double error = compareHist(hist, inSample.hist, method);
	outResult.colorError = error;
	
	return errorCode;
}

/**
* Determine pairs of matching keypoints between trainig image and
* object candidate. The error measurement is defined as the mean
* square error between keypoint descriptors.
*/
int ColorImageData::matchKeypoints(
	const ColorImageData & inSample,
	const ColorImageContext & inContext,
	ColorImageResult & outResult
){
	int errorCode = 0;

	size_t numbKeypoints = keypoints.size();
	size_t sampleNumbKeypoints = inSample.keypoints.size();

	if(!numbKeypoints || !sampleNumbKeypoints){
		std::cout << "Warning in " << __FUNCTION__ << std::endl;
		std::cout << "Matching of empty descriptor set" << std::endl;
		return errorCode;
	}

	std::vector<DMatch> matches;
	inContext.matcher.match(
		descriptors,
		inSample.descriptors,
		matches
	);

	size_t numbMatches = matches.size();
	if(!numbMatches){
		std::cout << "Warning in " << __FUNCTION__ << std::endl;
		std::cout << "Empty set of matches" << std::endl;
		return errorCode;
	}

	double totalError = 0;
	double totalSquareError = 0;
	for(size_t i = 0; i < numbMatches; i++){
		double distance = matches[i].distance;

		totalError += distance;
		totalSquareError += distance * distance;
	}

	outResult.keypointError = totalSquareError / numbMatches;
	
	return errorCode;
}

/**
* Match two shapes by calculating and comparing their Hu moments.
*/
int ColorImageData::matchShape(
	const ColorImageData & inSample,
	const ColorImageContext & inContext,
	ColorImageResult & outResult
){
	int errorCode = 0;

	if(!shape.size() || !inSample.shape.size()){
		std::cout << "Warning in " << __FUNCTION__ << std::endl;
		std::cout << "No shape found" << std::endl;
		return errorCode;
	}

	double error = matchShapes(
		shape,
		inSample.shape,
		CV_CONTOURS_MATCH_I1,
		0
	);
	outResult.shapeError = error;

	return errorCode;
}

/**
* Show the object candidate image with an overlay indicating the
* object's position.
* This function is used for debugging the coordinate transformation
* between an object candidate and a training image.
*/
int ColorImageData::showHomography(
	const ColorImageData & inSample,
	const ColorImageResult & inResult
){
	int errorCode = 0;

	std::vector<Point2f> corners(4);
	corners[0] = cvPoint(0, 0);
	corners[1] = cvPoint(gray.cols, 0);
	corners[2] = cvPoint(gray.cols, gray.rows);
	corners[3] = cvPoint(0, gray.rows);

	std::vector<Point2f> transformedCorners(4);
	perspectiveTransform(
		corners,
		transformedCorners,
		inResult.homography
	);

	Mat imageWithHomography(inSample.gray);
	for(size_t i = 0; i < 4; i++){
		line(
			imageWithHomography,
			transformedCorners[i],
			transformedCorners[(i + 1) % 4],
			Scalar(0, 255, 0),
			4
		);
	}

	imshow("Homography", imageWithHomography);
	waitKey();

	return errorCode;
}

/**
* Displays an image with all detected keypoints hightlighted.
* Only used for debugging.
*/
int ColorImageData::showKeypoints(){
	int errorCode = 0;

	Mat imageWithKeypoints;
	drawKeypoints(
		gray,
		keypoints,
		imageWithKeypoints,
		Scalar::all(-1),
		DrawMatchesFlags::DRAW_RICH_KEYPOINTS
	);

	imshow("Keypoints", imageWithKeypoints);
	waitKey(0);

	return errorCode;
}

/**
* Draw object candidate and training image side-by-side and
* indicate matching keypoints using lines.
* Only used for debugging.
*/
int ColorImageData::showMatches(
	const ColorImageData & inSample,
	const ColorImageResult & inResult
){
	int errorCode = 0;

	std::vector<DMatch> bestMatches;
	errorCode = inResult.getBestMatches(10, bestMatches);
	if(errorCode) return errorCode;
	
	Mat imageWithMatches;
	drawMatches(
		gray, keypoints,
		inSample.gray, inSample.keypoints,
		bestMatches,
		imageWithMatches,
		Scalar::all(-1),
		Scalar::all(-1),
		vector<char>(),
		DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
	);

	imshow("Matches", imageWithMatches);
	waitKey(0);

	return errorCode;
}

/**
* Read training image from file and launch training algorithm.
*/
int ColorImageData::train(const ColorImageContext & context){
	int errorCode = 0;

	if(path.empty()){
		std::cout << "Error in " << __FUNCTION__ << std::endl;
		std::cout << "No path given" << std::endl;
		return -1;
	}

	Mat image = imread(path, CV_LOAD_IMAGE_COLOR);
	if(!image.data){
		std::cout << "Error in " << __FUNCTION__ << std::endl;
		std::cout << "Could not read image" << std::endl;
		std::cout << path << std::endl;
		return -1;
	}

	errorCode = train(image, context);
	if(errorCode) return errorCode;

	return errorCode;
}

/**
* Calculating color distribution function from BRG8 image.
* Highly inspired by:
* http://docs.opencv.org/modules/imgproc/doc/histograms.html?highlight=calchist#calchist
*/
int ColorImageData::trainHistogram(const ColorImageContext & inContext){
	int errorCode = 0;

	int histBins = inContext.histBins;

/**
* In case of HLS8 image
*
	int histSize[] = {histBins, histBins};
	float hRange[] = {0, 180};
	float sRange[] = {0, 255};
	const float * ranges[] = {hRange, sRange};
	int channels[] = {0, 2}
*/

	int histSize[] = {histBins, histBins, histBins};
	float range[] = {0, 255};
	const float * ranges[] = {range, range, range};
	int channels[] = {0, 1, 2};

	MatND workingHist;
/**
* In case of HLS8 image
*
	calcHist(&color, 1, channels, Mat(), workingHist, 2, histSize, ranges);
*/	
	calcHist(&color, 1, channels, Mat(), workingHist, 3, histSize, ranges);
	normalize(workingHist, hist);

	return errorCode;
}

/**
* Detect keypoints and extract thei descriptor vectors.
*/
int ColorImageData::trainKeypoints(const ColorImageContext & inContext){
	int errorCode = 0;
	
	inContext.detector.detect(gray, keypoints);
	inContext.extractor.compute(gray, keypoints, descriptors);	

	return errorCode;
}

/**
* Find object shape.
*/
int ColorImageData::trainShape(const ColorImageContext & inContext){
	int errorCode = 0;

	const double blurRad = inContext.blurRad;
	const double cannyThresh = inContext.cannyThresh;

	Mat image = color.clone();
	
	/**
	* Convert color image go HSV
	*/
	Mat hsvImage;
	cvtColor(image, hsvImage, CV_BGR2HSV);

	/**
	* Reduce noise in HSV image for noise reduction
	*/
	Mat blurredHsvImage;
	blur(hsvImage, blurredHsvImage, Size(blurRad, blurRad));

	/**
	* Range of hue, saturation and value for the wodden floor
	* encountered in the competition maze.
	*/
	Scalar minColor(30 / 2, 0.00 * 255, 0.00 * 255);
	Scalar maxColor(60 / 2, 0.60 * 255, 0.90 * 255);

	/**
	* Find and enlarge pixel areas matching wooden floor.
	*/
	Mat floorMat;
	inRange(blurredHsvImage, minColor, maxColor, floorMat);
	blur(floorMat, floorMat, Size(10, 10));

	/**
	* Convert color image to grayscale and white out all
	* pixels that were identified as floor.
	*/
	Mat objectImage;
	Mat blurredImage;
	cvtColor(image, objectImage, CV_BGR2GRAY);
	add(objectImage, floorMat, objectImage, floorMat);
	blur(objectImage, blurredImage, Size(blurRad, blurRad));

	/**
	* Use Canny algorithm for edge detection in gray
	* scale image.
	*/
	Mat cannyImage;
	Canny(blurredImage, cannyImage, cannyThresh, 2 * cannyThresh);
	
	/**
	* Extract contours.
	*/
	vector< vector< Point > > contours;
	findContours(cannyImage, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	size_t numbContours = contours.size();
	if(!numbContours){
		std::cout << "Warning in " << __FUNCTION__ << std::endl;
		std::cout << "No contour found" << std::endl;
		return errorCode;
	}

	/**
	* Find biggest contour in the image.
	*/
	double maxContourArea = 0;
	size_t maxContourIndex = 0;
	for(size_t i = 0; i < contours.size(); i++){
		vector<Point> & contour = contours[i];
		
		Rect rect = boundingRect(contour);
		double area = rect.width * rect.height;

		if(area > maxContourArea){
			maxContourArea = area;
			maxContourIndex = i;
		}
	}

	shape = contours[maxContourIndex];

	return errorCode;
}

/**
* Launch all training algorithms.
*/
int ColorImageData::train(
	const Mat & inImage,
	const ColorImageContext & context
){
	int errorCode = 0;

	//cvtColor(inImage, color, CV_BGR2HLS);
	color = inImage.clone();
	cvtColor(inImage, gray, CV_BGR2GRAY);

	errorCode = trainKeypoints(context);
	if(errorCode){
		std::cout << "Error in " << __FUNCTION__ << std::endl;
		std::cout << "Keypoint training failed" << std::endl;
		return errorCode;
	}

/*
	errorCode = showKeypoints();
	if(errorCode){
		std::cout << "Error in " << __FUNCTION__ << std::endl;
		std::cout << "Could not show keypoints" << std::endl;
		return errorCode;
	}
*/

	errorCode = trainHistogram(context);
	if(errorCode){
		std::cout << "Error in " << __FUNCTION__ << std::endl;
		std::cout << "Histogram training failed" << std::endl;
		return errorCode;
	}

	errorCode = trainShape(context);
	if(errorCode){
		std::cout << "Error in " << __FUNCTION__ << std::endl;
		std::cout << "Shape training failed" << std::endl;
		return errorCode;
	}

	return errorCode;
}
