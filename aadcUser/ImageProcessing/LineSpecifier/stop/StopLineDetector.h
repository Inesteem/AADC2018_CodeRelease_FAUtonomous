/**
 * Copyright (c)
 * Audi Autonomous Driving Cup. TEAM FAUtonOHM.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.
 * 4. Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **********************************************************************
 * $Author:: schoen $   $Date:: 2016-01-16 #$
 **********************************************************************/

#ifndef STOPLINEDETECTOR_H_
#define STOPLINEDETECTOR_H_

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <list>
#include <deque>
#include "CVMath.h"

#include "ImageProcessingUtils.h"
#include "../Pose3D.h"

using namespace cv;
using namespace std;

class StopLineDetector {
	int stepWidthX;
	int roiWidth;
	int laneWidth_Pixel;
	float maxMovementDistance;

	Rect binaryImageSize;

	Mat whiteBlack;
	Mat blackWhite;

	bool debugEnabled;

	deque<Pose3D> detectedStopPosition;

	static const int STOP_LINE_SEARCH_Y_LIMIT_IMAGE_COORD;
	int MAX_STOP_LINE_PIXEL_WIDTH;
	int MIN_STOP_LINE_PIXEL_WIDTH;

	bool isInTrackingMode;
public:
	StopLineDetector(int minStopLinePixelWidth = 6, int maxStopLinePixelWidth = 13) {
		MAX_STOP_LINE_PIXEL_WIDTH = maxStopLinePixelWidth;
		MIN_STOP_LINE_PIXEL_WIDTH = minStopLinePixelWidth;
		roiWidth = 3;

		laneWidth_Pixel = METER_TO_PIXEL(0.465);
		stepWidthX = METER_TO_PIXEL(0.465 / cv::sqrt(2)) / 4.0;

		maxMovementDistance = METER_TO_PIXEL(0.1);

		binaryImageSize = Rect(0, 0, BINARY_IMAGE_WIDTH, BINARY_IMAGE_HEIGHT);

		whiteBlack = Mat::zeros(6, roiWidth, CV_8UC1);
		rectangle(whiteBlack, Point(0,0), Point(roiWidth, 3), CV_FILLED);

		blackWhite = Mat::zeros(6, roiWidth, CV_8UC1);
		rectangle(whiteBlack, Point(0,3), Point(roiWidth, 6), CV_FILLED);

		debugEnabled = false;
		isInTrackingMode = false;
	}
	virtual ~StopLineDetector();

	/**
	 *
	 * @param input binary image
	 * @param debugImage colored image, if debug mode is disabled nothing is drawn to debugImage
	 * @param carPosition in image coordinates
	 * @param dest in image coordinates
	 * @return Point of the stop line or Point2f(0,0) if no stop line detected
	 */
	Point2f Detect(Mat &input, Mat &debugImage, Point2f carPosition, Point2f dest, Point2f *direction, long long  int timestamp);

	void switchDebugMode(bool enable) {
		debugEnabled = enable;
	}

	deque<Pose3D> GetDetectedStopPositions() {
		return detectedStopPosition;
	}

	void ClearDetectedStopPositions() {
		detectedStopPosition.clear();
	}

	bool IsInTrackingMode() {
		return isInTrackingMode;
	}

	void SetTrackingMode(bool value) {
		isInTrackingMode = value;
	}
private:

	float AngleLimit(float p) {
		if(p > 1) {
			p = 1;
		} else if (p < 0) {
			p = 0;
		}

		return (1 - p) * cos(5* CV_PI / 180) + p * cos(30* CV_PI / 180);

	}
	Point2f VerifyDetection(Mat &input, Mat &debugImage, list<Point2i> &foundStopMarkers, Point2f carPosition, Point2f steeringDirection, Point2f *direction, long long  int time);

	void EvaluateStopLineMarkers(std::list<Point2i> & stopMarkers, Point2f steeringDir);

	inline bool Helper_CheckCondition(float angle, float limit_angle, float length, float limit_length) {
		return angle < limit_angle || length > limit_length;
	}

	bool EstimateVerticalLine(Mat &input, Mat &debugImage, Rect roi, Point2i searchedSlope, CVMath::Line *outputLine);
};

#endif /* STOPLINEDETECTOR_H_ */
