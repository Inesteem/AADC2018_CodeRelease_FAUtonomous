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
 * $Author:: schoen $   $Date:: 2016-02-08 #$
 **********************************************************************/

#ifndef CROSSING_INTERSECTIONDETECTOR_H_
#define CROSSING_INTERSECTIONDETECTOR_H_

#include "CVMath.h"
#include "ImageProcessingUtils.h"
#include "../Pose3D.h"
#include "../lines/LineDetectorVertical.h"
#include "../lines/LineDetectorHorizontal.h"

class IntersectionDetector {
	Rect binaryImageSize;

	LineDetector_Vertical vDetector;
	LineDetector_Vertical *vDetectorRight_rot;
	LineDetector_Vertical *vDetectorMiddle_Splitted;
	LineDetector_Horizontal hDetector;

	Mat endRightLine;
	Mat upperLeftCorner;
	cv::Point2f upperLeftCornerLineCenter;
	cv::Point2f lastUpperLeftCornerMatchLoc;

	bool debugEnabled;

	deque<Pose3D> detectedStopPosition;

	bool isInTrackingMode;

	int distanceEmergencyStop;
public:
	static const int DISTANCE_THICK_THIN_LINE_CROSSING;
	static const int DISTANCE_ADJACENT_SIGN;
	static const int DISTANCE_OPPOSITE_SIGN;

	static const int RIGHT_LINE_END_SEARCH_RANGE;
	static const int SIGN_DISTANCE_EMERGENCY_STOP;

	IntersectionDetector(int distanceEmergencyStop = SIGN_DISTANCE_EMERGENCY_STOP);
	virtual ~IntersectionDetector();

	int GetEmergencyStopDistance() {
		return distanceEmergencyStop;
	}

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

	bool DetectCrossingLine(Mat &input, Mat *debugImage, long long time);
	bool DetectStraightLine(Mat &input, Mat *debugImage);

	cv::Point2f DetectIntersection(Mat &input, Mat *debugImage, float imageRoadSignDistance, float imageNearPlaneDistance, Point2f *yawDirection, bool *badCrossingType, long long int time);

	Point2f OnLeftTurn(Mat &input, Mat *debugImage);
	Point2f OnRightTurn(Mat &input, Mat *debugImage);
};

#endif /* CROSSING_INTERSECTIONDETECTOR_H_ */
