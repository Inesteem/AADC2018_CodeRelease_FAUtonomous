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
 * $Author:: schoen $   $Date:: 2016-03-13 #$
 **********************************************************************/

#ifndef SPEED_SPEEDRECOMMENDER_H_
#define SPEED_SPEEDRECOMMENDER_H_


#include "../lines/LineDetectorVertical.h"
#include "../lines/LineDetectorHorizontal.h"
#include "ImageProcessingUtils.h"

class SpeedRecommender {
	cv::Rect binaryImageSize;

	LineDetector_Vertical vDetector;
	LineDetector_Vertical *vDetectorLeft_Splitted;

	cv::Point2f afterLeftCurve;

public:
	struct SpeedConfig {
		float straightSlowNear;
		float straightNormalNear;
		float straightSlowFar;
		float straightNormalFar;
		float min;
		float curve;
		float detection;

		SpeedConfig() {
			straightSlowNear = 0;
			straightNormalNear = 0;
			straightSlowFar = 0;
			straightNormalFar = 0;
			min = 0;
			curve = 0;
			detection = 0;

		}
	} speedConfig;

	int detectionModeActive;
	float isInCurve;
	float lastSpeed;
	float breakSpeed;
	float setSpeed;
	float straightProbability;
	float recommendedStraightSpeedSlow;
	float recommendedStraightSpeedNormal;
	float leftLineLength;
	float rightLineLength;

	Rect obstaclesROI;
	float obstacles;

	bool debugEnabled;

	SpeedRecommender() {
		detectionModeActive = 0;
		isInCurve = 0;
		lastSpeed = 0;
		breakSpeed = 0;
		setSpeed = 0;

		debugEnabled = false;
		straightProbability = 0.75;
		obstacles = 1;

		leftLineLength = 0;
		rightLineLength = 0;

		obstaclesROI = Rect((BINARY_IMAGE_WIDTH >> 1) - METER_TO_PIXEL(0.04), 0, METER_TO_PIXEL(0.08), BINARY_IMAGE_HEIGHT);

		recommendedStraightSpeedSlow = 0.5;
		recommendedStraightSpeedNormal = 0.5;

		vDetector.SetColor(Scalar(0, 255, 0));
		vDetectorLeft_Splitted = new LineDetector_Vertical(5, 4, 5);
		vDetectorLeft_Splitted->SetParameter(50, 10, 60, LineDetector_Vertical::INITIAL_STEP_INCREASE);

		vDetectorLeft_Splitted->SetColor(Scalar(0, 255, 0));
	}

	virtual ~SpeedRecommender() {
		delete vDetectorLeft_Splitted;
	}

	void switchDebugMode(bool enable) {
		debugEnabled = enable;
	}

	void DetectMaxSpeed(cv::Mat &in, cv::Mat *debugImage, float currentSteeringAngle, long long int time);

	float SpeedController(float probability, float prioritySignDistance, int actionSubCommand,
			float currentSteeringAngle);

	/**
	 *
	 * @param destination in meter
	 * @return in degree, between 60 and 120
	 */
	float DetermineSteeringAngle(cv::Mat &input, cv::Mat *debugImage, cv::Point2f destination, cv::Point2f offsetSteering, float steeringAngleScaleRight, float steeringAngleScaleLeft,  bool onLeftLane, long long int time);

	Point2f Backwards(cv::Mat &in, cv::Mat *debugImage,  cv::Point2f image_destination, long long int time, Point2f *direction, float image_yOffset);
};

#endif /* SPEED_SPEEDRECOMMENDER_H_ */
