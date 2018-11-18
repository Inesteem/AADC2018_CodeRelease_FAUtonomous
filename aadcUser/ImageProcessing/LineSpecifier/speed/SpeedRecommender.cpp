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

#include "SpeedRecommender.h"
#include "CVMath.h"
#include "ScmCommunication.h"
#include "../cache/DetectionCache.h"

void SpeedRecommender::DetectMaxSpeed(cv::Mat &input, cv::Mat *debugImage, float currentSteeringAngle,
		long long int time) {
	if (!debugEnabled) {
		debugImage = 0;
	}

	int yStart = BINARY_IMAGE_HEIGHT - 3;
	int xStart = BINARY_IMAGE_WIDTH / 2 + METER_TO_PIXEL(0.25);

	CVMath::LineSegment rightLine = DetectionCache::GetInstance().GetRightLine(time);

	if (!rightLine.Valid()) {
		if (vDetector.Detect_Up(input, debugImage, xStart, yStart, &rightLine)) {
			DetectionCache::GetInstance().SaveRightLine(rightLine, time);
		}
	}

	if (rightLine.Valid()) {
		if (cv::abs(rightLine.direction.x / rightLine.direction.y) < 0.176) {
			//tan(10) = 0.176

			straightProbability = rightLine.length * rightLine.length / (2 * BINARY_IMAGE_HEIGHT * BINARY_IMAGE_HEIGHT)
					+ 0.75;
			if (debugImage) {
				line(*debugImage, rightLine.start, rightLine.end, Scalar(0, 255, 10), 2, 4);
			}
		} else {
			rightLine = CVMath::LineSegment();
			straightProbability = 0.75;
		}
	} else {
		rightLine = CVMath::LineSegment();
		straightProbability = 0.75;
	}

	int whitePixel = cv::sum(input(obstaclesROI))[0];
	obstacles = 1 - whitePixel / float(255 * obstaclesROI.width * obstaclesROI.height);

	yStart = (BINARY_IMAGE_HEIGHT >> 1) + 50;
	xStart = 30;
	CVMath::LineSegment vertLeftLine;
	if (vDetectorLeft_Splitted->DetectSplitted_Up(input, debugImage, xStart, yStart, &vertLeftLine)) {
		DetectionCache::GetInstance().SaveOuterLeftLine(vertLeftLine, time);
		if (cv::abs(vertLeftLine.direction.x / vertLeftLine.direction.y) < 0.0875) {
			//tan(5) = 0.0875
			if (rightLine.length > METER_TO_PIXEL(1.25) && vertLeftLine.length > METER_TO_PIXEL(0.4)) {
				recommendedStraightSpeedNormal = speedConfig.straightNormalFar;
				recommendedStraightSpeedSlow = speedConfig.straightSlowFar;
			}

			straightProbability *= vertLeftLine.length * vertLeftLine.length
					/ (2 * BINARY_IMAGE_HEIGHT * BINARY_IMAGE_HEIGHT) + 0.75;

			if (debugImage) {
				line(*debugImage, vertLeftLine.start, vertLeftLine.end, Scalar(0, 255, 10), 2, 4);
			}
		} else {
			vertLeftLine = CVMath::LineSegment();
		}
	} else {
		vertLeftLine = CVMath::LineSegment();
	}

	if (lastSpeed > 0.2) {
		if (rightLine.length < rightLineLength - ImageUtils::MeterToPixel(lastSpeed * 0.033333)) {
			rightLineLength -= ImageUtils::MeterToPixel(lastSpeed * 0.033333);
		} else {
			rightLineLength = rightLine.length;
		}

		if (vertLeftLine.length < leftLineLength - ImageUtils::MeterToPixel(lastSpeed * 0.033333)) {
			leftLineLength -= ImageUtils::MeterToPixel(lastSpeed * 0.033333);
		} else {
			leftLineLength = vertLeftLine.length;
		}
	} else {
		rightLineLength = rightLine.length;
		leftLineLength = vertLeftLine.length;
	}

	if (recommendedStraightSpeedNormal >= speedConfig.straightNormalFar
			|| recommendedStraightSpeedSlow >= speedConfig.straightSlowFar) {
		if (leftLineLength < METER_TO_PIXEL(0.25) || rightLineLength < METER_TO_PIXEL(1)) {
			recommendedStraightSpeedNormal = speedConfig.straightNormalNear;
			recommendedStraightSpeedSlow = speedConfig.straightSlowNear;
		}
	}

	if (debugImage) {
		char info[100];
		sprintf(info, "v %.2f d %.2f b %.2f st %.0f P(C) %.2f", lastSpeed, setSpeed, breakSpeed, currentSteeringAngle,
				isInCurve);

        //cv::putText(*debugImage, cv::String(info), Point2f(0, 15), CV_FONT_NORMAL, 0.35, Scalar(0, 0, 255));
	}

	//TODO: remove
	LineDetector_Vertical redDetector(12, 4, 12);
	redDetector.SetColor(Scalar(0, 0, 255));
	CVMath::LineSegment redLine;
	redDetector.SetParameter(30, 10, 50, LineDetector_Vertical::INITIAL_STEP_INCREASE);
	if (redDetector.DetectSplitted_Up(input, debugImage, 100, 290, &redLine)) {
		if (redLine.length > 30) {
			if (debugImage) {
				line(*debugImage, redLine.start, redLine.end, Scalar(0, 0, 250), 2, 4);
			}

			//circle(*debugImage, redLine.start - Point2f(METER_TO_PIXEL(0.23), 0), 7.0, Scalar(0, 0, 255), 2, 8);
		}
	}
}

float SpeedRecommender::SpeedController(float probability, float prioritySignDistance, int actionSubCommand,
		float currentSteeringAngle) {
	float pSum = probability;// * obstacles;

	if (pSum > 1) {
		pSum = 1;
	}


	float min = speedConfig.min * (1.0 - pSum) + speedConfig.curve * pSum;
	float max = min;

        std::cout << "pre" << std::endl;
        std::cout << "min " << min << "speedconfig.min " << speedConfig.min << "speedconfig.curve" << speedConfig.curve << "psum " << pSum << std::endl;

	//float pSumStraight = pSum * (1 - isInCurve); // min( point there steering is 30 degree) is not affected by isInCurve probability
	if (prioritySignDistance < 0 && detectionModeActive == 0) {
		switch (actionSubCommand) {
		case AC_LS_SLOW_LEFTLANE:
			pSum *= 0.5;
		case AC_LS_SLOW_RIGHTLANE:
			max = speedConfig.min * (1.0 - pSum) + recommendedStraightSpeedSlow * pSum;
			break;
		case AC_LS_NORMAL_LEFTLANE:
			pSum *= 0.5;
		case AC_LS_NORMAL_RIGHTLANE:
			max = speedConfig.min * (1.0 - pSum) + recommendedStraightSpeedNormal * pSum;
			break;
		case AC_LS_DETECT_TRANS_PARKING_SPOT_SLOW:
		case AC_LS_DETECT_LONG_PARKING_SPOT_SLOW:
			min = speedConfig.min * (1.0 - pSum) + std::min(speedConfig.curve, speedConfig.detection) * pSum;
			max = speedConfig.min * (1.0 - pSum) + speedConfig.detection * pSum;
			break;
		}
	}

	if (detectionModeActive > 0) {
		min = speedConfig.min * (1.0 - pSum) + std::min(speedConfig.curve, speedConfig.detection) * pSum;
		max = speedConfig.min * (1.0 - pSum) + speedConfig.detection * pSum;
                std::cout << "dma active" << std::endl;
		detectionModeActive--;
	}


        std::cout << "post" << std::endl;
        std::cout << "max " << max  << " min " << min << " probability " << probability << " prioritySignDistance " << prioritySignDistance << std::endl;
	float absCurrentSteeringAngle = cv::abs(currentSteeringAngle);
	if (absCurrentSteeringAngle < 10) {
                float pStraight = (1 - isInCurve * (10 - absCurrentSteeringAngle) * 0.1);
		max = speedConfig.min * (1.0 - pStraight) + max * pStraight;
	}

	float currentSpeed = ((min - max) * 0.03333 * absCurrentSteeringAngle + max);

	if (currentSpeed < lastSpeed * 0.9) {
		breakSpeed = lastSpeed - currentSpeed;
		breakSpeed = 0.8 * breakSpeed * breakSpeed;

		if (breakSpeed > 1) {
			breakSpeed = 1;
		}
	} else {
		breakSpeed = 0;
	}

	float diffSpeed = currentSpeed - lastSpeed;

	if (diffSpeed > 0.1) {
		diffSpeed = 0.1;
	} else if (diffSpeed < -0.3) {
		diffSpeed = -0.3;
	}

	lastSpeed += diffSpeed;

	setSpeed = currentSpeed - breakSpeed;

	if (setSpeed < 0) {
		setSpeed = 0;
	}

        std::cout << "setSpeed " << setSpeed  << " detectionMode " << detectionModeActive << " isInCurve " << isInCurve << std::endl;
	return setSpeed;
}

float SpeedRecommender::DetermineSteeringAngle(cv::Mat &input, cv::Mat *debugImage, cv::Point2f destination,
		cv::Point2f offsetSteering, float steeringAngleScaleRight, float steeringAngleScaleLeft, bool onLeftLane,
		long long int time) {
	if (!debugEnabled) {
		debugImage = 0;
	}

	destination -= offsetSteering;


	if (destination.x < 0) {
		destination.x = -destination.x;
	} else if (destination.x == 0) {
		return 0;
	}

	if (destination.y > 0 && !onLeftLane) {
		CVMath::LineSegment rightLine = DetectionCache::GetInstance().GetRightLine(time);

		if (!rightLine.Valid()) {
			int yStart = BINARY_IMAGE_HEIGHT - 3;
			int xStart = BINARY_IMAGE_WIDTH / 2 + METER_TO_PIXEL(0.25);

			if (vDetector.Detect_Up(input, debugImage, xStart, yStart, &rightLine)) {
				DetectionCache::GetInstance().SaveRightLine(rightLine, time);
			}
		}

		if (rightLine.Valid() && rightLine.length > METER_TO_PIXEL(0.3)) {
			float tanValue = (rightLine.start.x - rightLine.end.x) / cv::abs(rightLine.end.y - rightLine.start.y);
			float length = ImageUtils::PixelToMeter(rightLine.length) - 0.3;
			//image coordinates
			if (tanValue > 0.01) {
				float weight = 0.14 / tanValue + length;
				if (weight > 1) {
					weight = 1;
				}
				//local coordinates
				afterLeftCurve = isInCurve * Point2f(0, 0.06) * weight; // in meter
			} else {
				afterLeftCurve *= 0.5;
			}
		}
	}

	//destination += afterLeftCurve;
	if (debugImage) {
		line(*debugImage, Point2f(150, 300), Point2f(150, 300) + afterLeftCurve.y * Point2f(-1500, 0),
				Scalar(0, 0, 250), 2, 4);
	}

	float steeringAngle = -atan(destination.y / destination.x) * 180 / 3.1415;

	float curveRelevantSteering = cv::abs(steeringAngle) > 4 ? steeringAngle : 0;
	isInCurve = 0.9 * (curveRelevantSteering * curveRelevantSteering * 0.0015f + isInCurve);
	if (isInCurve > 1) {
		isInCurve = 1;
	}

	afterLeftCurve = isInCurve * 4 > 1 ? afterLeftCurve : afterLeftCurve * isInCurve * 4;

	if (steeringAngle > 3) {
		steeringAngle *= steeringAngleScaleRight;
		if (steeringAngle > 90) {//30
			steeringAngle = 90;
		}
	} else if (steeringAngle < -3) {
		steeringAngle *= steeringAngleScaleLeft;
		if (steeringAngle < -90) {
			steeringAngle = -90;
		}
	}

	return steeringAngle;
}

Point2f SpeedRecommender::Backwards(cv::Mat &in, cv::Mat *debugImage, cv::Point2f image_destination, long long int time,
		Point2f *direction, float image_yOffset) {
	CVMath::LineSegment rightLine = DetectionCache::GetInstance().GetRightLine(time);

	if (rightLine.Valid() && rightLine.length > METER_TO_PIXEL(0.1) && rightLine.start.x > image_destination.x) {
		Point2f e = rightLine.start + METER_TO_PIXEL(0.235) * CVMath::RotateCW90(rightLine.direction);
		Point2f inter = CVMath::IntersectionTwoLines(Point2f(0, 1), BINARY_IMAGE_HEIGHT + image_yOffset, e,
				rightLine.direction);

		*direction = rightLine.direction;
		return inter;

		circle(*debugImage, inter, 5.0, Scalar(0, 0, 255), 2, 8);
	} else {
		Point2f inter = Point2f(BINARY_IMAGE_WIDTH / 2, BINARY_IMAGE_HEIGHT + image_yOffset);
		Point2f vec = image_destination - inter;

		*direction = vec /= cv::norm(vec);

		return inter;
	}
}
