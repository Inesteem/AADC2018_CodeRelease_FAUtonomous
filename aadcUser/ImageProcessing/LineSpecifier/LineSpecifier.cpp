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
 * $Author:: schoen $   $Date:: 2015-12-14 #$
 **********************************************************************/

#include "LineSpecifier.h"
#include "LineTemplateGenerator.h"
#include "ImageProcessingUtils.h"
using namespace cv;

LineSpecifier::LineSpecifier() {
	lastPoint = cv::Point2f(150, 300);
	lastWeight = 1;
	debugEnabled = false;
	firstPoint = true;

	lastResettedFrame = 0;

	lastLaneWidth = LineTemplateGenerator::AVERAGE_LANE_WIDTH;
}

void LineSpecifier::Init() {
	lastPoint = cv::Point2f(150, 300);
	lastWeight = 1;
	debugEnabled = false;
	firstPoint = true;

	lastResettedFrame = 0;

	lastLaneWidth = LineTemplateGenerator::AVERAGE_LANE_WIDTH;


	InitStraight();
	InitRightTurn();
	InitLeftTurn();
}

void LineSpecifier::InitStraight() {
	int index = 0, size = 0;
	LineTemplate first;

	////////////////////////////////////////////////////
	index = 0, size = 4;
	first.roi = Rect(60, 282, 100, 16);
	first.lineCenter = Point2f(4.5, 7.5);
	LineTemplateGenerator::GenerateStraight(straightMiddle, first, Point2f(0, -7), index, size, 4,
			LineTemplateGenerator::LINE_ON_LEFT_SIDE);

	first.roi = Rect(150, 282, 90, 16);
	first.lineCenter = Point2f(25.5, 7.5);
	LineTemplateGenerator::GenerateStraight(straightRight, first, Point2f(0, -7), index, size, 6,
			LineTemplateGenerator::LINE_ON_RIGHT_SIDE);

	index += size;

	////////////////////////////////////////////////////
	// put some blurred versions of the straight lines
	size = 4;
	for (int i = 0; i < size; i++) {
		LineTemplate temp (Mat::zeros(16, 48, CV_8UC1), 32, 0.25);
		straightMiddle.push_back(temp);
		straightRight.push_back(temp);
	}

	first.roi = Rect(70, 282, 80, 16);
	first.lineCenter = Point2f(6.5, 7.5);
	LineTemplateGenerator::GenerateStraight(straightMiddle, first, Point2f(0, -7), index, size, 6,
			LineTemplateGenerator::LINE_ON_LEFT_SIDE);

	first.roi = Rect(150, 282, 80, 16);
	first.lineCenter = Point2f(39.5, 7.5);
	LineTemplateGenerator::GenerateStraight(straightRight, first, Point2f(0, -7), index, size, 8,
			LineTemplateGenerator::LINE_ON_RIGHT_SIDE);
}

void LineSpecifier::InitRightTurn() {
	int index = 0, size = 0;
	int y = 0;
	LineTemplate first;

	turnRightMiddle.resize(10);

	y= 272;
	first.roi = Rect(100, y, 100, 16);
	first.lineCenter = Point2f(8, 8);
	size = 4;
	LineTemplateGenerator::Generate(turnRightMiddle, first, index, size, 10, 1.5, 4, LineTemplateGenerator::RIGHT,
			LineTemplateGenerator::LINE_ON_LEFT_SIDE, LineTemplateGenerator::AVERAGE_LANE_WIDTH - 11, 282 - y);
	index += size;

	first.roi = Rect(110, 282, 100, 16);
	first.lineCenter = Point2f(8, 8);
	size = 5;
	LineTemplateGenerator::Generate(turnRightMiddle, first, index, size, 25, 1.5, 4, LineTemplateGenerator::RIGHT,
			LineTemplateGenerator::LINE_ON_LEFT_SIDE, LineTemplateGenerator::AVERAGE_LANE_WIDTH - 16);
	index += size;

	first.roi = Rect(120, 282, 100, 16);
	first.lineCenter = Point2f(8, 8);
	size = 4;
	LineTemplateGenerator::Generate(turnRightMiddle, first, index, size, 40, 1.5, 4, LineTemplateGenerator::RIGHT,
			LineTemplateGenerator::LINE_ON_LEFT_SIDE, LineTemplateGenerator::AVERAGE_LANE_WIDTH - 17);
	index += size;

	first.roi = Rect(130, 282, 100, 16);
	first.lineCenter = Point2f(8, 8);
	size = 3;
	LineTemplateGenerator::Generate(turnRightMiddle, first, index, size, 50, 1.5, 4, LineTemplateGenerator::RIGHT,
			LineTemplateGenerator::LINE_ON_LEFT_SIDE, LineTemplateGenerator::AVERAGE_LANE_WIDTH - 23);
	index += size;

	first.roi = Rect(140, 282, 100, 16);
	first.lineCenter = Point2f(8, 8);
	size = 3;
	LineTemplateGenerator::Generate(turnRightMiddle, first, index, size, 55, 1.5, 4, LineTemplateGenerator::RIGHT,
			LineTemplateGenerator::LINE_ON_LEFT_SIDE, LineTemplateGenerator::AVERAGE_LANE_WIDTH - 28);
	index += size;

	///////////////////////////////////////////////
	index = 0, size = 0;
	y= 272;
	first.roi = Rect(170, y, 70, 16);
	first.lineCenter = Point2f(32 - 8, 8);
	size = 6;
	LineTemplateGenerator::Generate(turnRightInnerRight, first, index, size, 30, 1, 6, LineTemplateGenerator::RIGHT,
			LineTemplateGenerator::LINE_ON_RIGHT_SIDE, LineTemplateGenerator::AVERAGE_LANE_WIDTH + 30, 282 - y);
	index += size;

	first.roi = Rect(180, y, 70, 16);
	first.lineCenter = Point2f(32 - 8, 8);
	size = 4;
	LineTemplateGenerator::Generate(turnRightInnerRight, first, index, size, 20, 1, 6, LineTemplateGenerator::RIGHT,
			LineTemplateGenerator::LINE_ON_RIGHT_SIDE, LineTemplateGenerator::AVERAGE_LANE_WIDTH + 30, 282 - y);
	index += size;

	///////////////////////////////////////////////
	// right turn outer left thick line 6cm
	index = 0, size = 0;

	y = 235;
	first.roi = Rect(25, y, 80, 16);
	first.lineCenter = Point2f(8, 8);
	size = 4;
	LineTemplateGenerator::Generate(turnRightOuterLeft, first, index, size, 10, 2, 6, LineTemplateGenerator::RIGHT,
			LineTemplateGenerator::LINE_ON_LEFT_SIDE, LineTemplateGenerator::AVERAGE_LANE_WIDTH * 3 - 10, 282 - y);
	index += size;

	y = 260;
	first.roi = Rect(25, y, 80, 16);
	first.lineCenter = Point2f(9, 8);
	size = 5;
	LineTemplateGenerator::Generate(turnRightOuterLeft, first, index, size, 20, 2, 6, LineTemplateGenerator::RIGHT,
			LineTemplateGenerator::LINE_ON_LEFT_SIDE, LineTemplateGenerator::AVERAGE_LANE_WIDTH * 3 - 10, 282 - y);
	index += size;

	y = 275;
	first.roi = Rect(35, y, 80, 16);
	first.lineCenter = Point2f(10, 8);
	size = 5;
	LineTemplateGenerator::Generate(turnRightOuterLeft, first, index, size, 30, 2, 6, LineTemplateGenerator::RIGHT,
			LineTemplateGenerator::LINE_ON_LEFT_SIDE, LineTemplateGenerator::AVERAGE_LANE_WIDTH * 3 - 10, 282 - y);
	index += size;

	//////////////////////////////////////////
	//generate some blurred versions

	/*
	 * TODO: removed blurred versions
*/

	index = turnRightMiddle.size(), size = 4;
	for (int i = 0; i < size; i++) {
		LineTemplate temp(Mat::zeros(16, 48, CV_8UC1), 32, 0.25);
		turnRightMiddle.push_back(temp);
	}
	first.roi = Rect(100, 272, 100, 16);
	first.lineCenter = Point2f(9, 8);
	size = 4;
	LineTemplateGenerator::Generate(turnRightMiddle, first, index, size, 10, 1.5, 6, LineTemplateGenerator::RIGHT,
			LineTemplateGenerator::LINE_ON_LEFT_SIDE); //blurred line not 2cm thick
	index += size;



	//////////////////////////////////////////
	//left outer

	index = turnRightOuterLeft.size();

	y = 235;
	first.roi = Rect(25, y, 80, 16);
	first.lineCenter = Point2f(8, 8);
	size = 4;
	LineTemplateGenerator::Generate(turnRightOuterLeft, first, index, size, 10, 2, 8, LineTemplateGenerator::RIGHT,
			LineTemplateGenerator::LINE_ON_LEFT_SIDE, LineTemplateGenerator::AVERAGE_LANE_WIDTH * 3 - 10, 282 - y);
	index += size;

	y = 260;
	first.roi = Rect(25, y, 80, 16);
	first.lineCenter = Point2f(9, 8);
	size = 5;
	LineTemplateGenerator::Generate(turnRightOuterLeft, first, index, size, 20, 2, 8, LineTemplateGenerator::RIGHT,
			LineTemplateGenerator::LINE_ON_LEFT_SIDE, LineTemplateGenerator::AVERAGE_LANE_WIDTH * 3 - 10, 282 - y);
	index += size;

	y = 275;
	first.roi = Rect(35, y, 80, 16);
	first.lineCenter = Point2f(10, 8);
	size = 5;
	LineTemplateGenerator::Generate(turnRightOuterLeft, first, index, size, 30, 2, 8, LineTemplateGenerator::RIGHT,
			LineTemplateGenerator::LINE_ON_LEFT_SIDE, LineTemplateGenerator::AVERAGE_LANE_WIDTH * 3 - 10, 282 - y);
	index += size;



}

void LineSpecifier::InitLeftTurn() {
	// turn left outer right line 6cm
	int index = 0, size = 0;
	LineTemplate first;

	first.roi = Rect(120, 282, 90, 16);
	first.lineCenter = Point2f(32 - 8, 8);
	size = 5;
	LineTemplateGenerator::Generate(turnLeft, first, index, size, 180 - 10, 1.5, 6, LineTemplateGenerator::LEFT,
			LineTemplateGenerator::LINE_ON_RIGHT_SIDE, LineTemplateGenerator::AVERAGE_LANE_WIDTH - 30);
	index += size;

	first.roi = Rect(110, 282, 90, 16);
	first.lineCenter = Point2f(32 - 8, 8);
	size = 5;
	LineTemplateGenerator::Generate(turnLeft, first, index, size, 180 - 25, 1.5, 6, LineTemplateGenerator::LEFT,
			LineTemplateGenerator::LINE_ON_RIGHT_SIDE, LineTemplateGenerator::AVERAGE_LANE_WIDTH - 30);
	index += size;

	first.roi = Rect(100, 282, 110, 16);
	first.lineCenter = Point2f(32 - 8, 8);
	size = 4;
	LineTemplateGenerator::Generate(turnLeft, first, index, size, 180 - 35, 1.5, 6, LineTemplateGenerator::LEFT,
			LineTemplateGenerator::LINE_ON_RIGHT_SIDE, LineTemplateGenerator::AVERAGE_LANE_WIDTH - 30);
	index += size;
}

LineSpecifier::~LineSpecifier() {

}

void LineSpecifier::DetermineDestinationPoint(cv::Mat& in, cv::Mat& out, cv::Point2f * dest, float * probability, HINT turn) {
	Mat *debugImage = 0;
	if(debugEnabled) {
		debugImage = &out;
	}

	lastResettedFrame--;

	for (unsigned int i = 0; i < straightRight.size(); i++) {
		if(firstPoint) {
			straightRight[i].Match(in, debugImage, Scalar(0, 255, 0), 0);
		} else {
			straightRight[i].Match(in, debugImage, Scalar(0, 255, 0),  &lastPoint, 0.8);
		}
	}

	for (unsigned int i = 0; i < straightMiddle.size(); i++) {
		if(firstPoint) {
			straightMiddle[i].Match(in, debugImage, Scalar(0, 255, 0), 0);
		} else {
			straightMiddle[i].Match(in, debugImage, Scalar(0, 255, 0),  &lastPoint, 0.8);
		}
	}

	for (unsigned int i = 0; i < turnRightMiddle.size(); i++) {
		if(firstPoint) {
			turnRightMiddle[i].Match(in, debugImage, Scalar(0, 255, 255), 0);
		} else {
			turnRightMiddle[i].Match(in, debugImage, Scalar(0, 255, 255), &lastPoint);
		}
	}

	for (unsigned int i = 0; i < turnRightInnerRight.size(); i++) {
		if(firstPoint) {
			turnRightInnerRight[i].Match(in, debugImage, Scalar(0, 0, 255), 0);
		} else {
			turnRightInnerRight[i].Match(in, debugImage, Scalar(0, 0, 255), &lastPoint, 1);
		}
	}

	for (unsigned int i = 0; i < turnRightOuterLeft.size(); i++) {
		if(firstPoint) {
			turnRightOuterLeft[i].Match(in, debugImage, Scalar(255, 0, 0), 0);
		} else {
			turnRightOuterLeft[i].Match(in, debugImage, Scalar(255, 0, 0), &lastPoint);
		}
	}

	for (unsigned int i = 0; i < turnLeft.size(); i++) {
		if(firstPoint) {
			turnLeft[i].Match(in, debugImage, Scalar(0, 0, 255), 0);
		} else {
			turnLeft[i].Match(in, debugImage, Scalar(0, 0, 255), &lastPoint, 0.5);
		}
	}

	//adjust lane width near distance
	Point2f lineRight, lineMiddle;
	float totalProbabilityRight = 0;
	float totalProbabilityMiddle = 0;

	if(turn == NO_HINT) {
		for (unsigned int i = 0; i < straightRight.size(); i++) {
			totalProbabilityRight += straightRight[i].probability;
		}

		for (unsigned int i = 0; i < straightMiddle.size(); i++) {
			totalProbabilityMiddle += straightMiddle[i].probability;
		}
	}

	if (totalProbabilityRight > 0 && totalProbabilityMiddle > 0
			&& abs(totalProbabilityRight - totalProbabilityMiddle) < 0.5) {
		for (unsigned int i = 0; i < straightRight.size(); i++) {
			lineRight += (straightRight[i].matchLocation + straightRight[i].lineCenter) * straightRight[i].probability;
		}

		for (unsigned int i = 0; i < straightMiddle.size(); i++) {
			lineMiddle += (straightMiddle[i].matchLocation + straightMiddle[i].lineCenter)
					* straightMiddle[i].probability;
		}

		lineRight /= totalProbabilityRight;
		lineMiddle /= totalProbabilityMiddle;

		float laneWidth = abs(lineRight.x - lineMiddle.x);

		//set some limits
		if (laneWidth < LineTemplateGenerator::AVERAGE_LANE_WIDTH - 15) {
			laneWidth = LineTemplateGenerator::AVERAGE_LANE_WIDTH - 15;
		} else if (laneWidth > LineTemplateGenerator::AVERAGE_LANE_WIDTH + 15) {
			laneWidth = LineTemplateGenerator::AVERAGE_LANE_WIDTH + 15;
		}

		//skip calculations
		if (abs(lastLaneWidth - laneWidth) > 5) {
			laneWidth *= 0.5;

			for (unsigned int i = 0; i < straightRight.size(); i++) {
				straightRight[i].AdjustLaneWidth(laneWidth);
			}

			for (unsigned int i = 0; i < straightMiddle.size(); i++) {
				straightMiddle[i].AdjustLaneWidth(laneWidth);
			}
		}
	}

	int count_straight_middle = 0;
	int count_straight_right = 0;
	int count_right = 0;
	int count_Left = 0;

	float probability_straight = 0.0;
	float probability_right = 0.0;
	float probability_left = 0.0;

	Point2f pStraight = Point2f(0, 0);
	Point2f pLeft = Point2f(0, 0);
	Point2f pRight = Point2f(0, 0);

	if(turn == NO_HINT) {
		Sum(straightRight, pStraight, probability_straight, count_straight_right);
		Sum(straightMiddle, pStraight, probability_straight, count_straight_middle);
	}

	if(turn != LEFT_TURN) {
		Sum(turnRightMiddle, pRight, probability_right, count_right);
		Sum(turnRightInnerRight, pRight, probability_right, count_right);
		Sum(turnRightOuterLeft, pRight, probability_right, count_right);
	}

	if(turn != RIGHT_TURN) {
		Sum(turnLeft, pLeft, probability_left, count_Left);
	}

	float correctionRight = 1;
	float correctionLeft = 1;
	float correctionStraight = 1;

	if (probability_straight > 0) {
		if (float(count_Left) / turnLeft.size()
				> float(count_straight_right + count_straight_middle)
						/ (straightRight.size() + straightMiddle.size())) {
			//give matched straight lines a lower weight if we are probably in a left turn
			correctionStraight *= 0.3333;
		}

		if (count_straight_middle == 0 && count_straight_right > 0 && count_Left >= 3) {
			//give matched straight lines a lower weight if we are probably in a left turn
			correctionStraight *= 0.2;
		}
	}

	if((count_straight_right + count_straight_middle + count_Left) > 5) {
		correctionRight = 0;
	} else if((count_straight_right + count_straight_middle + count_Left) > 3) {
		correctionRight /= 3*(count_straight_right + count_straight_middle + count_Left);
	}


	float totalProbability = correctionStraight * probability_straight + correctionRight * probability_right + correctionLeft * probability_left;
	float currentWeight = totalProbability;
	Point2f res = (correctionStraight * pStraight + correctionRight * pRight + correctionLeft * pLeft);

	if (totalProbability > 0) {
		res /= totalProbability;
	} else {
		if (probability_straight == 0) {
			//move point down if no straight detected;
			lastPoint.y += 10;
		}
	}

	if (firstPoint && res.x > 1 && res.y > 1) {
		lastPoint = res;
		firstPoint = false;
	} else if (res.x > 1 && res.y > 1) {
		float sum = lastWeight + currentWeight;
		if (sum > 0) {
			lastPoint = (currentWeight * res + lastWeight * lastPoint) / sum;
		}

		lastWeight = currentWeight;
	} else {
		lastWeight = 0;
	}

	//check broken middle line
	unsigned int  middleMatchSingleCount = 0;
	size_t lastMiddleIndex = -1;
	unsigned int  middleNotBroken = 0;
	for (size_t i = 0; i < turnRightMiddle.size(); i++) {
		if(turnRightMiddle[i].matchedLastTime) {
			if(lastMiddleIndex + 1 == i) {
				middleMatchSingleCount++;
			}
			lastMiddleIndex = i;

			if(middleMatchSingleCount >= 3) {
				middleNotBroken++;
				middleMatchSingleCount = 0;
			}

			if(middleNotBroken >= 2) {
				if(lastResettedFrame == -1) {
					//last reset did not work. try to move the destination point to the left
					lastPoint -= Point2f(10,0);
					break;
				} else if(lastResettedFrame == -2) {
					//last reset did not work. try to move the destination point to the left
					lastPoint -= Point2f(10,0);
					break;
				} else if(lastResettedFrame == -3) {
					//last reset did not work. try to move the destination point to the left
					lastPoint -= Point2f(10,0);
					break;
				} else if(lastResettedFrame == -4) {
					//last reset did not work. try to move the destination point to the left
					lastPoint -= Point2f(LineTemplateGenerator::AVERAGE_LANE_WIDTH,0);
					break;
				} else {
					//do a clean new search in the next frame
					lastResettedFrame = 0;
					firstPoint = true;
					break;
				}

			}
		} else {
			middleMatchSingleCount = 0;
		}
	}

	//limit last  point
	if(lastPoint.x > BINARY_IMAGE_WIDTH) {
		lastPoint.x = BINARY_IMAGE_WIDTH;
	} else if (lastPoint.x < 0) {
		lastPoint.x = 0;
	}

	if(lastPoint.y > BINARY_IMAGE_HEIGHT) {
		lastPoint.y = BINARY_IMAGE_HEIGHT;
	} else if (lastPoint.y < 0) {
		lastPoint.y = 0;
	}

	if (debugEnabled) {
		line(out, Point(0, 100), Point(10, 100), Scalar(0, 255, 0));
		line(out, Point(0, 200), Point(10, 200), Scalar(0, 255, 0));
		line(out, Point(0, 300), Point(10, 300), Scalar(0, 255, 0));
		line(out, Point(100, 0), Point(100, 10), Scalar(0, 255, 0));
		line(out, Point(200, 0), Point(200, 10), Scalar(0, 255, 0));
		circle(out, lastPoint, 2.0, Scalar(0, 255, 0), 1, 8);
	}

	*dest = lastPoint;
	int div = count_Left + count_right;
	*probability = 2.0*lastWeight / (div > 1 ? div : 1); // average probability 0.5 so multiply weight with 2

	if(*probability > 1) {
		*probability = 1;
	}
}

void LineSpecifier::Sum(std::vector<LineTemplate> &temps, Point2f &result, float &totalProbability, int &matchCount) {
	for (unsigned int i = 0; i < temps.size(); i++) {
		if (temps[i].probability > 0) {
			result += temps[i].GetAbsoluteLaneLocation() * temps[i].probability;
			totalProbability += temps[i].probability;
			matchCount++;
		}
	}
}
