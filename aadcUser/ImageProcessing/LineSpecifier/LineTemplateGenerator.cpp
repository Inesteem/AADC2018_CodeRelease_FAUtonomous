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
 * $Author:: schoen $   $Date:: 2015-12-15 #$
 **********************************************************************/

#include "LineTemplateGenerator.h"
#include <cmath>
#include <opencv2/opencv.hpp>

const int LineTemplateGenerator::METER_TO_PIXEL = 200 * 0.995;
const float LineTemplateGenerator::AVERAGE_LANE_WIDTH = (0.44 + 0.02 / 2 + 0.03 / 2)
		* LineTemplateGenerator::METER_TO_PIXEL;
const int LineTemplateGenerator::DEFAULT_TEMPLATE_HEIGHT = 16;

const Rect LineTemplateGenerator::bitmaskSize = Rect(0, 0, 300, 300);
const Scalar LineTemplateGenerator::DEFAULT_COLOR_VALUE = Scalar(255);

using namespace cv;

LineTemplateGenerator::LineTemplateGenerator() {

}

LineTemplateGenerator::~LineTemplateGenerator() {
}

void LineTemplateGenerator::Generate(std::vector<LineTemplate> &lineTemplates, LineTemplate first, int startIndex,
		int elementCount, float startAngle, float radius, int lineWidth, TURN turn, SIDE side, float averageLaneWidth, float lineLocationY_Offset) {
	lineTemplates.resize(startIndex + elementCount);

	float angle = startAngle;
	float radiusInPixel = radius * METER_TO_PIXEL;
	Point2f circleDirection(cos(angle / 180 * CV_PI), sin(angle / 180 * CV_PI));
	Point2f circleCenter = first.lineCenter + circleDirection * radiusInPixel;
	Point2f topLeftCorner(0, 0);
	float x = 0, y = 0;

	//determine position of the lane center depending on line center, left or right
	if (turn == RIGHT) {
		if (side == LINE_ON_RIGHT_SIDE) {
			first.laneLocation = Point2f(
					IntersectCircle(circleCenter, radius * METER_TO_PIXEL + averageLaneWidth / 2.0, 8 + lineLocationY_Offset, turn), 8 + lineLocationY_Offset);
		} else {
			first.laneLocation = Point2f(
					IntersectCircle(circleCenter, radius * METER_TO_PIXEL - averageLaneWidth / 2.0, 8 + lineLocationY_Offset, turn), 8 + lineLocationY_Offset);
		}
	}

	if (turn == LEFT) {
		if (side == LINE_ON_RIGHT_SIDE) {
			first.laneLocation = Point2f(
					IntersectCircle(circleCenter, radius * METER_TO_PIXEL - averageLaneWidth / 2.0, 8 + lineLocationY_Offset, turn), 8 + lineLocationY_Offset);
		} else {
			first.laneLocation = Point2f(
					IntersectCircle(circleCenter, radius * METER_TO_PIXEL + averageLaneWidth / 2.0, 8 + lineLocationY_Offset, turn), 8 + lineLocationY_Offset);
		}
	}

	LineTemplate * old, *next;
	old = &first;

	for (int i = startIndex; i < elementCount + startIndex; i++) {
		//determine next template
		next = &lineTemplates[i];

		radiusInPixel = radius * METER_TO_PIXEL;
		y = first.lineCenter.y - DEFAULT_TEMPLATE_HEIGHT;
		x = IntersectCircle(circleCenter, radiusInPixel, y, turn);

		next->lineCenter = first.lineCenter;
		topLeftCorner = Point2f(x, y) - next->lineCenter;

		if (i == startIndex) {
			next->roi = first.roi;
		} else {
			next->roi = Rect(old->roi.x + int(topLeftCorner.x), old->roi.y + int(topLeftCorner.y), old->roi.width,
					old->roi.height);
		}
		//limit region of interest to the maximum
		next->roi = next->roi & bitmaskSize;


		//move circle center
		if (i != startIndex) {
			circleCenter -= topLeftCorner;
		}
		radiusInPixel = radius * METER_TO_PIXEL;

		circle(next->image, circleCenter, radiusInPixel, DEFAULT_COLOR_VALUE, lineWidth);

		//lane location is at the same point in absolute coordinates.
		if (i == startIndex) {
			next->laneLocation = old->laneLocation - old->lineCenter;
		} else {
			next->laneLocation = old->laneLocation - topLeftCorner;
		}
		next->laneWidthScalar = norm(next->lineCenter - next->laneLocation) * 2 / averageLaneWidth;

		old = next;
	}
}

void LineTemplateGenerator::GenerateStraight(std::vector<LineTemplate>& lineTemplates, LineTemplate first, Point2f offset,
		int startIndex, int elementCount, int lineWidth, SIDE side) {
	lineTemplates.resize(startIndex + elementCount);

	Point2f topLeftCorner(0, -DEFAULT_TEMPLATE_HEIGHT);
	LineTemplate * old, *next;

	if (side == LINE_ON_LEFT_SIDE) {
		first.laneLocation = Point2f (AVERAGE_LANE_WIDTH / 2.0, 0);
	} else {
		first.laneLocation = Point2f (-AVERAGE_LANE_WIDTH / 2.0, 0);
	}

	old = &first;
	for (int i = startIndex; i < elementCount + startIndex; i++) {
		next = &lineTemplates[i];

		if(i == startIndex) {
			next->roi = first.roi;
		} else {
			next->roi = Rect(old->roi.x + int(topLeftCorner.x + offset.x), old->roi.y + int(topLeftCorner.y + offset.y), old->roi.width,
								old->roi.height);
		}
		//limit region of interest to the maximum
		next->roi = next->roi & bitmaskSize;

		next->lineCenter = first.lineCenter;
		next->laneLocation = first.laneLocation;
		next->laneWidthScalar = 1.0;

		if(lineWidth % 2 == 0) {
			for(int i = -lineWidth / 2; i < lineWidth /2; i++) {
				line(next->image, next->lineCenter + Point2f(i + 0.51, -DEFAULT_TEMPLATE_HEIGHT), next->lineCenter + Point2f(i + 0.51, DEFAULT_TEMPLATE_HEIGHT), DEFAULT_COLOR_VALUE, 1);
			}
		} else {
			for(int i = -lineWidth / 2; i < lineWidth /2 + 1; i++) {
				line(next->image, next->lineCenter + Point2f(i + 0.01, -DEFAULT_TEMPLATE_HEIGHT), next->lineCenter + Point2f(i + 0.01, DEFAULT_TEMPLATE_HEIGHT), DEFAULT_COLOR_VALUE, 1);
			}
		}

		old = next;
	}
}

float LineTemplateGenerator::IntersectCircle(cv::Point2f center, float radius, float line_y, TURN turn) {
	if (pow(radius, 2) < cv::pow(center.y - line_y, 2)) {
		throw cv::Exception(0, "LineSpecifier: Cannot extract root of a negative number", "", __FILE__, __LINE__);
	}

	float x = sqrt(pow(radius, 2) - pow(center.y - line_y, 2));
	if (turn == RIGHT) {
		return -x + center.x;
	} else if (turn == LEFT) {
		return x + center.x;
	}

	return x;
}
