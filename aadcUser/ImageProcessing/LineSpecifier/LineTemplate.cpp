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
 * $Author:: eduard schoen $   $Date:: 2015-12-14 #$
 **********************************************************************/

#include "LineTemplate.h"
#include "LineTemplateGenerator.h"

const int LineTemplate::DEFAULT_MAX_WRONG_PIXELS = 32;

bool LineTemplate::Match(cv::Mat in, Mat *debugImage, Scalar color, cv::Point2f *lastDestination, float lastDestinationWeight) {
	Mat result;
	double maxVal = 0, minVal = 0;
	Point minLoc, maxLoc;
	Rect currentROI = roi;

	//reset probability
	probability = 0;

	if(lastDestination) {
		//get lane location, see GetAbsoluteLaneLocation()
		Point2f absLaneLocation = Point2f(roi.x + (roi.width >> 1) - (image.cols >> 1), roi.y) + lineCenter + laneLocation;
		float xLaneLocDiff = lastDestinationWeight * (lastDestination->x - absLaneLocation.x);

		Rect movedROI = roi;
		movedROI.x += xLaneLocDiff;
		movedROI &= LineTemplateGenerator::bitmaskSize;

		if(movedROI.width < image.cols) {
			matchedLastTime = false;
			return matchedLastTime;
		} else {
			currentROI = movedROI;
		}
	}

	in = in(currentROI);

	matchTemplate(in, image, result, CV_TM_SQDIFF);

	minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);
	minLoc += Point(currentROI.x, currentROI.y);

	matchLocation = Point2f(minLoc);
	float max = maxWrongPixels * 255 * 255;
	if(minVal < max) {
		probability = (max - minVal ) / max * probabilityScalar;
		PrintRectangle(color, currentROI, debugImage);
		matchedLastTime = true;
	} else {
		matchedLastTime = false;
	}

	return matchedLastTime;
}

void LineTemplate::AdjustLaneWidth(float halfWidth) {
	float length = norm(laneLocation);
	laneLocation *= halfWidth * laneWidthScalar/ length;
}

void LineTemplate::PrintRectangle(Scalar color, Rect roi, Mat *debugImage) {
	if(debugImage == 0) {
		return;
	}

	int thickness = probability * 5.0;
	cv::rectangle(*debugImage, matchLocation, Point(matchLocation.x + image.cols, matchLocation.y + image.rows), color,
			thickness, 8, 0);
	cv::line(*debugImage, Point(roi.x, roi.y), Point(roi.x, roi.y + roi.height), color);
	cv::line(*debugImage, Point(roi.x + roi.width, roi.y),
			Point(roi.x + roi.width, roi.y + roi.height), color);
}
