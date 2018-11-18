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
 * $Author:: schoen $   $Date:: 2016-02-04 #$
 **********************************************************************/

#include "LineDetectorHorizontal.h"
#include "ImageProcessingUtils.h"

const int LineDetector_Horizontal::MAX_WRONG_PIXEL_HORIZONTAL = 4  * 255 * 255;
const int LineDetector_Horizontal::STEP_SIZE_HORIZONTAL_LINE = 7;
const int LineDetector_Horizontal::STOP_SEARCH_HORIZONTAL = 20;
const int LineDetector_Horizontal::INITIAL_REGION_HEIGHT = 100;
const int LineDetector_Horizontal::INITIAL_STEP_INCREASE = 3;

LineDetector_Horizontal::LineDetector_Horizontal(int marginTop, int lineHeight, int marginBottom) {
	binaryImageSize = Rect(0,0, BINARY_IMAGE_WIDTH, BINARY_IMAGE_HEIGHT);
	debugColor = Scalar(0, 100, 255);

	horizontalLine = Mat::zeros(marginTop + lineHeight + marginBottom, 3, CV_8UC1);
	rectangle(horizontalLine, Point(0,marginTop), Point(2, marginTop + lineHeight - 1), Scalar(255), CV_FILLED);
	horizontalLineCenter = Point2f(1, marginTop + (lineHeight - 1) *0.5);

	minHeight = marginTop + marginBottom + horizontalLine.rows;

	stopStartSearchAtX = STOP_SEARCH_HORIZONTAL;
	stopEndSearchAtX = STOP_SEARCH_HORIZONTAL;
	m_width = horizontalLine.cols;
	m_height = INITIAL_REGION_HEIGHT;
	stepIncrease = INITIAL_STEP_INCREASE;

	splitted.horizontalPatch = Mat::ones(3, lineHeight, CV_8UC1) * 255;
	splitted.BLACK_COUNT = (3*lineHeight - 1) * 255;
	splitted.horizontalLineCenter = Point2f((lineHeight - 1) *0.5, 1);
	splitted.maxSplitHeight = marginBottom + lineHeight + marginTop;

	estimatedLine.reserve(20);
}

LineDetector_Horizontal::~LineDetector_Horizontal() {
}

CVMath::LineSegment *LineDetector_Horizontal::Detect_Right(Mat &input, Mat *debugImage, int xStart, int yStart, CVMath::LineSegment * segment) {
	Rect region = Rect(xStart, yStart - m_height / 2, m_width, m_height);
	region &= binaryImageSize;

	int width = this->m_width;
	int height = this->m_height;

	estimatedLine.clear();

	Point2f step(STEP_SIZE_HORIZONTAL_LINE, 0);
	for (int i = 0; estimatedLine.size() < 20; i++) {
		Mat result;
		double maxVal = 0, minVal = 0;
		Point minLoc, maxLoc;

		if (region.width < horizontalLine.cols || region.height < horizontalLine.rows) {
			break;
		}

		Mat image = input(region);

		matchTemplate(image, horizontalLine, result, CV_TM_SQDIFF);

		minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);

		if (minVal <= MAX_WRONG_PIXEL_HORIZONTAL) {
			Point2f p = Point2f(minLoc) + Point2f(region.x, region.y) + horizontalLineCenter;

			region = Step(p, region, step, height);
		} else {
			region = FailStep(region, step, width, height);

			if(estimatedLine.size() > 0) {
				if (region.x - estimatedLine.back().x > stopEndSearchAtX) {
					break;
				}
			} else if (region.x - xStart > stopStartSearchAtX) {
				break;
			}
		}

		region &= binaryImageSize;
	}

	return FitLine(debugImage, segment);
}


CVMath::LineSegment *LineDetector_Horizontal::DetectSplitted_Right(Mat &input, Mat *debugImage, int xStart, int yStart, CVMath::LineSegment * segment) {
	Rect region = Rect(xStart, yStart - m_height / 2, m_width, m_height);
	region &= binaryImageSize;

	estimatedLine.clear();

	int width = this->m_width;
	int height = this->m_height;

	Point2f step(STEP_SIZE_HORIZONTAL_LINE, 0);
	for (int i = 0; estimatedLine.size() < 20; i++) {
		if (region.width < splitted.horizontalPatch.cols || region.height < splitted.horizontalPatch.rows) {
			break;
		}

		Mat result;
		double maxVal = 0, minVal = 0;
		Point minLoc, maxLoc;

		Mat image = input(region);

		matchTemplate(image, splitted.horizontalPatch, result, CV_TM_SQDIFF);

		minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);

		bool condition = false;
		if (minVal <= 1 * 255) {
			int count = splitted.maxSplitHeight;

			for(int i = minLoc.y - 1; i >=0; i--) {
				if(result.at<float>(i) > splitted.BLACK_COUNT) {
					count -= minLoc.y - i;
					break;
				}
			}
			for(int i = minLoc.y + 1; i < result.cols; i++) {
				if(result.at<float>(i) > splitted.BLACK_COUNT) {
					count -= i - minLoc.y;
					break;
				}
			}
			if(count >= 0) {
				condition = true;
			}
		}

		if(condition) {
			Point2f p = Point2f(minLoc) + Point2f(region.x, region.y) + splitted.horizontalLineCenter;
			region = Step(p, region, step, height);
		} else {
			region = FailStep(region, step, width, height);

			if(estimatedLine.size() > 0) {
				if (region.x - estimatedLine.back().x > stopEndSearchAtX) {
					break;
				}
			} else if (region.x - xStart > stopStartSearchAtX) {
				break;
			}
		}

		region &= binaryImageSize;
	}

	return FitLine(debugImage, segment);
}

Rect LineDetector_Horizontal::Step(Point2f p, Rect region, Point2f &step, int &height) {
	if (estimatedLine.size() > 1) {
		Point2f last = estimatedLine.back();
		step.x += stepIncrease;
		step.y = p.y - last.y;
	} else if (estimatedLine.size() > 0) {
		Point2f last = estimatedLine.back();
		step.x += stepIncrease;
		step.y = p.y - last.y;

		height >>= 1;
		if (height < minHeight) {
			height = minHeight;
		}
	} else {
		height >>= 1;
		if (height < minHeight) {
			height = minHeight;
		}

		step.x += stepIncrease;
	}
	estimatedLine.push_back(p);

	return Rect(region.x + step.x, p.y - (height >> 1) + step.y, m_width, height);
}

Rect LineDetector_Horizontal::FailStep(Rect region, Point2f &step, int &width, int &height) {
	height += abs(2*step.y);

	//TODO: copy code from LineDetector_Vertical::FailStep
	//TODO: fix bug never ending loop
	/*
	size_t listSize = estimatedLine.size();
	if(listSize > 2) {
		Point2f p0 = estimatedLine.back();
		Point2f p2 = estimatedLine.at(listSize-3);

		step.y = p0.y - p2.y;
		step.x = (step.x /2);
	}
	*/

	return Rect(region.x + step.x, region.y - step.y, width, height);
}


CVMath::LineSegment * LineDetector_Horizontal::FitLine(Mat *debugImage, CVMath::LineSegment * segment) {
	Vec4f temporary;
	size_t listSize = estimatedLine.size();
	if (listSize > 2) {
		fitLine(estimatedLine, temporary, CV_DIST_L1, 0, 1, 1);
		*segment = CVMath::LineSegment(Point2f(temporary[2], temporary[3]), Point2f(temporary[0], temporary[1]));

		segment->CropX(estimatedLine.front().x, estimatedLine.back().x);
	} else if (listSize < 2) {
		return 0;
	} else {
		segment->SetStartEnd(estimatedLine.front(), estimatedLine.back());
	}

	if (segment->direction.x < 0) {
		segment->direction = -segment->direction;
	}

	if (debugImage) {
		for (size_t i = 0; i < estimatedLine.size(); i++) {
			Point2f p = estimatedLine[i];

			cv::circle(*debugImage, p, 3.0, Scalar(255, 100, 0), 2, 8);
		}
	}

	return segment;
}
