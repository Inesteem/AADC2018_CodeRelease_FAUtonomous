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

#include "LineDetectorVertical.h"
#include "ImageProcessingUtils.h"

const int LineDetector_Vertical::MAX_WRONG_PIXEL_VERTICAL = 8 * 255 * 255;
const int LineDetector_Vertical::STEP_SIZE_VERTICAL_LINE = 8;
const int LineDetector_Vertical::STOP_SEARCH_VERTICAL = 20;
const int LineDetector_Vertical::INITIAL_REGION_WIDTH = 100;
const int LineDetector_Vertical::INITIAL_STEP_INCREASE = 3;

LineDetector_Vertical::LineDetector_Vertical(const int marginLeft, const int lineWidth, const int marginRight) {
	binaryImageSize = Rect(0, 0, BINARY_IMAGE_WIDTH, BINARY_IMAGE_HEIGHT);
	debugColor = Scalar(255, 100, 0);

	verticalLine = Mat::zeros(3, marginLeft + lineWidth + marginRight, CV_8UC1);
	rectangle(verticalLine, Point(marginLeft, 0), Point(marginLeft + lineWidth - 1, 2), Scalar(255), CV_FILLED);
	verticalLineCenter = Point2f(marginLeft + (lineWidth - 1) *0.5, 1);

	minWidth = verticalLine.cols + marginLeft + marginRight;

	stopStartSearchAtY = STOP_SEARCH_VERTICAL;
	stopEndSearchAtY = STOP_SEARCH_VERTICAL;
	m_width = INITIAL_REGION_WIDTH;
	m_height = verticalLine.rows;
	stepIncrease = INITIAL_STEP_INCREASE;

	splitted.verticalPatch = Mat::ones(3, lineWidth, CV_8UC1) * 255;
	splitted.BLACK_COUNT = (3*lineWidth - 1) * 255;
	splitted.verticalLineCenter = Point2f((lineWidth - 1) *0.5, 1);
	splitted.maxSplitWidth = marginLeft + lineWidth + marginRight;

	estimatedLine.reserve(20);
}

LineDetector_Vertical::~LineDetector_Vertical() {
}

CVMath::LineSegment * LineDetector_Vertical::Detect_Up(Mat &input, Mat *debugImage, int xStart, int yStart, CVMath::LineSegment * segment) {
	Rect region = Rect(xStart - (m_width>>1), yStart, m_width, m_height);
	region &= binaryImageSize;

	int width = this->m_width;
	int height = this->m_height;

	estimatedLine.clear();

	Point2f step(0, -STEP_SIZE_VERTICAL_LINE);
	for (int i = 0; estimatedLine.size() < 20; i++) {
		if (region.width < verticalLine.cols || region.height < verticalLine.rows) {
			break;
		}

		Mat result;
		double maxVal = 0, minVal = 0;
		Point minLoc, maxLoc;
		Mat image = input(region);

		matchTemplate(image, verticalLine, result, CV_TM_SQDIFF);

		minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);

		if (minVal <= MAX_WRONG_PIXEL_VERTICAL) {
			Point2f p = Point2f(minLoc) + Point2f(region.x, region.y) + verticalLineCenter;

			region = Step(p, region, step, width);
		} else {
			region = FailStep(region, step, width, height);

			if(estimatedLine.size() > 0) {
				if (estimatedLine.back().y - region.y > stopEndSearchAtY) {
					break;
				}
			} else {
				if (yStart - region.y > stopStartSearchAtY) {
					break;
				}
			}
		}

		region &= binaryImageSize;
	}

	return FitLine(debugImage, segment);
}

CVMath::LineSegment *LineDetector_Vertical::DetectSplitted_Up(Mat &input, Mat *debugImage, int xStart, int yStart,
		CVMath::LineSegment * segment) {
	Rect region = Rect(xStart - (m_width >> 1), yStart, m_width, m_height);
	region &= binaryImageSize;

	estimatedLine.clear();

	int width = this->m_width;
	int height = this->m_height;

	Point2f step(0, -STEP_SIZE_VERTICAL_LINE);
	for (int i = 0; estimatedLine.size() < 20; i++) {
		if (region.width < splitted.verticalPatch.cols || region.height < splitted.verticalPatch.rows) {
			break;
		}

		Mat result;
		double maxVal = 0, minVal = 0;
		Point minLoc, maxLoc;
		Mat image = input(region);

		matchTemplate(image, splitted.verticalPatch, result, CV_TM_SQDIFF);

		minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);

		bool condition = false;
		if (minVal <= 1 * 255) {
			int count = splitted.maxSplitWidth;

			for(int i = minLoc.x - 1; i >=0; i--) {
				if(result.at<float>(i) > splitted.BLACK_COUNT) {
					count -= minLoc.x - i;
					break;
				}
			}
			for(int i = minLoc.x + 1; i < result.cols; i++) {
				if(result.at<float>(i) > splitted.BLACK_COUNT) {
					count -= i - minLoc.x;
					break;
				}
			}
			if(count >= 0) {
				condition = true;
			}
		}

		if(condition) {
			Point2f p = Point2f(minLoc) + Point2f(region.x, region.y) + splitted.verticalLineCenter;

			region = Step(p, region, step, height);
		} else {
			region = FailStep(region, step, width, height);

			if (estimatedLine.size() > 0) {
				if (estimatedLine.back().y - region.y > stopEndSearchAtY) {
					break;
				}
			} else if (yStart - region.y > stopStartSearchAtY) {
				break;
			}
		}

		region &= binaryImageSize;
	}

	return FitLine(debugImage, segment);
}

inline Rect LineDetector_Vertical::Step(Point2f p, Rect region, Point2f& step, int& width) {
	if (estimatedLine.size() > 1) {
		Point2f last = estimatedLine.back();
		step.y -= stepIncrease;
		step.x = p.x - last.x;
	} else if (estimatedLine.size() > 0) {
		Point2f last = estimatedLine.back();
		step.y -= stepIncrease;
		step.x = p.x - last.x;

		width >>= 1;
		if (width < minWidth) {
			width = minWidth;
		}
	} else {
		width >>= 1;
		if (width < minWidth) {
			width = minWidth;
		}
		step.y -= stepIncrease;
	}
	estimatedLine.push_back(p);

	return Rect(p.x - (width >> 1) + step.x, region.y + step.y, width, m_height);
}

inline Rect LineDetector_Vertical::FailStep(Rect region, Point2f& step, int& width, int& height) {
	width += abs(2*step.x);

	size_t listSize = estimatedLine.size();
	if(listSize > 2) {
		Point2f p0 = estimatedLine.back();
		Point2f p2 = estimatedLine.at(listSize-3);

		step.y = (step.y /2);
		step.x = p0.x - p2.x;
	}

	return Rect(region.x - step.x, region.y + step.y, width, height);
}

inline CVMath::LineSegment* LineDetector_Vertical::FitLine(Mat* debugImage, CVMath::LineSegment* segment) {
	Vec4f temporary;
	size_t listSize = estimatedLine.size();
	if (listSize > 2) {
		fitLine(estimatedLine, temporary, CV_DIST_L1, 0, 1, 1);
		*segment = CVMath::LineSegment(Point2f(temporary[2], temporary[3]), Point2f(temporary[0], temporary[1]));

		segment->CropY(estimatedLine.front().y, estimatedLine.back().y);
	} else if (listSize < 2) {
		return 0;
	} else {
		segment->SetStartEnd(estimatedLine.front(), estimatedLine.back());
	}

	if (segment->direction.y > 0) {
		segment->direction = -segment->direction;
	}

	if (debugImage) {
		for (size_t i = 0; i < estimatedLine.size(); i++) {
			Point2f p = estimatedLine[i];

			cv::circle(*debugImage, p, 3.0, debugColor, 2, 8);
		}
	}

	return segment;
}
