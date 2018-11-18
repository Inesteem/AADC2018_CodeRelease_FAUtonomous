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

#ifndef LINES_LINEDETECTORVERTICAL_H_
#define LINES_LINEDETECTORVERTICAL_H_

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iostream>

#include "CVMath.h"

using namespace cv;
using namespace std;

class LineDetector_Vertical {
	Rect binaryImageSize;

	Mat verticalLine;
	Point2f verticalLineCenter;

	vector<Point2f> estimatedLine;

	int minWidth;

	int stopStartSearchAtY;
	int stopEndSearchAtY;
	int m_width;
	int m_height;
	int stepIncrease;

	Scalar debugColor;

	struct Splitted {
		int BLACK_COUNT;
		Point2f verticalLineCenter;
		Mat verticalPatch;
		int maxSplitWidth;
	} splitted;

public:
	static const int STEP_SIZE_VERTICAL_LINE;
	static const int MAX_WRONG_PIXEL_VERTICAL;
	static const int STOP_SEARCH_VERTICAL;
	static const int INITIAL_REGION_WIDTH;
	static const int INITIAL_STEP_INCREASE;

	LineDetector_Vertical(const int marginLeft = 3, const int lineWidth = 6, const int marginRight = 3);
	virtual ~LineDetector_Vertical();

	void SetParameter(int stopStartSearchAtY, int stopEndSearchAtY, int width, int stepIncrease) {
		this->stopStartSearchAtY = stopStartSearchAtY;
		this->stopEndSearchAtY = stopEndSearchAtY;
		this->m_height = verticalLine.rows;
		if(width < minWidth) {
			this->m_width = minWidth;
		} else {
			this->m_width = width;
		}
		this->stepIncrease = stepIncrease;
	}

	CVMath::LineSegment *Detect_Up(Mat &input, Mat *debugImage, int xStart, int yStart, CVMath::LineSegment * segment);

	CVMath::LineSegment *DetectSplitted_Up(Mat &input, Mat *debugImage, int xStart, int yStart, CVMath::LineSegment * segment);

	void SetColor(Scalar color) {
		debugColor = color;
	}

private:
	inline Rect Step(Point2f p, Rect region, Point2f &step, int &width);
	inline Rect FailStep(Rect region, Point2f &step, int &width, int &height);
	inline CVMath::LineSegment *FitLine(Mat *debugImage, CVMath::LineSegment * segment);
};

#endif /* LINES_LINEDETECTORVERTICAL_H_ */
