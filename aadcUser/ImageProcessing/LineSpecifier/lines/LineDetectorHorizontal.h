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

#ifndef LINES_LINEDETECTORHORIZONTAL_H_
#define LINES_LINEDETECTORHORIZONTAL_H_

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#include "CVMath.h"

using namespace cv;
using namespace std;

class LineDetector_Horizontal {
	Rect binaryImageSize;

	Mat horizontalLine;
	Point2f horizontalLineCenter;

	vector<Point2f> estimatedLine;

	int minHeight;

	int stopStartSearchAtX;
	int stopEndSearchAtX;
	int m_width;
	int m_height;
	int stepIncrease;

	Scalar debugColor;

	struct Splitted {
		int BLACK_COUNT;
		Point2f horizontalLineCenter;
		Mat horizontalPatch;
		int maxSplitHeight;
	} splitted;

public:
	static const int STEP_SIZE_HORIZONTAL_LINE;
	static const int MAX_WRONG_PIXEL_HORIZONTAL;
	static const int STOP_SEARCH_HORIZONTAL;
	static const int INITIAL_REGION_HEIGHT;
	static const int INITIAL_STEP_INCREASE;

	LineDetector_Horizontal(int marginTop = 5, int lineHeight = 4, int marginBottom = 0);
	virtual ~LineDetector_Horizontal();

	void SetParameter(int stopStartSearchAtX, int stopEndSearchAtX, int height, int stepIncrease) {
		this->stopStartSearchAtX = stopStartSearchAtX;
		this->stopEndSearchAtX = stopEndSearchAtX;
		this->m_width = horizontalLine.cols;
		if(height < minHeight) {
			this->m_height = minHeight;
		} else {
			this->m_height = height;
		}
		this->stepIncrease = stepIncrease;
	}

	CVMath::LineSegment *Detect_Right(Mat &input, Mat *debugImage, int xStart, int yStart, CVMath::LineSegment * segment);

	CVMath::LineSegment *DetectSplitted_Right(Mat &input, Mat *debugImage, int xStart, int yStart, CVMath::LineSegment * segment);

	CVMath::LineSegment *DetectSplitted_Left(Mat &input, Mat *debugImage, int xStart, int yStart, CVMath::LineSegment * segment);

private:
	inline Rect Step(Point2f p, Rect region, Point2f &step, int &height);
	inline Rect FailStep(Rect region, Point2f &step, int &width, int &height);
	inline CVMath::LineSegment *FitLine(Mat *debugImage, CVMath::LineSegment * segment);
};

#endif /* LINES_LINEDETECTORHORIZONTAL_H_ */
