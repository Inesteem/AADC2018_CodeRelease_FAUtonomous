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

#ifndef LINETEMPLATE_H_
#define LINETEMPLATE_H_

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

class LineTemplate {
public:
	static const int DEFAULT_MAX_WRONG_PIXELS;

	int maxWrongPixels;

	/**
	 * template image, which is searched
	 */
	cv::Mat image;

	/**
	 * where do we look for template occurrences
	 */
	cv::Rect roi;

	/**
	 * line center of the template. Coordinates start at the top-left corner. The origin (0,0) is in the center of the pixel.
	 */
	cv::Point2f lineCenter;

	/**
	 * defines the location of the center of the lane. Measured from lineCenter.
	 */
	cv::Point2f laneLocation;

	/**
	 * if AdjustLaneWidth is called, this value describes a scale factor, the laneWidth differs for this template
	 */
	float laneWidthScalar;

	float probabilityScalar;
	/**
	 * probability that the template matches the found location. Not normed to 1. This is a temporary value.
	 */
	float probability;

	/**
	 * last match location
	 */
	cv::Point2f matchLocation;

	bool matchedLastTime;

	LineTemplate() {
		maxWrongPixels = DEFAULT_MAX_WRONG_PIXELS;
		image = Mat::zeros(16, 32, CV_8UC1);
		roi = cv::Rect(0, 0, 32, 16);
		lineCenter = cv::Point2f(0, 0);
		laneLocation = cv::Point2f(0, 0);
		laneWidthScalar = 1.0;
		probabilityScalar = 1.0;
		probability = 0;
		matchLocation = cv::Point(0, 0);
		matchedLastTime = false;
	}

	LineTemplate(Mat image, int maxWrongPixel, float probabilityScalar) {
		this->maxWrongPixels = maxWrongPixel;
		this->image = image;
		roi = cv::Rect(0, 0, 32, 16);
		lineCenter = cv::Point2f(0, 0);
		laneLocation = cv::Point2f(0, 0);
		laneWidthScalar = 1.0;
		this->probabilityScalar = probabilityScalar;
		probability = 0;
		matchLocation = cv::Point(0, 0);
		matchedLastTime = false;
	}

	LineTemplate (const LineTemplate& rhs) {
		maxWrongPixels = rhs.maxWrongPixels;
		image = rhs.image.clone();
		roi = rhs.roi;
		lineCenter = rhs.lineCenter;
		laneLocation = rhs.laneLocation;
		laneWidthScalar = rhs.laneWidthScalar;
		probabilityScalar = rhs.probabilityScalar;
		probability = rhs.probability;
		matchLocation = rhs.matchLocation;
		matchedLastTime = rhs.matchedLastTime;
	}

	virtual ~LineTemplate() {
	}

	bool Match(cv::Mat in, Mat *debugImage, Scalar color, cv::Point2f *lastDestination, float lastDestinationWeight = 1);
	void AdjustLaneWidth(float halfWidth);
	cv::Point2f GetAbsoluteLaneLocation() {
		return matchLocation + lineCenter + laneLocation;
	}

	void PrintRectangle(Scalar color, Rect roi, Mat *debugImage);
};

#endif /* LINETEMPLATE_H_ */
