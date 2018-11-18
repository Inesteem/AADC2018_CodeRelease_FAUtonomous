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
 * $Author:: schoen $   $Date:: 2016-02-02 #$
 **********************************************************************/

#ifndef PARKING_PARKINGSPOTDETECTOR_H_
#define PARKING_PARKINGSPOTDETECTOR_H_

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#include "CVMath.h"
#include "ImageProcessingUtils.h"
#include "../lines/LineDetectorVertical.h"
#include "../lines/LineDetectorHorizontal.h"

using namespace cv;
using namespace std;

class ParkingSpotDetector {
	LineDetector_Vertical vDetector;
	LineDetector_Horizontal hDetector;
	LineDetector_Horizontal hDetectorTrack;

	LineDetector_Horizontal hDetectorTransSecondLine;

	bool debugEnabled;
public:
	static const int PARKING_SPOT_LENGTH_LONG;
	static const int PARKING_SPOT_LENGTH_TRANS;
	static const int PARKING_SPOT_LENGTH_TOLERANCE;
	static const int DISTANCE_THICK_THIN_LINE_CROSSING;


	struct ParkingSpot {
	public:
		bool valid;
		bool typeLong;
		float parkingSpotLength;
		CVMath::Line verticalLine;
		Point2f locationOnVerticalLine;

		ParkingSpot() {
			valid = false;
			typeLong = false;
			parkingSpotLength = 0;
		}

		bool IsTypeLong() {
			return typeLong;
		}

		bool IsTypeTrans() {
			return (typeLong == false);
		}
	};

	ParkingSpotDetector();
	virtual ~ParkingSpotDetector();

	void switchDebugMode(bool enable) {
		debugEnabled = enable;
	}

	ParkingSpot Detect(Mat &image, Mat *debugImage, int type, bool isFirstDetection);
	bool IsLongitudinalParkingSpot(Mat &image, Mat *debugImage);
	bool IsTransversalParkingSpot(Mat &image, Mat *debugImage, Point2f *goal, Point2f *direction);

private:
	Point2f FindHorizontalLineStartPoint(Mat &image, Mat *debugImage, CVMath::LineSegment vert, int yOffset);
	Point2f FindStartPoint_SecondLine(Mat &image, Mat *debugImage, CVMath::LineSegment vert, int yOffset);
};

#endif /* PARKING_PARKINGSPOTDETECTOR_H_ */
