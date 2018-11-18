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

#ifndef LINESPECIFIER_H_
#define LINESPECIFIER_H_

#include "LineTemplate.h"
#include <opencv2/opencv.hpp>
#include "CVMath.h"

using namespace cv;
using namespace std;

class LineSpecifier {
	/**
	 * point in image coordinates
	 */
	cv::Point2f lastPoint;

	/**
	 * a value which describes how many pixels matched last time
	 */
	float lastWeight;

	bool debugEnabled;

	bool firstPoint;

	/**
	 * 0 is the current frame, -1 is the previous frame
	 */
	int lastResettedFrame;

	float lastLaneWidth;

	std::vector<LineTemplate> straightMiddle;
	std::vector<LineTemplate> straightRight;

	std::vector<LineTemplate> turnRightMiddle;

	std::vector<LineTemplate> turnRightInnerRight;

	std::vector<LineTemplate> turnRightOuterLeft;

	std::vector<LineTemplate> turnLeft;

public:
	enum HINT {
		NO_HINT,
		LEFT_TURN,
		RIGHT_TURN,
	};

	LineSpecifier();
	void Init();
	void InitStraight();
	void InitRightTurn();
	void InitLeftTurn();

	void SetFirstPoint(bool value) {
		firstPoint = value;
		lastPoint = cv::Point2f(150, 300);
	}

	Point2f GetLastDestination() {
		return lastPoint;
	}

	void MoveDestPoint(Point2f movement) {
		lastPoint += movement;
	}

	void SetLastDestination(Point2f value, float weight) {
		lastPoint = value * weight + lastPoint * (1 - weight);
	}

	virtual ~LineSpecifier();

	void DetermineDestinationPoint (cv::Mat &in, cv::Mat &debugOut, cv::Point2f * dest, float * probability, HINT turn = NO_HINT);

	void switchDebugMode(bool enable) {
		debugEnabled = enable;
	}
private:
	void inline Sum(std::vector<LineTemplate> &temps, Point2f &result, float &totalProbability, int &matchCount);
};

#endif /* LINESPECIFIER_H_ */
