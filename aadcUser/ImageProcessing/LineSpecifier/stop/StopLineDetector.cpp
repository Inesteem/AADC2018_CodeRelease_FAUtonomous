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
 * $Author:: schoen $   $Date:: 2016-01-16 #$
 **********************************************************************/

#include "StopLineDetector.h"
#include "../cache/DetectionCache.h"
#include "CVMath.h"
#include <vector>

#include "ImageProcessingUtils.h"

using namespace std;

const int StopLineDetector::STOP_LINE_SEARCH_Y_LIMIT_IMAGE_COORD = 100;

StopLineDetector::~StopLineDetector() {
}

Point2f StopLineDetector::Detect(Mat &input, Mat &debugImage, Point2f carPosition, Point2f dest, Point2f *direction, long long int time) {
	const int ROI_WIDTH = 3;
	const int WHITE_TRIGGER = 255 * ROI_WIDTH;
	const int BLACK_TRIGGER = 0;

	//some cleaning up
	if(detectedStopPosition.size() > 20) {
		detectedStopPosition.resize(15);
	}

	enum State {
		SEARCH_BEGIN,
		SEARCH_END
	};

	Point2f steeringDirection = dest - carPosition;
	steeringDirection = steeringDirection / cv::norm(steeringDirection);

	list<Point2i> foundStopMarkersList;

	int endCol = input.cols;
	int startRow = 0;
	if(IsInTrackingMode()) {
		startRow = detectedStopPosition.front().pos.y - METER_TO_PIXEL(0.22);
	}

	Point2f start, stepWidth;

	float diff = dest.x - carPosition.x;
	if(diff > stepWidthX) {
		//calculate ray
		Point2f e = CVMath::RotateCW90(steeringDirection) * laneWidth_Pixel + carPosition;
		//calculate start of ray
		start = CVMath::IntersectionTwoLines(Point2f(0, 1), 284 /*pixel*/, e, steeringDirection);

		float scale = 1 / steeringDirection.x * stepWidthX;
		stepWidth = Point2i(steeringDirection * scale);

	} else if (diff <  -stepWidthX)  {
		//calculate ray
		Point2f e = CVMath::RotateCW90(steeringDirection) * (-laneWidth_Pixel) + carPosition;
		//calculate start of ray
		start = CVMath::IntersectionTwoLines(Point2f(1, 0), 0 /*pixel*/, e, steeringDirection);

		float scale = 1 / abs(steeringDirection.x) * stepWidthX;
		stepWidth = Point2i(- steeringDirection * scale);
	} else {
		endCol = dest.x + laneWidth_Pixel;
		if(endCol > input.cols) {
			endCol = input.cols;
		}

		start = Point2i(dest.x - laneWidth_Pixel, 0);
		stepWidth = Point2i(stepWidthX, 0);
	}

	for (;(start.x < endCol- ROI_WIDTH) && (start.y < 285) && (0 < start.x); start += stepWidth) {
		State state = SEARCH_BEGIN;
		float yLocation = 0;
		int sum[3] = { 0, 0, 0};

		Mat work = input.colRange(start.x, start.x + ROI_WIDTH);
		const int upperLimit = STOP_LINE_SEARCH_Y_LIMIT_IMAGE_COORD;
		int row = start.y;
		if(start.y < upperLimit) {
			row = upperLimit;
		}

		if(start.y < startRow) {
			row = startRow;
		}

		if(row < 0) {
			row = 0;
		}

		for (; row < work.rows; row++) {
			Scalar a = cv::sum(work.row(row));
			sum[0] = a.val[0];

			if (state == SEARCH_BEGIN) {
				bool found = false;
				float lineWidth = 0;
				if (sum[0] == WHITE_TRIGGER && sum[1] == BLACK_TRIGGER) {
					lineWidth = row - yLocation;
					yLocation = row;
					found = true;
				} else if (sum[0] == WHITE_TRIGGER && sum[2] == BLACK_TRIGGER) {
					lineWidth = row - 1 - yLocation;
					yLocation = row - 1;
					found = true;
				} else if(sum[0] + sum[1] + sum[2] > 255 * 4) {
					yLocation = row;
				}

				if(found && lineWidth > 3) {
					state = SEARCH_END;
				} else if (found){
					yLocation  += 2;
				}
			} else if (state == SEARCH_END) {
				bool found = false;
				float lineWidth = 0;

				if (sum[0] == BLACK_TRIGGER && sum[1] == WHITE_TRIGGER) {
					lineWidth = row - yLocation;
					found = true;
				} else	if (sum[0] == BLACK_TRIGGER && sum[2] == WHITE_TRIGGER) {
					lineWidth = row - 1 - yLocation;
					found = true;
				} else if(sum[0] + sum[1] + sum[2] < 255 * 6) {
					state = SEARCH_BEGIN;
					yLocation = row;
				}

				if(found) {
					if (lineWidth > MIN_STOP_LINE_PIXEL_WIDTH && lineWidth < MAX_STOP_LINE_PIXEL_WIDTH) {
						Point2f a = Point2f(start.x + ROI_WIDTH / 2, yLocation + lineWidth / 2);
						foundStopMarkersList.push_back(Point2i(a));
					}
					state = SEARCH_BEGIN;
					yLocation = row;
				}
			}

			sum[2] = sum[1];
			sum[1] = sum[0];
		}
	}

	if(debugEnabled) {
		for (list<Point2i>::iterator it = foundStopMarkersList.begin(); it != foundStopMarkersList.end(); it++) {
			circle(debugImage, *it, 3.0, Scalar(0, 0, 255), 1, 8);
		}
	}

	EvaluateStopLineMarkers(foundStopMarkersList, steeringDirection);

	if(foundStopMarkersList.empty()) {
		detectedStopPosition.push_front(Pose3D());
		return Point2f(0,0);
	}

	return VerifyDetection(input, debugImage, foundStopMarkersList, carPosition, steeringDirection, direction, time);
}

Point2f StopLineDetector::VerifyDetection(Mat &input, Mat &debugImage, list<Point2i> &foundStopMarkers, Point2f carPosition, Point2f steeringDirection, Point2f *direction, long long  int time) {
	Point2i mean = Point2f(0,0);
	int i = 0;
	for (list<Point2i>::iterator it = foundStopMarkers.begin(); it != foundStopMarkers.end(); it++) {
		mean += *it;
		i++;
	}
	Point2f stopPoint = Point2f(mean) / float(i);

	Point2f stopSteeringVector = (stopPoint - carPosition);
	float distance = cv::norm(stopSteeringVector);
	stopSteeringVector /= distance;

	steeringDirection /= cv::norm(steeringDirection);

	float cosAngle = abs(steeringDirection.dot(stopSteeringVector));

	if(cosAngle < AngleLimit(abs(atan(steeringDirection.x / steeringDirection.y) * 1.9099 /* 180 / CV_PI / 30*/))) {
		detectedStopPosition.push_front(Pose3D());
		return Point2f(0,0);
	}

	detectedStopPosition.push_front(Pose3D(stopPoint, time));

	if(debugEnabled) {
		Point2f a = *(foundStopMarkers.begin());
		Point2f b = *(foundStopMarkers.rbegin());
		line(debugImage, a, b, Scalar (0, 0, 255), 1);
	}

	int size = detectedStopPosition.size();
	int valIdx[3];
	int n = 0;
	for (int i = 0; i < size && n < 3; i++) {
		if(!detectedStopPosition[i].valid) {
			continue;
		} else {
			valIdx[n] = i;
			n++;
		}
	}

	if (n == 3) {
		Point2f p0 = detectedStopPosition[valIdx[0]].pos;
		Point2f p1 = detectedStopPosition[valIdx[1]].pos;
		Point2f p2 = detectedStopPosition[valIdx[2]].pos;

		if ((cv::norm(p0 - p1) < maxMovementDistance * (valIdx[1] - valIdx[0]))
				&& (cv::norm(p1 - p2) < maxMovementDistance * (valIdx[2] - valIdx[1]))
				&& (cv::norm(p0 - p2) < maxMovementDistance * (valIdx[2] - valIdx[0]))) {

			vector<Point2i> vec1(foundStopMarkers.begin(),foundStopMarkers.end());
			Vec4f tempStopLine;
			fitLine(vec1, tempStopLine, CV_DIST_L2, 0, 0.1, 0.1);
			CVMath::Line stopLine(Point2f(tempStopLine[2], tempStopLine[3]), Point2f(tempStopLine[0], tempStopLine[1]));

			Point2f begin = *(foundStopMarkers.begin());
			Point2f end = *(foundStopMarkers.rbegin());

			Rect roiRight = Rect(end.x - 20, end.y - 10, 80, 55) & binaryImageSize;
			Rect roiLeft = Rect(begin.x - 60, begin.y - 5, 80, 50) & binaryImageSize;

			if (debugEnabled) {
				line(debugImage, begin, end, Scalar(0, 0, 255), 3);
				cv::rectangle(debugImage, roiRight, Scalar(0, 0, 200), 1);
				cv::rectangle(debugImage, roiLeft, Scalar(0, 0, 200), 1);
			}

			CVMath::LineSegment tmp = DetectionCache::GetInstance().GetRightLine(time);
			CVMath::Line lineRight;
			bool validRightLine = false;
			if(tmp.Valid() && tmp.length > METER_TO_PIXEL(0.3)) {
				lineRight = tmp;
				validRightLine = true;
			} else {
				validRightLine = EstimateVerticalLine(input, debugImage, roiRight, stopLine.normal, &lineRight);
			}

			if (validRightLine) {
				float t_stopLine;
				Point2f intersection = CVMath::IntersectionTwoLines(stopLine, lineRight, &t_stopLine);
				if(lineRight.direction.y > 0) {
					lineRight.direction = - lineRight.direction;
				}

				Point2f resultingStopPoint;
				if(stopLine.direction.x > 0) {
					stopLine.direction = - stopLine.direction;
				}

				Point2f yawDirection = lineRight.direction - CVMath::RotateCW90(stopLine.direction);
				yawDirection /= cv::norm(yawDirection);

				*direction = yawDirection;

				resultingStopPoint = intersection + stopLine.direction * laneWidth_Pixel * 0.5;

				if (debugEnabled) {
					circle(debugImage, resultingStopPoint, 7.0, Scalar(0, 0, 255), 2, 8);
					line(debugImage, resultingStopPoint, resultingStopPoint + 20 * yawDirection, Scalar(255, 0, 0), 2);
				}

				return resultingStopPoint;
			}

			//EstimateVericalLine(input, debugImage, roiLeft, normalOfStopLine);

			return Point2f(0,0);

		} else {
			detectedStopPosition[valIdx[0]].valid = false;
		}
	}

	return Point2f(0,0);
}

void StopLineDetector::EvaluateStopLineMarkers(list<Point2i> & stopMarkers, Point2f dir) {
	//clear upper row if two rows are detected

	Rect boundingRect = Rect(0, BINARY_IMAGE_HEIGHT, 0, 00);
	for (list<Point2i>::iterator it = stopMarkers.begin(); it != stopMarkers.end(); it++) {
		if(boundingRect.y > it->y) {
			boundingRect.y = it->y;
			boundingRect.height += boundingRect.y - it->y;
		}

		if(boundingRect.y + boundingRect.height < it->y) {
			boundingRect.height = it->y - boundingRect.y;
		}
	}

	if(boundingRect.height > METER_TO_PIXEL(0.10)) {
		int limit = boundingRect.y + boundingRect.height / 2;
		for (list<Point2i>::iterator it = stopMarkers.begin(); it != stopMarkers.end(); ) {
			if(it->y < limit) {
				it = stopMarkers.erase(it);
			} else {
				it++;
			}
		}
	}

	list<Point2i>::iterator it = stopMarkers.begin();
	int i = 0;
	list<Point2i>::iterator last = it;
	it++;
	if(it == stopMarkers.end()) {
		stopMarkers.clear();
		return;
	}

	dir = CVMath::RotateCW90(dir);

	float limit_ANG = cosf(15 * CV_PI / 180);
	float limit_LEN = METER_TO_PIXEL(0.44) / 3.9;

	list<Point2i>::iterator itC = last;

	bool chainFound = false;

	//find chain of three succeeding points
	for (i = 1; it != stopMarkers.end(); it++, i++) {
		if(i > 1) {
			last = it, last--;
			itC = last, itC--;

			Point2i c = *itC;
			Point2i b = *last;
			Point2i a = *it;

			Point2f CB = b-c;
			float lenCB = cv::norm(CB);
			CB /= lenCB;
			float cosAngleCB = abs(dir.dot(CB));

			Point2f BA = a-b;
			float lenBA = cv::norm(BA);
			BA /= lenBA;
			float cosAngleBA = abs(dir.dot(BA));

			Point2f CA = a-c;
			float lenCA = cv::norm(CA);
			CA /= lenCA;
			float cosAngleCA = abs(dir.dot(CA));

			if(Helper_CheckCondition(cosAngleCB, limit_ANG, lenCB, limit_LEN)) {
				if(Helper_CheckCondition(cosAngleBA, limit_ANG, lenBA, limit_LEN) || chainFound) {
					stopMarkers.erase(last);
					i--;
				}

				if(chainFound) {
					if(Helper_CheckCondition(cosAngleCA, limit_ANG, lenCA, limit_LEN)) {
						it = stopMarkers.erase(it, stopMarkers.end());
						//i--;
						break;
					}
				} else {
					if(Helper_CheckCondition(cosAngleCA, limit_ANG, lenCA, limit_LEN)) {
						stopMarkers.erase(itC);
						i--;
					}
				}
			} else {
				if(Helper_CheckCondition(cosAngleBA, limit_ANG, lenBA, limit_LEN)) {
					if(chainFound == false) {
						stopMarkers.erase(last);
						stopMarkers.erase(itC);
						i -= 2;
					} else if(Helper_CheckCondition(cosAngleCA, limit_ANG, lenCA, limit_LEN)) {
						it = stopMarkers.erase(it);
						it--;
						i--;
					}
				} else if (chainFound == false){
					chainFound = true;
					dir = (CB + BA + CA) * 0.3333333f;
					float dirLength = cv::norm(dir);
					dir /= dirLength;
					limit_LEN = lenCA * 0.5 * 1.1;
					limit_ANG = cosf(10 * CV_PI / 180);
				}
			}
		}
	}

	if (i < 3) {
		stopMarkers.clear();
	}
}

bool StopLineDetector::EstimateVerticalLine(Mat &input, Mat &debugImage, Rect roi, Point2i searchedSlope, CVMath::Line *outputLine) {
	vector<Vec4i> foundLines;
	Mat localCopy;
	input(roi).copyTo(localCopy);
	Canny(localCopy, localCopy, 50, 200, 3);

	HoughLinesP(localCopy, foundLines, 1, CV_PI / 180 , 10, 15, 4);

	const float COS_ANGLE_LIMIT = cos(20.0 / 180 * CV_PI);

	int resultListSize = 0;
	Point2f resultTangent;
	Point2f resultPoint;

	size_t listSize = foundLines.size();
	for (size_t i = 0; i < listSize; i++) {
		Vec4i l = foundLines[i];

		Point2f p0 = Point2f(l[0], l[1]) + Point2f(roi.x, roi.y);
		Point2f p1 = Point2f(l[2], l[3]) + Point2f(roi.x, roi.y);

		Point2f tangent = p0 - p1;
		tangent /= cv::norm(tangent);

		float dotProduct = tangent.dot(searchedSlope);
		if (dotProduct > COS_ANGLE_LIMIT || -dotProduct > COS_ANGLE_LIMIT ) {
			if(debugEnabled) {
				line(debugImage, p0, p1, Scalar(255, 0, 0), 2);
			}

			resultPoint += p0;
			resultTangent += tangent;
			resultListSize++;
		}
	}

	if(resultListSize > 0) {
		*outputLine = CVMath::Line (resultPoint / resultListSize, resultTangent / resultListSize);
		if(debugEnabled) {
			line(debugImage, outputLine->pointOnLine - outputLine->direction*20, outputLine->pointOnLine + outputLine->direction*20, Scalar(0, 100, 100), 2);
		}
		return true;
	}

	return false;
}
