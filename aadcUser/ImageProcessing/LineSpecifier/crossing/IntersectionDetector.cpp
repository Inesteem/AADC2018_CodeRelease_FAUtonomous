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
 * $Author:: schoen $   $Date:: 2016-02-08 #$
 **********************************************************************/

#include "IntersectionDetector.h"
#include "ImageProcessingUtils.h"
#include "../cache/DetectionCache.h"
#include <algorithm>
#include "../stdafx.h"

using namespace cv;

const int IntersectionDetector::DISTANCE_THICK_THIN_LINE_CROSSING = METER_TO_PIXEL(0.465);
//const int IntersectionDetector::DISTANCE_ADJACENT_SIGN = METER_TO_PIXEL(1.18);
const int IntersectionDetector::DISTANCE_ADJACENT_SIGN = METER_TO_PIXEL(-0.37);
//const int IntersectionDetector::DISTANCE_OPPOSITE_SIGN = METER_TO_PIXEL(0.83);
const int IntersectionDetector::DISTANCE_OPPOSITE_SIGN = METER_TO_PIXEL(0.4);
const int IntersectionDetector::RIGHT_LINE_END_SEARCH_RANGE = METER_TO_PIXEL(0.4);

IntersectionDetector::IntersectionDetector(int distanceEmergencyStop) {
	debugEnabled = false;
	binaryImageSize = Rect(0,0, BINARY_IMAGE_WIDTH, BINARY_IMAGE_HEIGHT);

	vDetectorRight_rot = new LineDetector_Vertical(10, 4, 5);
	vDetectorRight_rot->SetColor(Scalar(0, 255, 0));

	vDetectorMiddle_Splitted = new LineDetector_Vertical(5, 3, 8);

	endRightLine = Mat::zeros(18, 18, CV_8UC1);
	rectangle(endRightLine, Point(6, 6), Point(11, 17), Scalar(255), CV_FILLED);
	rectangle(endRightLine, Point(6, 6), Point(17, 11), Scalar(255), CV_FILLED);

	upperLeftCorner = Mat::zeros(36, 20, CV_8UC1);
	rectangle(upperLeftCorner, Point(0, 6), Point(8, 11), Scalar(255), CV_FILLED);
	rectangle(upperLeftCorner, Point(9, 0), Point(8, 11), Scalar(255), CV_FILLED);
	upperLeftCornerLineCenter = Point2f(15, 21);

	lastUpperLeftCornerMatchLoc = Point2f(0,0);

	this->distanceEmergencyStop = distanceEmergencyStop;
	isInTrackingMode = false;
}

IntersectionDetector::~IntersectionDetector() {
	delete vDetectorRight_rot;
	delete vDetectorMiddle_Splitted;
}

bool IntersectionDetector::DetectCrossingLine(Mat &input, Mat *debugImage, long long time) {
	if(!debugEnabled) {
		debugImage = 0;
	}

	CVMath::LineSegment lowerHori;

	bool foundLowerHori = false;
	hDetector.SetParameter(50, 20, LineDetector_Horizontal::INITIAL_REGION_HEIGHT, LineDetector_Horizontal::INITIAL_STEP_INCREASE);
	if(hDetector.Detect_Right(input, debugImage, BINARY_IMAGE_WIDTH / 2 - METER_TO_PIXEL(0.25), BINARY_IMAGE_HEIGHT - METER_TO_PIXEL(0.30), &lowerHori)) {
		foundLowerHori = true;
	}

	if(!foundLowerHori) {
		if(hDetector.Detect_Right(input, debugImage, BINARY_IMAGE_WIDTH / 2 - METER_TO_PIXEL(0.25), BINARY_IMAGE_HEIGHT - METER_TO_PIXEL(0.60), &lowerHori)) {
			foundLowerHori = true;
		}
	}

	if(!foundLowerHori) {
		if(hDetector.Detect_Right(input, debugImage, BINARY_IMAGE_WIDTH / 2 - METER_TO_PIXEL(0.25), BINARY_IMAGE_HEIGHT - METER_TO_PIXEL(0.90), &lowerHori)) {
			foundLowerHori = true;
		}
	}

	if(foundLowerHori && lowerHori.direction.x < 0.939) { // cos(20) = 0.939
		foundLowerHori = false;
	}

	if(foundLowerHori && cv::norm(lowerHori.start - lowerHori.end) > METER_TO_PIXEL(0.4)) {
		DetectionCache::GetInstance().SaveCrossingLine(lowerHori, time);

		if (debugImage) {
			line(*debugImage, lowerHori.start, lowerHori.end, Scalar(255, 100, 0), 2, 4);
		}

		return true;
	}

	bool foundUpperHori = false;
	CVMath::LineSegment upperHori;
	hDetector.SetParameter(30, 20, LineDetector_Horizontal::INITIAL_REGION_HEIGHT, LineDetector_Horizontal::INITIAL_STEP_INCREASE);
	if(hDetector.Detect_Right(input, debugImage, BINARY_IMAGE_WIDTH / 2 - METER_TO_PIXEL(0.25), lowerHori.start.y - DISTANCE_THICK_THIN_LINE_CROSSING, &upperHori)) {
		if(cv::norm(upperHori.start - upperHori.end) > METER_TO_PIXEL(0.4)) {
			if(upperHori.direction.x > 0.939) { // cos(20) = 0.939
				foundUpperHori = true;
			}
		}

		if (!foundUpperHori) {
			hDetector.SetParameter(30, 20, 50, LineDetector_Horizontal::INITIAL_STEP_INCREASE);
			if(hDetector.Detect_Right(input, debugImage, BINARY_IMAGE_WIDTH / 2 - METER_TO_PIXEL(0.25), std::min(upperHori.start.y,upperHori.end.y) + 30, &upperHori)) {
				if(cv::norm(upperHori.start - upperHori.end) > METER_TO_PIXEL(0.4)) {
					if(upperHori.direction.x > 0.939) { // cos(20) = 0.939
						foundUpperHori = true;
					}
				}
			}
		}
	}

	if (!foundLowerHori && !foundUpperHori) {
		hDetector.SetParameter(50, 20, LineDetector_Horizontal::INITIAL_REGION_HEIGHT, LineDetector_Horizontal::INITIAL_STEP_INCREASE);
		if(hDetector.Detect_Right(input, debugImage, BINARY_IMAGE_WIDTH / 2 - METER_TO_PIXEL(0.5), BINARY_IMAGE_HEIGHT - (LineDetector_Horizontal::INITIAL_REGION_HEIGHT >> 1), &upperHori)) {
			if(cv::norm(upperHori.start - upperHori.end) > METER_TO_PIXEL(0.4)) {
				if(upperHori.direction.x > 0.939) { // cos(20) = 0.939
					foundUpperHori = true;
				}
			}
		}
	}


	if (debugImage && foundUpperHori) {
		line(*debugImage, upperHori.start, upperHori.end, Scalar(255, 100, 0), 2, 4);
	}

	if(foundUpperHori) {
		DetectionCache::GetInstance().SaveCrossingLine(upperHori, time);
		return true;
	}

	return false;
}


bool IntersectionDetector::DetectStraightLine(Mat &input, Mat *debugImage) {
	if(!debugEnabled) {
		debugImage = 0;
	}

	int yStart = BINARY_IMAGE_HEIGHT - 3;
	int xStart = BINARY_IMAGE_WIDTH / 2 + METER_TO_PIXEL(0.25);

	CVMath::LineSegment vert;

	bool foundVert = false;
	if(vDetector.Detect_Up(input, debugImage, xStart, yStart, &vert)) {
		foundVert = true;
	}

	if (debugImage && foundVert) {
		line(*debugImage, vert.start, vert.end, Scalar(255, 100, 0), 2, 4);
	}

	return foundVert;
}

cv::Point2f IntersectionDetector::DetectIntersection(Mat &input, Mat *debugImage, float imageRoadSignDistance,
		float imageNearPlaneDistance, Point2f *yawDirection, bool *badCrossingType, long long int time) {
	if (!debugEnabled) {
		debugImage = 0;
	}

	//some cleaning up
	if (detectedStopPosition.size() > 20) {
		detectedStopPosition.resize(15);
	}

	*badCrossingType = false;

	int yStart = BINARY_IMAGE_HEIGHT - 3;
	int xStart = BINARY_IMAGE_WIDTH / 2 + METER_TO_PIXEL(0.25);

	bool foundRight = false;
	CVMath::LineSegment right = DetectionCache::GetInstance().GetRightLine(time);

	if (right.Valid()) {
		foundRight = true;
	} else {
		if (vDetector.Detect_Up(input, debugImage, xStart, yStart, &right)) {
			foundRight = true;
		}
	}

	if (!foundRight) {
		int yStart = BINARY_IMAGE_HEIGHT - 10;
		int xStart = BINARY_IMAGE_WIDTH / 2 + METER_TO_PIXEL(0.28);

		if (vDetectorRight_rot->DetectSplitted_Up(input, debugImage, xStart, yStart, &right)) {
			foundRight = true;
		}
	}

	if (!foundRight) {
		int yStart = BINARY_IMAGE_HEIGHT - 20;
		int xStart = BINARY_IMAGE_WIDTH / 2 + METER_TO_PIXEL(0.31);

		vDetectorRight_rot->SetParameter(50, 20, 100, 3);
		if (vDetectorRight_rot->DetectSplitted_Up(input, debugImage, xStart, yStart, &right)) {
			foundRight = true;
		}
	}

	bool foundMiddleLine = false;
	CVMath::LineSegment middle;

	if (foundRight) {
		int yStart = BINARY_IMAGE_HEIGHT - 3;
		int xStart = right.start.x - METER_TO_PIXEL(0.465);
		vDetectorMiddle_Splitted->SetParameter(50, 20, METER_TO_PIXEL(0.1),
				LineDetector_Vertical::INITIAL_STEP_INCREASE);
		if (vDetectorMiddle_Splitted->DetectSplitted_Up(input, debugImage, xStart, yStart, &middle)) {
			foundMiddleLine = true;
		}

		if (!foundMiddleLine) {
			int yStart = BINARY_IMAGE_HEIGHT - 45;
			int xStart = right.start.x - METER_TO_PIXEL(0.465);
			vDetectorMiddle_Splitted->SetParameter(50, 20, METER_TO_PIXEL(0.1),
					LineDetector_Vertical::INITIAL_STEP_INCREASE);
			if (vDetectorMiddle_Splitted->DetectSplitted_Up(input, debugImage, xStart, yStart, &middle)) {
				foundMiddleLine = true;
			}
		}
	}

	if (debugImage) {
		if (right.Valid()) {
			right.Draw(debugImage, Scalar(255, 100, 0));
		}
		if (middle.Valid()) {
			middle.Draw(debugImage, Scalar(255, 100, 0));
		}
	}

	//Calculate x distance to sign
	//float adjacentCarDistance = cv::sqrt(
	//		imageRoadSignDistance * imageRoadSignDistance - DISTANCE_OPPOSITE_SIGN * DISTANCE_OPPOSITE_SIGN);


	float adjacentCarDistance = imageRoadSignDistance;


	//Y-Coordinate for Start searching
	int searchY = BINARY_IMAGE_HEIGHT + imageNearPlaneDistance - (adjacentCarDistance - DISTANCE_ADJACENT_SIGN);

	if (debugImage) {
		cv::line(*debugImage, Point(0, searchY + RIGHT_LINE_END_SEARCH_RANGE),
				Point(BINARY_IMAGE_WIDTH, searchY + RIGHT_LINE_END_SEARCH_RANGE), Scalar(255, 0, 0), 2, 4);
		cv::line(*debugImage, Point(0, searchY - RIGHT_LINE_END_SEARCH_RANGE),
				Point(BINARY_IMAGE_WIDTH, searchY - RIGHT_LINE_END_SEARCH_RANGE), Scalar(255, 0, 0), 2, 4);
	}
/*
	if (foundRight && abs(right.end.y - searchY) < RIGHT_LINE_END_SEARCH_RANGE) {
		Rect roi = Rect(right.end.x - METER_TO_PIXEL(0.10), right.end.y - METER_TO_PIXEL(0.18), METER_TO_PIXEL(0.20),
				METER_TO_PIXEL(0.11));
		roi &= binaryImageSize;

		int whitePixel = cv::sum(input(roi))[0];
		//allow a maximum of 5 white pixels per row
		int whiteMax = METER_TO_PIXEL(0.025001) * roi.height * 255;

		if (whitePixel < whiteMax && roi.width > 3) {
			if (debugImage) {
				cv::rectangle(*debugImage, roi, Scalar(0, 50, 200), 1, 4);
			}
		} else {
			roi = Rect(right.end.x - METER_TO_PIXEL(0.10), right.end.y - METER_TO_PIXEL(0.25), METER_TO_PIXEL(0.20),
							METER_TO_PIXEL(0.11));
			roi &= binaryImageSize;

			whitePixel = cv::sum(input(roi))[0];

			if (debugImage) {
				cv::rectangle(*debugImage, roi, Scalar(0, 50, 200), 1, 4);
			}
		}

		if (whitePixel < whiteMax && roi.width > 3) {
			if (debugImage) {
				cv::rectangle(*debugImage, roi, Scalar(50, 0, 255), 1, 4);
			}

			//adjust right line end with the corner template
			Rect endRightROI = Rect(right.end.x - endRightLine.cols, right.end.y - 2 * endRightLine.rows,
					2 * endRightLine.cols, 2 * endRightLine.rows + METER_TO_PIXEL(0.06));
			endRightROI &= binaryImageSize;

			if (endRightROI.width > endRightLine.cols && endRightROI.height > endRightLine.rows + METER_TO_PIXEL(0.02)) {
				Mat result;
				double maxVal = 0, minVal = 0;
				Point minLoc, maxLoc;

				if (debugImage) {
					cv::rectangle(*debugImage, endRightROI, Scalar(0, 50, 200), 1, 4);
				}

				matchTemplate(input(endRightROI), endRightLine, result, CV_TM_SQDIFF);

				minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);
				int foundEndY = minLoc.y + endRightROI.y + (endRightLine.rows >> 1);

				right.CropY(right.start.y, foundEndY);

				if (debugImage) {
					if (right.Valid()) {
						//draw the right line with the new end again
						right.Draw(debugImage, Scalar(255, 100, 0));
					}

					minLoc += Point(endRightROI.x, endRightROI.y);
					cv::rectangle(*debugImage, minLoc, minLoc + Point(endRightLine.cols, endRightLine.rows),
							Scalar(0, 0, 255), 2, 4);
				}
			}

			Point2f laneStep = CVMath::RotateCW90(right.direction);
			Point2f stop;
			if (foundMiddleLine) {
				float distance = abs(abs(middle.distance) - abs(right.distance));
				stop = right.end - right.direction * METER_TO_PIXEL(0.13) + laneStep * distance * 0.5;
			} else {
				stop = right.end - right.direction * METER_TO_PIXEL(0.13) + laneStep * METER_TO_PIXEL(0.23);
			}

			if (debugEnabled) {
				circle(*debugImage, stop, 7.0, Scalar(0, 0, 255), 2, 8);
				line(*debugImage, stop, stop + 20 * right.direction, Scalar(255, 0, 0), 2);
			}

			if (foundMiddleLine) {
				float weight = right.length + middle.length;
				*yawDirection = (right.length * right.direction + middle.length * middle.direction) / weight;
			} else {
				*yawDirection = right.direction;
			}

			detectedStopPosition.push_front(Pose3D(stop, time));
			return stop;
		}
	}

	bool isCrossingLine = DetectCrossingLine(input, debugImage, time);

	while (foundRight && right.length > METER_TO_PIXEL(1)) {
		Rect upperLeftCornerROI = Rect(0, 0, 100, searchY - RIGHT_LINE_END_SEARCH_RANGE);
		if(IsInTrackingMode()) {
			upperLeftCornerROI = Rect(lastUpperLeftCornerMatchLoc.x - 10, lastUpperLeftCornerMatchLoc.y, 2 * upperLeftCorner.cols + 20, 2 * upperLeftCorner.rows + 20);
		}

		upperLeftCornerROI &= binaryImageSize;
		if(upperLeftCornerROI.width <= 2 * upperLeftCorner.cols) {
			upperLeftCornerROI.width = 2 * upperLeftCorner.cols + 1;
		}

		if(upperLeftCornerROI.height <= 2 * upperLeftCorner.rows) {
			upperLeftCornerROI.height = 2 * upperLeftCorner.rows + 1;
		}


		Mat result;
		double maxVal = 0, minVal = 0;
		Point minLoc, maxLoc;

		int shiftX = 10;
		int shiftY = 10;
		if(upperLeftCornerROI.x != 0) {
			shiftX = 0;
		}
		if(upperLeftCornerROI.y != 0) {
			shiftY = 0;
		}

		Mat searchRegionInput;
		resize(input(upperLeftCornerROI), searchRegionInput, Size(upperLeftCornerROI.width / 2, upperLeftCornerROI.height /2), 0, 0, CV_INTER_AREA);

		if(cv::sum(searchRegionInput).val[0] < 255 * 10) {
			break;
		}

		if(debugImage) {
			cv::rectangle(*debugImage, upperLeftCornerROI, Scalar(0, 0, 255), 1, 4);
		}

		copyMakeBorder(searchRegionInput, searchRegionInput, shiftY, 0, shiftX, 0, BORDER_REPLICATE);

		matchTemplate(searchRegionInput, upperLeftCorner, result, CV_TM_SQDIFF);

		minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);

		int limit;
		if(IsInTrackingMode()) {
			limit = 2 * 255 * 255 * upperLeftCorner.rows;
		} else {
			limit = 1*255 * 255 * upperLeftCorner.rows;
		}

		if (minVal < limit) {
			CVMath::LineSegment leftLine = DetectionCache::GetInstance().GetOuterLeftLine(time);

			minLoc = 2 * minLoc + Point(upperLeftCornerROI.x - shiftX * 2, upperLeftCornerROI.y - shiftY * 2);

			if(right.Valid()) {
				if(minLoc.x > right.end.x - METER_TO_PIXEL(0.75)) {
					break;
				}
			}

			if(leftLine.Valid()) {
				if(minLoc.y < 0.4*leftLine.start.y + 0.6 * leftLine.end.y || minLoc.x < leftLine.end.x) {
					break;
				}
			}

			if(isCrossingLine) {
				CVMath::LineSegment crossingLine = DetectionCache::GetInstance().GetCrossingLine(time);
				if(minLoc.y - crossingLine.start.y < 15) {
					break;
				}

				if(crossingLine.start.y > minLoc.y ) {
					break;
				}
			}

			lastUpperLeftCornerMatchLoc = minLoc;

			if (debugImage) {
				cv::rectangle(*debugImage, minLoc, minLoc + Point(2*upperLeftCorner.cols, 2*upperLeftCorner.rows),
						Scalar(0, 0, 255), 2, 4);
			}

			Point2f laneStep = CVMath::RotateCW90(right.direction);
			Point2f leftPoint = Point2f(minLoc) + upperLeftCornerLineCenter - right.direction * METER_TO_PIXEL(1.18);
			if(right.length < METER_TO_PIXEL(0.1)) {
				leftPoint = Point2f(minLoc) + upperLeftCornerLineCenter - Point2f(0, -1) * METER_TO_PIXEL(1.18);
			}

			float tmp;
			Point2f rightPoint = CVMath::IntersectionTwoLines(right, CVMath::Line(leftPoint, laneStep), &tmp);

			Point2f stop;
			if (foundMiddleLine) {
				Point2f middlePoint = CVMath::IntersectionTwoLines(middle, CVMath::Line(leftPoint, laneStep), &tmp);
				stop = 0.125 * leftPoint + 0.625 * rightPoint + 0.25 * middlePoint;
			} else {
				stop = leftPoint * 0.24 + rightPoint * 0.76;
			}

			if (debugEnabled) {
				circle(*debugImage, stop, 7.0, Scalar(0, 0, 255), 2, 8);
				line(*debugImage, stop, stop + 20 * right.direction, Scalar(255, 0, 0), 2);
			}

			if (foundMiddleLine) {
				float weight = right.length + middle.length;
				*yawDirection = (right.length * right.direction + middle.length * middle.direction) / weight;
			} else {
				if(right.length > METER_TO_PIXEL(0.1)) {
					*yawDirection = right.direction;
				} else {
					*yawDirection = Point2f(0, -1);
				}
			}

			detectedStopPosition.push_front(Pose3D(stop, time));
			return stop;
		}

		break;
	}
	*/
		//LOG_INFO(cString::Format("LS: searchY: %d",searchY));
		
		
		
	if(searchY > BINARY_IMAGE_HEIGHT )
	//if ((foundRight && right.length > METER_TO_PIXEL(0.75) && searchY > BINARY_IMAGE_HEIGHT + 30) //T-Crossing
	//		|| (!foundRight && searchY > BINARY_IMAGE_HEIGHT ) //no T-crossing, crossing not fully visible
	//		|| (foundRight && right.length < METER_TO_PIXEL(0.1) && searchY > BINARY_IMAGE_HEIGHT )) //no T-crossing, crossing not fully visible
	{
		//TODO: workaround, no lines detected. stop with the distance of the sign


		LOG_WARNING(cString::Format("LS: emergency stop: %d",adjacentCarDistance));
		Point2f stop = Point2f(BINARY_IMAGE_WIDTH / 2,
		BINARY_IMAGE_HEIGHT + imageNearPlaneDistance - (adjacentCarDistance - distanceEmergencyStop));

		if (foundRight) {
			*yawDirection = right.direction;
		} else {
			*yawDirection = Point2f(0, -1);
		}

		if (debugEnabled) {



			circle(*debugImage, stop, 7.0, Scalar(0, 0, 255), 2, 8);
			line(*debugImage, stop, stop + 20 * *yawDirection, Scalar(255, 0, 0), 2);


			char info[100];
			sprintf(info, "Emergency x %f y %f", stop.x, stop.y);
            //cv::putText(*debugImage, cv::String(info), Point2f(0, 40),
            //		CV_FONT_NORMAL, 0.35, Scalar(0, 0, 255));
		}

		detectedStopPosition.push_front(Pose3D(stop, time));

		*badCrossingType = true;

		return stop;
	}

	detectedStopPosition.push_front(Pose3D());
	return Point2f(0, 0);
}

Point2f IntersectionDetector::OnLeftTurn(Mat &input, Mat *debugImage) {
	if(!debugEnabled) {
		debugImage = 0;
	}

	LineDetector_Vertical redDetector(12, 4, 12);
	redDetector.SetColor(Scalar(0, 0, 255));
	CVMath::LineSegment redLine;
	redDetector.SetParameter(30, 10, 60, LineDetector_Vertical::INITIAL_STEP_INCREASE);;
	if (redDetector.DetectSplitted_Up(input, debugImage, 100, 290, &redLine)) {
		if(redLine.length > METER_TO_PIXEL(0.31)) {
			if (debugImage) {
				line(*debugImage, redLine.start, redLine.end, Scalar(0, 0, 250), 2, 4);
			}

			return redLine.start - Point2f(METER_TO_PIXEL(0.225), 0) * (cv::abs(redLine.direction.x) + cv::abs(redLine.direction.y));
		}
	}

	if (redDetector.DetectSplitted_Up(input, debugImage, 140, 270, &redLine)) {
		if(redLine.length > METER_TO_PIXEL(0.31)) {
			if (debugImage) {
				line(*debugImage, redLine.start, redLine.end, Scalar(0, 0, 250), 2, 4);
			}

			return redLine.start - Point2f(METER_TO_PIXEL(0.225), 0) * (cv::abs(redLine.direction.x) + cv::abs(redLine.direction.y));
		}
	}

	if(redLine.Valid()) {
		return Point2f(0,0);
	} else {
		return Point2f(0,290);
	}
}

Point2f IntersectionDetector::OnRightTurn(Mat &input, Mat *debugImage) {
	if(!debugEnabled) {
		debugImage = 0;
	}

	LineDetector_Vertical redDetector(12, 4, 12);
	redDetector.SetColor(Scalar(0, 0, 255));
	CVMath::LineSegment redLine;
	redDetector.SetParameter(30, 20, 100, LineDetector_Vertical::INITIAL_STEP_INCREASE);;
	if (redDetector.DetectSplitted_Up(input, debugImage, 100, 290, &redLine)) {
		if(redLine.length > METER_TO_PIXEL(0.40) && redLine.end.y - redLine.start.y > 0) {
			if (debugImage) {
				line(*debugImage, redLine.start, redLine.end, Scalar(0, 0, 250), 2, 4);
			}

			return redLine.start + Point2f(METER_TO_PIXEL(0.6), 0) * (cv::abs(redLine.direction.x) + cv::abs(redLine.direction.y));
		}

	}

	return Point2f(0,0);
}
