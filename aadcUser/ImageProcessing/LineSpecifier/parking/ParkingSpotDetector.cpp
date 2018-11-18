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

#include "ParkingSpotDetector.h"

#include "ScmCommunication.h"

const int ParkingSpotDetector::PARKING_SPOT_LENGTH_LONG = METER_TO_PIXEL(0.7853);
const int ParkingSpotDetector::PARKING_SPOT_LENGTH_TRANS = METER_TO_PIXEL(0.47);
const int ParkingSpotDetector::PARKING_SPOT_LENGTH_TOLERANCE = METER_TO_PIXEL(0.08);
const int ParkingSpotDetector::DISTANCE_THICK_THIN_LINE_CROSSING = METER_TO_PIXEL(0.465);

ParkingSpotDetector::ParkingSpotDetector() {
	debugEnabled = false;

	hDetector.SetParameter(LineDetector_Horizontal::STOP_SEARCH_HORIZONTAL, 30, LineDetector_Horizontal::INITIAL_REGION_HEIGHT, LineDetector_Horizontal::INITIAL_STEP_INCREASE);
}

ParkingSpotDetector::~ParkingSpotDetector() {
}

ParkingSpotDetector::ParkingSpot ParkingSpotDetector::Detect(Mat &input, Mat *debugImage, int type, bool isFirstDetection) {
	if(!debugEnabled) {
		debugImage = 0;
	}

	int yStart = BINARY_IMAGE_HEIGHT - 3;
	int xStart = BINARY_IMAGE_WIDTH / 2 + METER_TO_PIXEL(0.25);

	CVMath::LineSegment vert;
	CVMath::LineSegment lowerHori;
	CVMath::LineSegment upperHori;

	bool foundVert = false;
	if(vDetector.Detect_Up(input, debugImage, xStart, yStart, &vert)) {
		foundVert = true;
	} else {
		return ParkingSpot();
	}

	if (debugImage && foundVert) {
		line(*debugImage, vert.start, vert.end, Scalar(255, 100, 0), 2, 4);
	}

	Point2f begin = FindHorizontalLineStartPoint(input, debugImage, vert, 0);

	bool foundHorizontalLine = false;
	if(hDetector.Detect_Right(input, debugImage, begin.x, begin.y, &lowerHori)) {
		foundHorizontalLine = true;
	} else {
		const int VERTICAL_STEP_WIDTH = BINARY_IMAGE_HEIGHT >> 2;
		int yOffset = VERTICAL_STEP_WIDTH;
		Point2f begin = FindHorizontalLineStartPoint(input, debugImage, vert, yOffset);

		if (hDetector.Detect_Right(input, debugImage, begin.x, begin.y, &lowerHori)) {
			foundHorizontalLine = true;
		} else {
			yOffset += VERTICAL_STEP_WIDTH;
			if (yOffset < BINARY_IMAGE_HEIGHT) {
				Point2f begin = FindHorizontalLineStartPoint(input, debugImage, vert, yOffset);

				if (hDetector.Detect_Right(input, debugImage, begin.x, begin.y, &lowerHori)) {
					foundHorizontalLine = true;
				} else {
					yOffset += VERTICAL_STEP_WIDTH;
					if (yOffset < BINARY_IMAGE_HEIGHT) {
						Point2f begin = FindHorizontalLineStartPoint(input, debugImage, vert, yOffset);

						if (hDetector.Detect_Right(input, debugImage, begin.x, begin.y, &lowerHori)) {
							foundHorizontalLine = true;
						}
					}
				}
			}
		}
	}

	if(!foundHorizontalLine) {
		return ParkingSpot();
	}

	if (debugImage) {
		if (foundHorizontalLine) {
			line(*debugImage, lowerHori.start, lowerHori.end, Scalar(255, 100, 0), 2, 4);
		}
	}

	bool foundSecondHorizontalLine = false;

	if(type == AC_LS_DETECT_LONG_PARKING_SPOT_SLOW) {
		begin = FindStartPoint_SecondLine(input, debugImage, vert, lowerHori.start.y - PARKING_SPOT_LENGTH_LONG);

		if (hDetector.Detect_Right(input, debugImage, begin.x, begin.y, &upperHori)) {
			foundSecondHorizontalLine = true;
		}
	} else {
		begin = FindStartPoint_SecondLine(input, debugImage, vert, lowerHori.start.y - PARKING_SPOT_LENGTH_TRANS + 25);

		hDetectorTransSecondLine.SetParameter(LineDetector_Horizontal::STOP_SEARCH_HORIZONTAL, LineDetector_Horizontal::STOP_SEARCH_HORIZONTAL, 60, 3);
		if (hDetectorTransSecondLine.Detect_Right(input, debugImage, begin.x, begin.y, &upperHori)) {
			foundSecondHorizontalLine = true;
		} else {
			begin = FindStartPoint_SecondLine(input, debugImage, vert, lowerHori.start.y - PARKING_SPOT_LENGTH_TRANS - 25);

			if (hDetectorTransSecondLine.Detect_Right(input, debugImage, begin.x, begin.y, &upperHori)) {
				foundSecondHorizontalLine = true;
			}
		}
	}


	if (debugImage) {
		if (foundSecondHorizontalLine) {
			line(*debugImage, upperHori.start, upperHori.end, Scalar(255, 100, 0), 2, 4);
		}
	}

	if(foundSecondHorizontalLine) {
		float t1, t2;
		Point2f lower = CVMath::IntersectionTwoLines(vert, lowerHori, &t1);
		Point2f upper = CVMath::IntersectionTwoLines(vert, upperHori, &t2);

		if(t1 > vert.length || t2 > vert.length) {
			return ParkingSpot();
		}

		float dotProduct = lowerHori.direction.dot(upperHori.direction);
		if(-0.939 < dotProduct && dotProduct < 0.939) {
			return ParkingSpot();
		}

		dotProduct = lowerHori.direction.dot(vert.direction);
		if(dotProduct < -0.342 || 0.342 < dotProduct) {
			if(debugImage) {
				char info[100];
				sprintf(info, "FAILED: parking wrong dotProduct %.2f ", dotProduct);

                //cv::putText(*debugImage, cv::String(info), Point2f(0, 30), CV_FONT_NORMAL, 0.35, Scalar(0, 0, 255));
			}
			return ParkingSpot();
		}

		float parkingSpotLength = cv::norm(lower - upper);
		float limitParkingSpotLongOrTrans = (PARKING_SPOT_LENGTH_LONG - PARKING_SPOT_LENGTH_TRANS) / 2
				+ PARKING_SPOT_LENGTH_TRANS;

		if(type == AC_LS_DETECT_TRANS_PARKING_SPOT_SLOW && parkingSpotLength > (PARKING_SPOT_LENGTH_LONG - PARKING_SPOT_LENGTH_TOLERANCE)) {
			if(debugImage) {
				char info[100];
				sprintf(info, "FAILED: parking trans wrong length %.2f ", parkingSpotLength);

                //cv::putText(*debugImage, cv::String(info), Point2f(0, 30), CV_FONT_NORMAL, 0.35, Scalar(0, 0, 255));
			}
			return ParkingSpot();
		}

		if(type == AC_LS_DETECT_LONG_PARKING_SPOT_SLOW && (parkingSpotLength < (PARKING_SPOT_LENGTH_TRANS + PARKING_SPOT_LENGTH_TOLERANCE) || parkingSpotLength > (PARKING_SPOT_LENGTH_LONG + PARKING_SPOT_LENGTH_TOLERANCE))) {
			if(debugImage) {
				char info[100];
				sprintf(info, "FAILED: parking long wrong length %.2f ", parkingSpotLength);

                //cv::putText(*debugImage, cv::String(info), Point2f(0, 30), CV_FONT_NORMAL, 0.35, Scalar(0, 0, 255));
			}
			return ParkingSpot();
		}


		if(0 && isFirstDetection && upperHori.length < METER_TO_PIXEL(0.2)) {
			if(debugImage) {
				char info[100];
				sprintf(info, "FAILED: parking not free %.2f ", upperHori.length);

                //cv::putText(*debugImage, cv::String(info), Point2f(0, 30), CV_FONT_NORMAL, 0.35, Scalar(0, 0, 255));
			}
			return ParkingSpot();
		}

		if(parkingSpotLength >  PARKING_SPOT_LENGTH_TRANS / 2) {
			ParkingSpot spot;
			spot.parkingSpotLength = parkingSpotLength;
			spot.valid = true;
			spot.verticalLine = vert;

			if(parkingSpotLength > limitParkingSpotLongOrTrans) {
				spot.typeLong = true;
			} else {
				spot.typeLong = false;
			}
			spot.locationOnVerticalLine = 0.5 * (lower + upper);

			if(debugEnabled) {
                //putText(*debugImage, cv::String("P"), spot.locationOnVerticalLine, CV_FONT_NORMAL, 1.0, Scalar(255, 0, 0));
			}

			return spot;
		}
	}

	return ParkingSpot();
}

Point2f ParkingSpotDetector::FindHorizontalLineStartPoint(Mat &image, Mat *debugImage, CVMath::LineSegment vert, int yOffset) {
	int yStart = vert.start.y - yOffset - (LineDetector_Horizontal::INITIAL_REGION_HEIGHT >> 1);
	int xStart = 0;
	if(vert.direction.x > 0.05) {
		xStart = vert.GetPointFor_Y(yStart).x + 4;
	} else if(vert.direction.x > 0) {
		xStart = vert.GetPointFor_Y(yStart).x + 10;
	} else if (vert.direction.x < -0.05) {
		xStart = vert.GetPointFor_Y(vert.start.y - yOffset).x + 4;
	} else {
		xStart = vert.GetPointFor_Y(vert.start.y - yOffset).x + 10;
	}
	return Point2f(xStart, vert.start.y - yOffset);
}


Point2f ParkingSpotDetector::FindStartPoint_SecondLine(Mat &image, Mat *debugImage, CVMath::LineSegment vert, int yOffset) {
	int y = yOffset -(LineDetector_Horizontal::INITIAL_REGION_HEIGHT >> 1);
	int xStart = 0;
	if(vert.direction.x > 0.05) {
		xStart = vert.GetPointFor_Y(y).x + 4;
	} else if(vert.direction.x > 0) {
		xStart = vert.GetPointFor_Y(y).x + 10;
	} else if (vert.direction.x < -0.05) {
		xStart = vert.GetPointFor_Y(y + LineDetector_Horizontal::INITIAL_REGION_HEIGHT).x + 4;
	} else {
		xStart = vert.GetPointFor_Y(y + LineDetector_Horizontal::INITIAL_REGION_HEIGHT).x + 10;
	}

	return Point2f(xStart, yOffset);
}

bool ParkingSpotDetector::IsLongitudinalParkingSpot(Mat& input, Mat* debugImage) {
	if(!debugEnabled) {
		debugImage = 0;
	}

	int yStart = BINARY_IMAGE_HEIGHT - 3;
	int xStart = BINARY_IMAGE_WIDTH / 2 - METER_TO_PIXEL(0.25);

	CVMath::LineSegment vertLeft;

	if(vDetector.Detect_Up(input, debugImage, xStart, yStart, &vertLeft)) {
		if (debugImage) {
			line(*debugImage, vertLeft.start, vertLeft.end, Scalar(255, 100, 0), 2, 4);
		}

		if(vertLeft.direction.y < -0.939) {
			return true;
		}
	}

	yStart = BINARY_IMAGE_HEIGHT - 3;
	xStart = BINARY_IMAGE_WIDTH / 2 + METER_TO_PIXEL(0.25);

	CVMath::LineSegment vertRight;

	if(vDetector.Detect_Up(input, debugImage, xStart, yStart, &vertRight)) {
		if (debugImage) {
			line(*debugImage, vertRight.start, vertRight.end, Scalar(255, 100, 0), 2, 4);
		}

		if(vertRight.direction.y < -0.939) {
			return true;
		}
	}

	return false;
}


bool ParkingSpotDetector::IsTransversalParkingSpot(Mat &input, Mat *debugImage, Point2f *goal, Point2f *direction) {
	if(!debugEnabled) {
		debugImage = 0;
	}

	CVMath::LineSegment lowerHori;

	bool foundLowerHori = false;
	hDetectorTrack.SetParameter(30, 20, LineDetector_Horizontal::INITIAL_REGION_HEIGHT, LineDetector_Horizontal::INITIAL_STEP_INCREASE);
	if(hDetectorTrack.Detect_Right(input, debugImage, BINARY_IMAGE_WIDTH / 2 - METER_TO_PIXEL(0.25), BINARY_IMAGE_HEIGHT - METER_TO_PIXEL(0.30), &lowerHori)) {
		foundLowerHori = true;
	}

	if(!foundLowerHori) {
		if(hDetectorTrack.Detect_Right(input, debugImage, BINARY_IMAGE_WIDTH / 2 - METER_TO_PIXEL(0.25), BINARY_IMAGE_HEIGHT - METER_TO_PIXEL(0.60), &lowerHori)) {
			foundLowerHori = true;
		}
	}

	if(!foundLowerHori) {
		if(hDetectorTrack.Detect_Right(input, debugImage, BINARY_IMAGE_WIDTH / 2 - METER_TO_PIXEL(0.25), BINARY_IMAGE_HEIGHT - METER_TO_PIXEL(0.90), &lowerHori)) {
			foundLowerHori = true;
		}
	}

	if(foundLowerHori && lowerHori.direction.x < 0.939) { // cos(20) = 0.939
		foundLowerHori = false;
	}

	if (debugImage && foundLowerHori) {
		line(*debugImage, lowerHori.start, lowerHori.end, Scalar(255, 100, 0), 2, 4);
	}

	bool foundUpperHori = false;
	CVMath::LineSegment upperHori;
	hDetectorTrack.SetParameter(30, 20, LineDetector_Horizontal::INITIAL_REGION_HEIGHT, LineDetector_Horizontal::INITIAL_STEP_INCREASE);
	if(hDetectorTrack.Detect_Right(input, debugImage, BINARY_IMAGE_WIDTH / 2 - METER_TO_PIXEL(0.25), lowerHori.start.y - DISTANCE_THICK_THIN_LINE_CROSSING, &upperHori)) {
		if(cv::norm(upperHori.start - upperHori.end) > METER_TO_PIXEL(0.4)) {
			if(upperHori.direction.x > 0.939) { // cos(20) = 0.939
				foundUpperHori = true;
			}
		}
	}

	if (debugImage && foundUpperHori) {
		line(*debugImage, upperHori.start, upperHori.end, Scalar(255, 100, 0), 2, 4);
	}

	goal->x = BINARY_IMAGE_WIDTH / 2;
	if(foundLowerHori && foundUpperHori) {
		*direction = (upperHori.direction + lowerHori.direction) / 2;
		Point2f p1 = CVMath::IntersectionTwoLines(Point2f(1,0), BINARY_IMAGE_WIDTH / 2, upperHori.start, upperHori.direction);
		Point2f p2 = CVMath::IntersectionTwoLines(Point2f(1,0), BINARY_IMAGE_WIDTH / 2, lowerHori.start, lowerHori.direction);

		*goal = (p1 + p2) / 2;
	} else if(foundLowerHori) {
		*direction = lowerHori.direction;
		Point2f p2 = CVMath::IntersectionTwoLines(Point2f(1,0), BINARY_IMAGE_WIDTH / 2, lowerHori.start, lowerHori.direction);

		p2.y -= METER_TO_PIXEL(0.23);
		*goal = p2;
	} else if(foundUpperHori) {
		*direction = upperHori.direction;
		Point2f p1 = CVMath::IntersectionTwoLines(Point2f(1,0), BINARY_IMAGE_WIDTH / 2, upperHori.start, upperHori.direction);

		p1.y += METER_TO_PIXEL(0.23);
		*goal = p1;
	} else {
		return false;
	}

	return true;
}
