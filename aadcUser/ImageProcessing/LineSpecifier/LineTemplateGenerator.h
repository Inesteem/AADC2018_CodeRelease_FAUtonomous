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
 * $Author:: schoen $   $Date:: 2015-12-15 #$
 **********************************************************************/

#ifndef LINETEMPLATEGENERATOR_H_
#define LINETEMPLATEGENERATOR_H_

#include "LineTemplate.h"

class LineTemplateGenerator {
public:
	static const int METER_TO_PIXEL;
	static const int DEFAULT_TEMPLATE_HEIGHT;
	static const float AVERAGE_LANE_WIDTH;

	static const Scalar DEFAULT_COLOR_VALUE;

	static const Rect bitmaskSize;

	enum TURN{
		LEFT,
		RIGHT
	};

	enum SIDE {
		LINE_ON_RIGHT_SIDE,
		LINE_ON_LEFT_SIDE
	};

	LineTemplateGenerator();
	virtual ~LineTemplateGenerator();

	/**
	 * draws some small circle parts in matrices of the given size in firstTemplate.
	 * The initial Region of Interest is defined in firstTemplate. All other region of interest rely on this.
	 * The initial line center is defined in firstTemplate.
	 * @param lineTemplates a vector containing the created templates. Templates are save at startIndex, and there are elementCount templates created
	 * @param firstTemplate
	 * @param startIndex
	 * @param elementCount
	 * @param startAngle the angle of the tangent on the circle.
	 * @param radius the radius of the circle
	 * @param lineWidth a line width in pixel
	 * @param turn create a left or right turn template
	 * @param side determines whether the line is on the right or left side of the viewing direction
	 * @param averageLineWidth
	 */
	static void Generate(std::vector<LineTemplate> & lineTemplates, LineTemplate firstTemplate, int startIndex,
			int elementCount, float startAngle, float radius, int lineWidth, TURN turn, SIDE side, float averageLineWidth = AVERAGE_LANE_WIDTH, float lineLocationY_Offset = 0);

	static void GenerateStraight(std::vector<LineTemplate> & lineTemplates, LineTemplate firstTemplate, Point2f offset, int startIndex, int elementCount, int lineWidth, SIDE side);

private:
	static float IntersectCircle(cv::Point2f center, float radius, float line_y, TURN turn);
};

#endif /* LINETEMPLATEGENERATOR_H_ */
