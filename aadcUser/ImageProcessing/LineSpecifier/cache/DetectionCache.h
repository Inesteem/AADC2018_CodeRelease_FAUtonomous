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
 * $Author:: schoen $   $Date:: 2016-03-17 #$
 **********************************************************************/

#ifndef CACHE_DETECTIONCACHE_H_
#define CACHE_DETECTIONCACHE_H_

#include "CVMath.h"

class DetectionCache {
	DetectionCache() {}

	DetectionCache(DetectionCache const&);
    DetectionCache& operator=(DetectionCache const&);

    struct {
    	CVMath::LineSegment d;
    	long long int time;
    } rightLine;

    struct {
    	CVMath::LineSegment d;
    	long long int time;
    } outerLeftLine;

    struct {
    	CVMath::LineSegment d;
    	long long int time;
    } crossingLine;

public:
    static DetectionCache& GetInstance()
    {
        static DetectionCache instance;
        return instance;
    }

    void SaveRightLine(CVMath::LineSegment data, long long int time) {
    	rightLine.d = data;
    	rightLine.time = time;
    }

    CVMath::LineSegment GetRightLine(long long int time) {
    	if(time == rightLine.time) {
    		return rightLine.d;
    	} else {
    		return CVMath::LineSegment();
    	}
    }

    void SaveOuterLeftLine(CVMath::LineSegment data, long long int time) {
    	outerLeftLine.d = data;
    	outerLeftLine.time = time;
    }

    CVMath::LineSegment GetOuterLeftLine(long long int time) {
    	if(time == outerLeftLine.time) {
    		return outerLeftLine.d;
    	} else {
    		return CVMath::LineSegment();
    	}
    }

    void SaveCrossingLine(CVMath::LineSegment data, long long int time) {
    	outerLeftLine.d = data;
    	outerLeftLine.time = time;
    }

    CVMath::LineSegment GetCrossingLine(long long int time) {
    	if(time == outerLeftLine.time) {
    		return outerLeftLine.d;
    	} else {
    		return CVMath::LineSegment();
    	}
    }
};

#endif /* CACHE_DETECTIONCACHE_H_ */
