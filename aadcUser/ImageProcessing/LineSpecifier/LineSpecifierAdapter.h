/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. Team FAUtonomous. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/

#pragma once
#define CID_LINE_SPECIFIER_ADAPTER_FILTER "line_specifier.filter.user.aadc.cid"


#include "stdafx.h"
#include "PoseCache.h"

// namespaces
using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

class LineSpecifier;
class StopLineDetector;
class PoseCache;
class ParkingSpotDetector;
class IntersectionDetector;
class SpeedRecommender;

// LineSpecifierAdapter
class LineSpecifierAdapter : public cTriggerFunction
{
private:
    TSignalValue    m_SteeringSignalId;
    TSignalValue    m_SpeedSignalId;
    TActionStruct   m_ActionId;
    TFeedbackStruct m_FeedbackId;
    TPoseStruct     m_PoseStructGoalId;
    TPoseStruct     m_PoseStructInId;
    TRoadSignExt    m_TrafficSignId;


    /*-------------- PINS --------------*/
    // create pins for input and output
    // coding convention:	m_Reader******	for input  signals
    //			m_Writer******	for output signals

    // input pins
    cPinReader m_ReaderBinaryVideo;
    cPinReader m_ReaderAction;
    cPinReader m_ReaderTrafficSign;
    cPinReader m_ReaderPose;

    // output pins
    cPinWriter m_WriterSteeringAngle;
    cPinWriter m_WriterLocalGoal;
    cPinWriter m_WriterSpeed;
    cPinWriter m_WriterFeedback;


    /*------------ PROPERTIES ----------*/
    // set property variables
    // coding convention:	m_prop******

    adtf::base::property_variable<tFloat32> m_propStoplineOffsetX         = tFloat(-0.6f);
    adtf::base::property_variable<tFloat32> m_propStoplineOffsetY         = tFloat( 0.0f);
    adtf::base::property_variable<tFloat32> m_propStoplineMinWidth        = tFloat( 0.025f);
    adtf::base::property_variable<tFloat32> m_propStoplineMaxWidth        = tFloat( 0.076f);
    adtf::base::property_variable<tFloat32> m_propStoplineMaxXYRatio      = tFloat( 1.0f);
    adtf::base::property_variable<tFloat32> m_propStoplineYawScale        = tFloat( 0.7f);
    adtf::base::property_variable<tFloat32> m_propStoplineTransmission    = tFloat( 1.0f);

    adtf::base::property_variable<tFloat32> m_propEmergencyDistance       = tFloat(-0.1f);
    adtf::base::property_variable<tFloat32> m_propSteeringOffsetX         = tFloat( 0.35f);
    adtf::base::property_variable<tFloat32> m_propSteeringOffsetY         = tFloat( 0.0f);
    adtf::base::property_variable<tFloat32> m_propSteeringLeftOffsetX     = tFloat( 0.35f);
    adtf::base::property_variable<tFloat32> m_propSteeringLeftOffsetY     = tFloat( 0.0f);
    adtf::base::property_variable<tFloat32> m_propSteeringAngleScaleLeft  = tFloat( 1.0f);
    adtf::base::property_variable<tFloat32> m_propSteeringAngleScaleRight = tFloat( 1.0f);

    adtf::base::property_variable<tFloat32> m_propParkingTransOffsetX     = tFloat(-0.1f);
    adtf::base::property_variable<tFloat32> m_propParkingTransOffsetY     = tFloat( 0.25f);
    // pull out parking trans offset x:
    adtf::base::property_variable<tFloat32> m_propPUParkingTransOffsetX   = tFloat(-1.1f);
    adtf::base::property_variable<tFloat32> m_propParkingLongOffsetX      = tFloat(-0.1f);
    adtf::base::property_variable<tFloat32> m_propParkingLongOffsetY      = tFloat( 0.25f);
    adtf::base::property_variable<tFloat32> m_propBackwardsOffsetX        = tFloat(-1.0f);
    adtf::base::property_variable<tFloat32> m_propBackwardsOffsetY        = tFloat( 0.0f);

    adtf::base::property_variable<tFloat32> m_propSpeedStraightSlowFar    = tFloat( 1.1f);
    adtf::base::property_variable<tFloat32> m_propSpeedStraightSlowNear   = tFloat( 0.8f);
    adtf::base::property_variable<tFloat32> m_propSpeedStraightNormalFar  = tFloat( 1.1f);
    adtf::base::property_variable<tFloat32> m_propSpeedStraightNormalNear = tFloat( 0.8f);
    adtf::base::property_variable<tFloat32> m_propSpeedMin                = tFloat( 0.5f);
    adtf::base::property_variable<tFloat32> m_propSpeedCurve              = tFloat( 0.7f);
    adtf::base::property_variable<tFloat32> m_propSpeedDetection          = tFloat( 0.6f);

    adtf::base::property_variable<tUInt32> m_propStartCommand = tUInt32(0.0f);

    /*------------ VARIABLES -----------*/
    // coding convention:	m_******

    cv::Mat    m_MatImage;          // our image from camera, converted to a cv::mat
    tBool      m_bOnLeftLane;       // bool, if we on left lane
    tFloat32   m_f32SteeringAngleScaleLeft;
    tFloat32   m_f32SteeringAngleScaleRight;
    tFloat32   m_f32CurrentSteeringAngle;
    tTimeStamp m_tsLastVideoTime;
    tBool      m_bFirstParkingDetection;
    volatile tBool m_bCheckTrafficSign = false;
    TActionStruct::Data m_Action;

    // pointer to the class which implements the lane tracker
    LineSpecifier        *m_cLineSpecifier;
    PoseCache            *m_cCarPoses;
    StopLineDetector     *m_cStopLineDetector;
    ParkingSpotDetector  *m_cParkingDetector;
    IntersectionDetector *m_cIntersectionDetector;
    SpeedRecommender     *m_cSpeedController;

    // image formats for video pins
    adtf::streaming::tStreamImageFormat m_sInputImageFormat;

    // The reference clock
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    // -----------------------------
    // structs

    struct Offset
    {
        /**
         * car position in the image frame. in Meter.
         */
        cv::Point2f steering;

        cv::Point2f steeringLeft;

        /**
         * stopline offset
         */
        cv::Point2f stopline;
        tFloat32 yawScale;
        tFloat32 maxXYRation_Stopline;

        /**
         * parking offset
         */
        cv::Point2f parkingTrans;

        /**
         * parking offset
         */
        cv::Point2f parkingLong;
        cv::Point2f backwards;
        tFloat32 pullOutXOffset;
        tFloat32 stopTransmissionDistance;
    } offset;

    struct Sign
    {
        float distance;
        bool  haveWay;

        Sign()
        {
            distance = -1;
            haveWay = false;
        }
    } _prioritySign;


    struct LastIntersection
    {
        TFeedbackStruct::Data feedback;
        tTimeStamp time;
        CarPose onEnterIntersection;
        tBool processHint;
        tFloat32 yawIntegral;
        tBool badCrossingType;

        LastIntersection()
        {
            time = -1;
            processHint = tFalse;
            yawIntegral = 0;
            badCrossingType = tFalse;
        }

        tBool Valid()
        {
            if(this->time != -1)
            {
                return tTrue;
            }
            else
            {
                return tFalse;
            }
        }

        tVoid Clear()
        {
            time = -1;
        }

        tBool ProcessHint(tBool value)
        {
            tBool last = processHint;
            if(Valid())
            {
                processHint = value;
            }
            else
            {
                processHint = tFalse;
            }
            return last;
        }

        tVoid Save(CarPose intersectionPoint, tTimeStamp time)
        {
            onEnterIntersection = intersectionPoint;
            this->time = time;
        }

    private:
        tInt32 CheckTurn(tFloat32 currentYaw)
        {
            //tFloat32 diff = onEnterIntersection.yaw - currentYaw;
            //TODO:
            return 0;
        }
    } _lastIntersection;


public:
    /*------------ FUNCTIONS -----------*/
    /*------------ FROM AUDI -----------*/
    // constructor
    LineSpecifierAdapter();

    // destructor
    ~LineSpecifierAdapter() = default;

    // Configure
    virtual tResult Configure() override;

    // Process
    virtual tResult Process(tTimeStamp tmTimeOfTrigger) override;

    // ---------- FROM USER ------------*/
    tResult ProcessAction           (TActionStruct::Data inputAction);
    tResult ProcessMatImage         ();
    tResult ProcessStoplineDetection(cv::Mat &debugImage, cv::Point2f frameCoord, cv::Point2f image_destination,
                                     tFloat32 nearPlane, tTimeStamp outputTime);
    tResult ProcessParkingDetection (cv::Mat &debugImage, cv::Point2f frameCoord, tTimeStamp outputTime);
    tResult ProcessPullOut          (cv::Mat &debugImage, cv::Point2f frameCoord, tTimeStamp outputTime);
    tResult ProcessIntersection     (cv::Mat &debugImage, cv::Point2f frameCoord, tFloat32 nearPlane, tTimeStamp outputTime);
    tResult ProcessBackwards        (cv::Mat &debugImage, Point2f frameCoord, cv::Point2f image_destination, tTimeStamp outputTime);
    tResult PropertyChanged         (const tChar* strName);
    tResult ReadAsFloatAndClear     (cv::Mat &image, tUInt32 bytePosition, tFloat32 *value);
    TPoseStruct::Data DetermineWorldStopPoint(Point2f stopPosition, Point2f image_yawDirection);
    tResult ChangeType(adtf::streaming::cDynamicSampleReader& reader, adtf::streaming::tStreamImageFormat& inputFormat,
                       const adtf::streaming::ant::IStreamType& oType);
}; // LineSpecifierAdapter
