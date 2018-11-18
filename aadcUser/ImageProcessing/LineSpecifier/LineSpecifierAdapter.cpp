/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. Team FAUtonomous. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

TODO: write a short description of your filter!!
This filter is our template filter for aadc2018.
Use this filter for creating your own filters.
Keep in mind to change "cTemplateFilter" to your filtername.

**********************************************************************/

#include "stdafx.h"
#include "LineSpecifierAdapter.h"
#include "PoseCache.h"
#include "LineSpecifier.h"
#include "cache/DetectionCache.h"
#include "speed/SpeedRecommender.h"
#include "stop/StopLineDetector.h"
#include "parking/ParkingSpotDetector.h"
#include "crossing/IntersectionDetector.h"
#include "ImageProcessingUtils.h"
#include "ScmCommunication.h"
#include "aadc_roadSign_enums.h"
#include <iostream>

#define LS_STOPLINE_OFFSET_X      "stopline::offset_x"
#define LS_STOPLINE_OFFSET_Y      "stopline::offset_y"
#define LS_STOPLINE_MIN_WIDTH     "stopline::min width"
#define LS_STOPLINE_MAX_WIDTH     "stopline::max width"
#define LS_STOPLINE_MAX_Y_X_RATIO "stopline::max_y_x ratio"
#define LS_STOPLINE_YAW_SCALE     "stopline::yaw scale"
#define LS_STOPLINE_TRANSMISSION  "stopline::transmission_distance"

#define LS_HALT_EMERGENCY_DISTANCE "halt::emergency_distance"
#define LS_STEERING_OFFSET_X       "steering::offset x"
#define LS_STEERING_OFFSET_Y       "steering::offset_y"

#define LS_STEERING_LEFT_OFFSET_X "steering::leftlane_offset x"
#define LS_STEERING_LEFT_OFFSET_Y "steering::leftlane_offset_y"

#define LS_STEERING_ANGLE_SCALE_LEFT  "steering::angle scale left"
#define LS_STEERING_ANGLE_SCALE_RIGHT "steering::angle scale right"

#define LS_PARKING_TRANS_OFFSET_X "parkingTrans::offset_x"
#define LS_PARKING_TRANS_OFFSET_Y "parkingTrans::offset_y"

#define LS_PULL_OUT_PARKING_TRANS_OFFSET_X "pullOutParkingTrans::offset_x"

#define LS_PARKING_LONG_OFFSET_X "parkingLong::offset_x"
#define LS_PARKING_LONG_OFFSET_Y "parkingLong::offset_y"

#define LS_BACKWARDS_OFFSET_X "backwards::offset_x"
#define LS_BACKWARDS_OFFSET_Y "backwards::offset_y"

#define LS_SPEED_STRAIGHT_SLOW_FAR    "speed::straight far slow"
#define LS_SPEED_STRAIGHT_SLOW_NEAR   "speed::straight near_slow"
#define LS_SPEED_STRAIGHT_NORMAL_FAR  "speed::straight far normal"
#define LS_SPEED_STRAIGHT_NORMAL_NEAR "speed::straight near_normal"
#define LS_SPEED_MIN                  "speed::min"
#define LS_SPEED_CURVE                "speed::curve"
#define LS_SPEED_DETECTION            "speed::detection"

#define LS_START_COMMAND "Debug::start action command"
#define LS_SAVE_TO_DISK  "Debug::save to disk"

#define LOCAL_IMAGE_COPY 0
#define FEATURE_TESTING 1

#define PRIORITY_SIGN_MIN_DISTANCE -0.2


// This will define the filter and expose it via plugin class factory.
// Class cTemplateFilter will be embedded to the filter and called by
// data trigger of "speed" by default.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
    CID_LINE_SPECIFIER_ADAPTER_FILTER,		        // references to header file
    "LineSpecifierAdapter",                         // label
    LineSpecifierAdapter,                           // class
    adtf::filter::pin_trigger({"inputAction", "inputVideo"}));	// set trigger pin


// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
LineSpecifierAdapter::LineSpecifierAdapter()
{
    // register pins
    m_ActionId      .registerPin(this, m_ReaderAction     , "inputAction" );
    m_TrafficSignId .registerPin(this, m_ReaderTrafficSign, "inputTrafficSign");
    m_PoseStructInId.registerPin(this, m_ReaderPose       , "inputPoseStruct");

    m_SpeedSignalId   .registerPin(this, m_WriterSpeed        , "outputSpeed" );
    m_SteeringSignalId.registerPin(this, m_WriterSteeringAngle, "outputSteering");
    m_FeedbackId      .registerPin(this, m_WriterFeedback     , "outputFeedback" );
    m_PoseStructGoalId.registerPin(this, m_WriterLocalGoal    , "outputLocalGoal");

    //create and set inital input format type
    m_sInputImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(GREYSCALE_8);
    adtf::ucom::object_ptr<IStreamType> pTypeInput = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pTypeInput, m_sInputImageFormat);
    //Register input pin
    Register(m_ReaderBinaryVideo, "inputVideo", pTypeInput);
    //register callback for type changes
    m_ReaderBinaryVideo.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
    {
        return ChangeType(m_ReaderBinaryVideo, m_sInputImageFormat, *pType.Get());
    });

    // -----------------------------------------
    // set property variables
    RegisterPropertyVariable(LS_BACKWARDS_OFFSET_X             , m_propBackwardsOffsetX);
    RegisterPropertyVariable(LS_BACKWARDS_OFFSET_Y             , m_propBackwardsOffsetY);
    RegisterPropertyVariable(LS_HALT_EMERGENCY_DISTANCE        , m_propEmergencyDistance);
    RegisterPropertyVariable(LS_PARKING_LONG_OFFSET_X          , m_propParkingLongOffsetX);
    RegisterPropertyVariable(LS_PARKING_LONG_OFFSET_Y          , m_propParkingLongOffsetY);
    RegisterPropertyVariable(LS_PARKING_TRANS_OFFSET_X         , m_propParkingTransOffsetX);
    RegisterPropertyVariable(LS_PARKING_TRANS_OFFSET_Y         , m_propParkingTransOffsetY);
    RegisterPropertyVariable(LS_PULL_OUT_PARKING_TRANS_OFFSET_X, m_propPUParkingTransOffsetX);
    RegisterPropertyVariable(LS_SPEED_CURVE                    , m_propSpeedCurve);
    RegisterPropertyVariable(LS_SPEED_DETECTION                , m_propSpeedDetection);
    RegisterPropertyVariable(LS_SPEED_MIN                      , m_propSpeedMin);
    RegisterPropertyVariable(LS_SPEED_STRAIGHT_NORMAL_FAR      , m_propSpeedStraightNormalFar);
    RegisterPropertyVariable(LS_SPEED_STRAIGHT_NORMAL_NEAR     , m_propSpeedStraightNormalNear);
    RegisterPropertyVariable(LS_SPEED_STRAIGHT_SLOW_FAR        , m_propSpeedStraightSlowFar);
    RegisterPropertyVariable(LS_SPEED_STRAIGHT_SLOW_NEAR       , m_propSpeedStraightSlowNear);
    RegisterPropertyVariable(LS_START_COMMAND                  , m_propStartCommand);
    RegisterPropertyVariable(LS_STEERING_ANGLE_SCALE_LEFT      , m_propSteeringAngleScaleLeft);
    RegisterPropertyVariable(LS_STEERING_ANGLE_SCALE_RIGHT     , m_propSteeringAngleScaleRight);
    RegisterPropertyVariable(LS_STEERING_LEFT_OFFSET_X         , m_propSteeringLeftOffsetX);
    RegisterPropertyVariable(LS_STEERING_LEFT_OFFSET_Y         , m_propSteeringLeftOffsetY);
    RegisterPropertyVariable(LS_STEERING_OFFSET_X              , m_propSteeringOffsetX);
    RegisterPropertyVariable(LS_STEERING_OFFSET_Y              , m_propSteeringOffsetY);
    RegisterPropertyVariable(LS_STOPLINE_MAX_WIDTH             , m_propStoplineMaxWidth);
    RegisterPropertyVariable(LS_STOPLINE_MAX_Y_X_RATIO         , m_propStoplineMaxXYRatio);
    RegisterPropertyVariable(LS_STOPLINE_MIN_WIDTH             , m_propStoplineMinWidth);
    RegisterPropertyVariable(LS_STOPLINE_OFFSET_X              , m_propStoplineOffsetX);
    RegisterPropertyVariable(LS_STOPLINE_OFFSET_Y              , m_propStoplineOffsetY);
    RegisterPropertyVariable(LS_STOPLINE_TRANSMISSION          , m_propStoplineTransmission);
    RegisterPropertyVariable(LS_STOPLINE_YAW_SCALE             , m_propStoplineYawScale);

    // set class objects
    m_cLineSpecifier        = new LineSpecifier();
    m_cStopLineDetector     = new StopLineDetector(METER_TO_PIXEL(m_propStoplineMinWidth),
                                                   METER_TO_PIXEL(m_propStoplineMaxWidth));
    m_cParkingDetector      = new ParkingSpotDetector();
    m_cIntersectionDetector = new IntersectionDetector(METER_TO_PIXEL(m_propEmergencyDistance));
    m_cCarPoses             = new PoseCache();
    m_cSpeedController      = new SpeedRecommender();

    // set offset
    offset.steering       = Point2f(m_propSteeringOffsetX    , m_propSteeringOffsetY);
    offset.steeringLeft   = Point2f(m_propSteeringLeftOffsetX, m_propSteeringLeftOffsetY);
    offset.parkingLong    = Point2f(m_propParkingLongOffsetX , m_propParkingLongOffsetY);
    offset.parkingTrans   = Point2f(m_propParkingTransOffsetX, m_propParkingTransOffsetY);
    offset.backwards      = Point2f(m_propBackwardsOffsetX   , m_propBackwardsOffsetY);
    offset.stopline       = Point2f(m_propStoplineOffsetX    , m_propStoplineOffsetY);
    offset.maxXYRation_Stopline     = m_propStoplineMaxXYRatio;
    offset.yawScale                 = m_propStoplineYawScale;
    offset.pullOutXOffset           = m_propPUParkingTransOffsetX;
    offset.stopTransmissionDistance = m_propStoplineTransmission;

    // set steering
    m_f32SteeringAngleScaleLeft  = m_propSteeringAngleScaleLeft;
    m_f32SteeringAngleScaleRight = m_propSteeringAngleScaleRight;

    // set speedcontroller
    m_cSpeedController->speedConfig.straightSlowNear   = m_propSpeedStraightSlowNear;
    m_cSpeedController->speedConfig.straightNormalNear = m_propSpeedStraightNormalNear;
    m_cSpeedController->speedConfig.straightSlowFar    = m_propSpeedStraightSlowFar;
    m_cSpeedController->speedConfig.straightNormalFar  = m_propSpeedStraightNormalFar;
    m_cSpeedController->speedConfig.min                = m_propSpeedMin;
    m_cSpeedController->speedConfig.curve              = m_propSpeedCurve;
    m_cSpeedController->speedConfig.detection          = m_propSpeedDetection;

    try
    {
        m_cLineSpecifier->Init();
    }
    catch (cv::Exception &e)
    {
        LOG_ERROR(e.msg.c_str());
    }

    // set start command
    m_Action.ui32Command = m_propStartCommand;
    if (m_Action.ui32Command >= 1000 && m_Action.ui32Command < 2000)
    {
        m_Action.bEnabled    = tTrue;
        m_Action.bStarted    = tTrue;
    }

    m_tsLastVideoTime = 0;
    m_bOnLeftLane     = tFalse;
    m_bFirstParkingDetection = tTrue;
}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult LineSpecifierAdapter::Configure()
{
    // done
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}


// ------------------------------
// ---------- PROCESS -----------
// ------------------------------
// process() is always called each time a trigger occured (new data sample arrive).
// IMPORTANT: Make sure to read ALL samples of ALL readers!
// This is how to read a sample
tResult LineSpecifierAdapter::Process(tTimeStamp tmTimeOfTrigger)
{
    // read from pose pin
    TPoseStruct::Data poseIn;
    if(IS_OK(m_PoseStructInId.readPin(m_ReaderPose, (void *) &poseIn)))
    {
        CarPose pose = CarPose(Point2f(poseIn.f32PosX, poseIn.f32PosY), poseIn.f32Yaw,
                               tmTimeOfTrigger, poseIn.ui32ArduinoTimestamp);
        m_cCarPoses->PushFront(pose);

        if (m_tsLastVideoTime > 0)
        {
            tFloat32 f32lastDistance = m_cCarPoses->GetLastDistance();
            if (f32lastDistance > 0)
            {
                _prioritySign.distance -= f32lastDistance;
            }
            else
            {
                _prioritySign.distance -= (tmTimeOfTrigger - m_tsLastVideoTime) * 0.000001 * m_cSpeedController->lastSpeed;
            }
            // just to be sure
            if (_prioritySign.distance < PRIORITY_SIGN_MIN_DISTANCE)
            {
                _prioritySign.distance = -1;
            }
        }
    }

    // read from trafficsign pin
    TTrafficSign::Data trafficSignIn;
    if(IS_OK(m_TrafficSignId.readPin(m_ReaderTrafficSign, (void *) &trafficSignIn)))
    {
        m_bCheckTrafficSign = true;
    }

    // read action
    TActionStruct::Data actionIn;
    if(IS_OK(m_ActionId.readPin(m_ReaderAction, (void *) &actionIn)))
    {
        ProcessAction(actionIn);
    }

    // read video
    object_ptr<const ISample> pReadVideoSample;
    if (IS_OK(m_ReaderBinaryVideo.GetNextSample(pReadVideoSample)))
    {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        //lock read buffer
        if (IS_OK(pReadVideoSample->Lock(pReadBuffer)))
        {
            //create a opencv matrix from the media sample buffer
            Mat matFromVideoSample(cv::Size(m_sInputImageFormat.m_ui32Width, m_sInputImageFormat.m_ui32Height),
                                   CV_8UC1, (uchar*)pReadBuffer->GetPtr());

            m_MatImage = matFromVideoSample;

            ProcessMatImage();
        }

    }

    // done
    RETURN_NOERROR;
}


// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------

// ProcessAction
tResult LineSpecifierAdapter::ProcessAction(TActionStruct::Data inputAction)
{
    // create feedback
    TFeedbackStruct::Data feedback;
    feedback.ui8FilterId = F_LINE_SPECIFIER;

    m_bFirstParkingDetection = tTrue;
    _prioritySign.distance = -1;

    // if action not enabled or started (we are probably in the beginning)
    if (inputAction.bEnabled == tFalse || inputAction.bStarted == tFalse)
    {
        // set STOP as command
        m_Action.ui32Command    = AC_LS_STOP;
        m_cLineSpecifier       ->SetFirstPoint(tTrue);
        m_cStopLineDetector    ->ClearDetectedStopPositions();
        m_cStopLineDetector    ->SetTrackingMode(tFalse);
        m_cIntersectionDetector->ClearDetectedStopPositions();
        m_cIntersectionDetector->SetTrackingMode(tFalse);
    }

    // if stop, then stop!! :)
    else if (inputAction.ui32Command == AC_LS_STOP)
    {
        m_bOnLeftLane = tFalse;

        // transmit feedback
        feedback.ui32FeedbackStatus = FB_LS_STOPPED;
        RETURN_IF_FAILED(m_FeedbackId.writePin(m_WriterFeedback, (void *) &feedback,
                                               m_pClock->GetStreamTime()));
    }

    // if command is no speed
    else if (inputAction.ui32Command == AC_LS_NOSPEED)
    {
        m_bOnLeftLane = tFalse;
        _lastIntersection.yawIntegral = 0;
        feedback.ui32FeedbackStatus = FB_LS_NOSPEED;
        RETURN_IF_FAILED(m_FeedbackId.writePin(m_WriterFeedback, (void *) &feedback,
                                               m_pClock->GetStreamTime()));
    }

    // command no speed, but car on the left lane
    else if (inputAction.ui32Command == AC_LS_LEFT_NOSPEED)
    {
        m_bOnLeftLane = tTrue;
        _lastIntersection.yawIntegral = 0;
        feedback.ui32FeedbackStatus = FB_LS_LEFT_NOSPEED;
        RETURN_IF_FAILED(m_FeedbackId.writePin(m_WriterFeedback, (void *) &feedback,
                                               m_pClock->GetStreamTime()));
    }

    // command slow on the right lane
    else if (inputAction.ui32Command == AC_LS_SLOW_RIGHTLANE)
    {
        m_bOnLeftLane = tFalse;
        _lastIntersection.yawIntegral = 0;
        feedback.ui32FeedbackStatus = FB_LS_SLOW_RIGHTLANE;
        RETURN_IF_FAILED(m_FeedbackId.writePin(m_WriterFeedback, (void *) &feedback,
                                               m_pClock->GetStreamTime()));
    }

    // command normal on right lane
    else if (inputAction.ui32Command == AC_LS_NORMAL_RIGHTLANE)
    {
        m_bOnLeftLane = tFalse;
        _lastIntersection.yawIntegral = 0;
        feedback.ui32FeedbackStatus = FB_LS_NORMAL_RIGHTLANE;
        RETURN_IF_FAILED(m_FeedbackId.writePin(m_WriterFeedback, (void *) &feedback,
                                               m_pClock->GetStreamTime()));
    }

    // command slow on left lane
    else if (inputAction.ui32Command == AC_LS_SLOW_LEFTLANE)
    {
        m_bOnLeftLane = tTrue;
        _lastIntersection.yawIntegral = 0;
        feedback.ui32FeedbackStatus = FB_LS_SLOW_LEFTLANE;
        RETURN_IF_FAILED(m_FeedbackId.writePin(m_WriterFeedback, (void *) &feedback,
                                               m_pClock->GetStreamTime()));
    }

    // command normal on left lane
    else if (inputAction.ui32Command == AC_LS_NORMAL_LEFTLANE)
    {
        m_bOnLeftLane = tTrue;
        _lastIntersection.yawIntegral = 0;
        feedback.ui32FeedbackStatus = FB_LS_NORMAL_LEFTLANE;
        RETURN_IF_FAILED(m_FeedbackId.writePin(m_WriterFeedback, (void *) &feedback,
                                               m_pClock->GetStreamTime()));
    }

    // command detect parking spot (long)
    else if (inputAction.ui32Command == AC_LS_DETECT_LONG_PARKING_SPOT_SLOW)
    {
        if (m_Action.ui32Command == AC_LS_DETECT_LONG_PARKING_SPOT_SLOW)
        {
            m_bFirstParkingDetection = tFalse;
        }
        _lastIntersection.yawIntegral = 0;
    }

    // command detect parking spot (trans)
    else if (inputAction.ui32Command == AC_LS_DETECT_TRANS_PARKING_SPOT_SLOW)
    {
        if (m_Action.ui32Command == AC_LS_DETECT_TRANS_PARKING_SPOT_SLOW)
        {
            m_bFirstParkingDetection = tFalse;
        }
        _lastIntersection.yawIntegral = 0;
    }

    m_Action = inputAction;

    // done
    RETURN_NOERROR;
}

// ProcessMatImage
tResult LineSpecifierAdapter::ProcessMatImage()
{
    Point2f frameCoord;
    tFloat32 nearPlane;
    ReadAsFloatAndClear(m_MatImage,  4, &frameCoord.x);
    ReadAsFloatAndClear(m_MatImage,  8, &frameCoord.y);
    ReadAsFloatAndClear(m_MatImage, 12, &nearPlane);

    // if left lane flip image
    if(m_bOnLeftLane)
    {
        cv::flip(m_MatImage, m_MatImage, 1);
    }

    Mat debugImage(1, 1, CV_8UC3);

    /* only for debugging!
    if (debugEnabled) {
        resize(debugImage, debugImage,
                Size(BINARY_IMAGE_WIDTH, BINARY_DEBUG_IMAGE_HEIGHT), CV_8UC3);
        Mat temp;
        cvtColor(image, temp, CV_GRAY2RGB);
        copyMakeBorder(temp, debugImage, 0,
        BINARY_DEBUG_IMAGE_HEIGHT - BINARY_IMAGE_HEIGHT, 0, 0, BORDER_CONSTANT);
    }
    */

    LineSpecifier::HINT hint = LineSpecifier::NO_HINT;

    if (m_Action.ui32Command == AC_LS_STOP)
    {
        tFloat32 yawDiff = m_cCarPoses->GetYawDiff();
        _lastIntersection.yawIntegral += yawDiff;

        Point2f localDestination = ImageUtils::ConvertToWorldCoordinates(
                    m_cLineSpecifier->GetLastDestination(), frameCoord);

        if (_lastIntersection.yawIntegral < -0.10)
        {
            hint = LineSpecifier::RIGHT_TURN;
            //right turn;

            Point2f rightDir = m_cIntersectionDetector->OnRightTurn(m_MatImage, &debugImage);
            if (rightDir.x != 0 && rightDir.y != 0)
            {
                m_cLineSpecifier->SetLastDestination(rightDir, 0.5);
            }
            else if (localDestination.y > 0)
            {
                m_cLineSpecifier->SetFirstPoint(true);
            }
        }
        else if (_lastIntersection.yawIntegral > 0.10)
        {
            hint = LineSpecifier::LEFT_TURN;
            //left turn;

            Point2f leftDir = m_cIntersectionDetector->OnLeftTurn(m_MatImage, &debugImage);
            if (leftDir.x != 0 && leftDir.y != 0)
            {
                m_cLineSpecifier->SetLastDestination(leftDir, 0.9);
            }
            else if (localDestination.y < 0)
            {
                m_cLineSpecifier->SetFirstPoint(true);
            }
        }

        if (yawDiff != 0 && cv::abs(yawDiff) < 0.08)
        {
            Point2f movement = Point2f(-yawDiff * (METER_TO_PIXEL(frameCoord.x)
                                                     - BINARY_IMAGE_HEIGHT), 0);
            if (0 < movement.x && movement.x < 1)
            {
                movement.x = 1;
            }

            if (-1 < movement.x && movement.x < 0)
            {
                movement.x = -1;
            }

            m_cLineSpecifier->MoveDestPoint(movement);
        }
    }

    Point2f image_destination;
    tFloat32 probability;
    m_cLineSpecifier->DetermineDestinationPoint(m_MatImage, debugImage,
                                                &image_destination, &probability, hint);
    Point2f destination = ImageUtils::ConvertToWorldCoordinates(image_destination, frameCoord);

    m_f32CurrentSteeringAngle = m_cSpeedController->DetermineSteeringAngle(m_MatImage, &debugImage, destination,
                                                                   m_bOnLeftLane ? offset.steeringLeft : offset.steering,
                                                                   m_propSteeringAngleScaleRight, m_propSteeringAngleScaleLeft,
                                                                   m_bOnLeftLane, m_pClock->GetStreamTime());

    switch (m_Action.ui32Command)
    {
        case AC_LS_SLOW_LEFTLANE:
        case AC_LS_NORMAL_LEFTLANE:
        case AC_LS_SLOW_RIGHTLANE:
        case AC_LS_NORMAL_RIGHTLANE:
        case AC_LS_DETECT_LONG_PARKING_SPOT_SLOW:
        case AC_LS_DETECT_TRANS_PARKING_SPOT_SLOW:
        {
            if (m_bOnLeftLane)
            {
                TSignalValue::Data steeringSignalValue;
                steeringSignalValue.f32Value = - m_f32CurrentSteeringAngle;
                RETURN_IF_FAILED(m_SteeringSignalId.writePin(m_WriterSteeringAngle, (void *) &steeringSignalValue, m_pClock->GetStreamTime()));
            }
            else
            {
                TSignalValue::Data steeringSignalValue;
                steeringSignalValue.f32Value = m_f32CurrentSteeringAngle;
                RETURN_IF_FAILED(m_SteeringSignalId.writePin(m_WriterSteeringAngle, (void *) &steeringSignalValue, m_pClock->GetStreamTime()));
            }

            m_cSpeedController->DetectMaxSpeed(m_MatImage, &debugImage, m_f32CurrentSteeringAngle, m_pClock->GetStreamTime());

            tFloat32 speed = m_cSpeedController->SpeedController(probability, _prioritySign.distance,
                                                                m_Action.ui32Command, m_f32CurrentSteeringAngle);

            TSignalValue::Data speedSignalValue;
            speedSignalValue.f32Value = speed;
            RETURN_IF_FAILED(m_SpeedSignalId.writePin(m_WriterSpeed, (void *) &speedSignalValue, m_pClock->GetStreamTime()));

            break;
        }
        case AC_LS_NOSPEED:
        case AC_LS_LEFT_NOSPEED:
            // TRANSMITTING Steering Angle
            if (m_bOnLeftLane)
            {
                TSignalValue::Data steeringSignalValue;
                steeringSignalValue.f32Value = - m_f32CurrentSteeringAngle;
                RETURN_IF_FAILED(m_SteeringSignalId.writePin(m_WriterSteeringAngle, (void *) &steeringSignalValue, m_pClock->GetStreamTime()));
            }
            else
            {
                TSignalValue::Data steeringSignalValue;
                steeringSignalValue.f32Value = m_f32CurrentSteeringAngle;
                RETURN_IF_FAILED(m_SteeringSignalId.writePin(m_WriterSteeringAngle, (void *) &steeringSignalValue, m_pClock->GetStreamTime()));
            }
            m_cSpeedController->DetectMaxSpeed(m_MatImage, &debugImage,
                                              m_f32CurrentSteeringAngle, m_pClock->GetStreamTime());

            break;
        case AC_LS_NOSPEED_BACKWARDS:
            m_cSpeedController->DetectMaxSpeed(m_MatImage, &debugImage,
                                              m_f32CurrentSteeringAngle, m_pClock->GetStreamTime());
            RETURN_IF_FAILED(ProcessBackwards(debugImage, frameCoord, image_destination, m_pClock->GetStreamTime()));
            break;
    }

    switch (m_Action.ui32Command)
    {
        case AC_LS_SLOW_RIGHTLANE:
        case AC_LS_NORMAL_RIGHTLANE:
        {
            if (_prioritySign.distance > PRIORITY_SIGN_MIN_DISTANCE && _prioritySign.haveWay)
            {
                RETURN_IF_FAILED(ProcessIntersection(debugImage, frameCoord, nearPlane, m_pClock->GetStreamTime()));
            }

            else
            {
                RETURN_IF_FAILED(ProcessStoplineDetection(debugImage, frameCoord, image_destination,
                                                          nearPlane, m_pClock->GetStreamTime()));
            }
            break;
        }
        case AC_LS_DETECT_LONG_PARKING_SPOT_SLOW:
        case AC_LS_DETECT_TRANS_PARKING_SPOT_SLOW:
        case AC_LS_DETECT_LONG_PARKING_SPOT_NEXT:
        case AC_LS_DETECT_TRANS_PARKING_SPOT_NEXT:
        {
            RETURN_IF_FAILED(ProcessParkingDetection(debugImage, frameCoord, m_pClock->GetStreamTime()));
            break;
        }
        case AC_LS_GET_PARKSPOT_TYPE:
        {
            RETURN_IF_FAILED(ProcessPullOut(debugImage, frameCoord, m_pClock->GetStreamTime()));
            break;
        }
        case AC_LS_CHECK_CROSSING_LINE:
        {
            TFeedbackStruct::Data feedback;
            feedback.ui8FilterId = F_LINE_SPECIFIER;
            if (m_cIntersectionDetector->DetectCrossingLine(m_MatImage, &debugImage, m_pClock->GetStreamTime()))
            {
                feedback.ui32FeedbackStatus = FB_LS_CROSSING_LINE_HORIZONTAL;
            }
            else if (m_cIntersectionDetector->DetectStraightLine(m_MatImage, &debugImage))
            {
                feedback.ui32FeedbackStatus = FB_LS_CROSSING_LINE_VERTICAL;
            }
            else
            {
                feedback.ui32FeedbackStatus = FB_LS_CROSSING_LINE_NONE;
            }

            RETURN_IF_FAILED(m_FeedbackId.writePin(m_WriterFeedback, (void *) &feedback,
                                                   m_pClock->GetStreamTime()));
        }
    }

    m_tsLastVideoTime = m_pClock->GetStreamTime();

    // done
    RETURN_NOERROR;
}

// ReadAsFloatAndClear
tResult LineSpecifierAdapter::ReadAsFloatAndClear(cv::Mat& image, tUInt32 bytePosition,
                                                  tFloat32 *value)
{
    if (bytePosition > image.dataend - image.datastart)
    {
        RETURN_AND_LOG_ERROR_STR(ERR_INVALID_INDEX,
                "LineSpecifierAdapter::ReadAsFloatAndClear bytePosition out of bounds");
    }

    union
    {
        tUInt8 bytes[4];
        tFloat32 f;
    } byteToFloat;

    byteToFloat.bytes[0] = image.data[bytePosition + 0];
    byteToFloat.bytes[1] = image.data[bytePosition + 1];
    byteToFloat.bytes[2] = image.data[bytePosition + 2];
    byteToFloat.bytes[3] = image.data[bytePosition + 3];

    *value = byteToFloat.f;

    image.data[bytePosition + 0] = 0;
    image.data[bytePosition + 1] = 0;
    image.data[bytePosition + 2] = 0;
    image.data[bytePosition + 3] = 0;

    RETURN_NOERROR;
}

// ProcessStoplineDetection
tResult LineSpecifierAdapter::ProcessStoplineDetection(cv::Mat &debugImage, cv::Point2f frameCoord,
                                                       cv::Point2f image_destination, tFloat32 nearPlane,
                                                       tTimeStamp outputTime)
{
    Point2f car_image_position = ImageUtils::ConvertToImageCoordinates(offset.steering, frameCoord);
    Point2f image_yawDirection;

    Point2f image_stopPosition = m_cStopLineDetector->Detect(m_MatImage, debugImage, car_image_position,
                                                             image_destination, &image_yawDirection,
                                                             outputTime);

    if ((image_stopPosition.x > 0 && image_stopPosition.y > 0))
    {
        m_cStopLineDetector->SetTrackingMode(true);
        m_cSpeedController->detectionModeActive = 5; // 5 frames
    }

    if (m_cStopLineDetector->IsInTrackingMode())
    {
        Point2f localStopPosition = m_cCarPoses->InterpolatePoses(
                    m_cStopLineDetector->GetDetectedStopPositions(), frameCoord);

        if (localStopPosition.x > offset.stopTransmissionDistance)
        {
            RETURN_NOERROR;
        }

        if (localStopPosition.x == 0 && localStopPosition.y == 0)
        {
            if (m_cCarPoses->IsEmpty())
            {
                //fallback, if no car poses in the cache
                localStopPosition = ImageUtils::ConvertToWorldCoordinates(image_stopPosition, frameCoord);
            }
            else if (m_cStopLineDetector->GetDetectedStopPositions().size() > 1)
            {
                localStopPosition = m_cStopLineDetector->GetDetectedStopPositions().front().pos;

                if (localStopPosition.x == 0 && localStopPosition.y == 0)
                {
                    localStopPosition.x = offset.stopTransmissionDistance - 0.1;
                }
                image_yawDirection = Point2f(0, -1);
                RETURN_NOERROR;
            }
            else
            {
                RETURN_NOERROR;
            }
        }

        TPoseStruct::Data stop = DetermineWorldStopPoint(localStopPosition, image_yawDirection);

        if (stop.f32PosX == 0 && stop.f32PosY == 0)
        {
            RETURN_NOERROR;
        }

        m_cStopLineDetector->SetTrackingMode(false);
        m_cStopLineDetector->ClearDetectedStopPositions();

        if (m_Action.ui32Command == AC_LS_SLOW_RIGHTLANE || m_Action.ui32Command == AC_LS_NORMAL_RIGHTLANE) {
            // TRANSMITTING stop Line
            _prioritySign.distance = -1;
            RETURN_IF_FAILED(m_PoseStructGoalId.writePin(m_WriterLocalGoal, (void *) &stop, outputTime));

            // brake before loosing control
            if (stop.f32PosX < 0.3)
            {
                TSignalValue::Data speed;
                speed.f32Value = 0.2f;
                RETURN_IF_FAILED(m_SpeedSignalId.writePin(m_WriterSpeed, (void *) &speed, outputTime));
            }
            else
            {
                TSignalValue::Data speed;
                speed.f32Value = m_cSpeedController->speedConfig.min;
                RETURN_IF_FAILED(m_SpeedSignalId.writePin(m_WriterSpeed, (void *) &speed, outputTime));
            }

            if (!m_cCarPoses->IsEmpty())
            {
                //save last intersection
                _lastIntersection.Save(m_cCarPoses->GetGlobalPose(Point2f(stop.f32PosX, stop.f32PosY),
                                                                  stop.f32Yaw), outputTime);
            }

            TFeedbackStruct::Data feedback;
            feedback.ui8FilterId        = F_LINE_SPECIFIER;
            feedback.ui32FeedbackStatus = FB_LS_STOPLINE;
            RETURN_IF_FAILED(m_FeedbackId.writePin(m_WriterFeedback, (void *) &feedback, outputTime));
        }
    }
    else if (_prioritySign.distance > PRIORITY_SIGN_MIN_DISTANCE)
    {
        RETURN_IF_FAILED(ProcessIntersection(debugImage, frameCoord, nearPlane, outputTime));
    }

    RETURN_NOERROR;
}

// ProcessParkingDetection
tResult LineSpecifierAdapter::ProcessParkingDetection(cv::Mat &debugImage,
                                                      cv::Point2f frameCoord, tTimeStamp outputTime)
{
    ParkingSpotDetector::ParkingSpot parking = m_cParkingDetector->Detect(m_MatImage, &debugImage,
                                                                          m_Action.ui32Command, m_bFirstParkingDetection);

    if (parking.valid)
    {
        Point2f parkingPosition = ImageUtils::ConvertToWorldCoordinates(parking.locationOnVerticalLine, frameCoord);
        Point2f direction = cv::Point2f(-parking.verticalLine.direction.y, -parking.verticalLine.direction.x);

        if (parking.typeLong)
        {
            parkingPosition += offset.parkingLong.x * direction;
            parkingPosition += offset.parkingLong.y * CVMath::RotateCW90(-direction);
        }
        else
        {
            parkingPosition += offset.parkingTrans.x * direction;
            parkingPosition += offset.parkingTrans.y * CVMath::RotateCW90(-direction);
        }

        TPoseStruct::Data data;
        data.f32PosX = parkingPosition.x;
        data.f32PosY = parkingPosition.y;

        if(direction.x == 0)
        {
            RETURN_NOERROR;
        }
        else if (direction.x < 0)
        {
            data.f32Yaw = atan(direction.y / (-direction.x));
        }
        else
        {
            data.f32Yaw = atan(direction.y / direction.x);
        }

        // TRANSMITTING stop Line
        RETURN_IF_FAILED(m_PoseStructGoalId.writePin(m_WriterLocalGoal, (void *) &data, outputTime));

        TFeedbackStruct::Data feedback;
        feedback.ui8FilterId = F_LINE_SPECIFIER;

        if (parking.typeLong)
        {
            feedback.ui32FeedbackStatus = FB_LS_PARKING_LONG;
        }
        else
        {
            feedback.ui32FeedbackStatus = FB_LS_PARKING_TRANS;
        }

        RETURN_IF_FAILED(m_FeedbackId.writePin(m_WriterFeedback, (void *) &feedback, outputTime));
    }
    else
    {
        TPoseStruct::Data data;
        data.f32PosX = 1;
        RETURN_IF_FAILED(m_PoseStructGoalId.writePin(m_WriterLocalGoal, (void *) &data, outputTime));
    }

    RETURN_NOERROR;
}

// ProcessPullOut
tResult LineSpecifierAdapter::ProcessPullOut(cv::Mat &debugImage,
                                             cv::Point2f frameCoord, tTimeStamp outputTime)
{
    TFeedbackStruct::Data feedback;
    feedback.ui8FilterId = F_LINE_SPECIFIER;
    Point2f goal;
    Point2f directionImage;

    if (m_cParkingDetector->IsLongitudinalParkingSpot(m_MatImage, &debugImage))
    {
        feedback.ui32FeedbackStatus = FB_LS_IN_PARKSPOT_LONG;
        RETURN_IF_FAILED(m_FeedbackId.writePin(m_WriterFeedback, (void *) &feedback, outputTime));
    }
    else if (m_cParkingDetector->IsTransversalParkingSpot(m_MatImage, &debugImage,
                                                          &goal, &directionImage))
    {
        Point2f goalWorld = ImageUtils::ConvertToWorldCoordinates(goal, frameCoord);

        Point2f direction = cv::Point2f(-directionImage.y, -directionImage.x); //Transform to world
        direction = CVMath::RotateCW90(-direction);

        tFloat32 pullOutDistance = direction.dot(goalWorld) + offset.pullOutXOffset;

        TPoseStruct::Data stop;
        stop.f32PosX = direction.x * pullOutDistance;
        stop.f32PosY = direction.y * pullOutDistance;

        if (direction.x == 0)
        {
            stop.f32Yaw = 0;
        }
        else if (direction.x < 0)
        {
            stop.f32Yaw = atan(direction.y / (-direction.x));
        }
        else
        {
            stop.f32Yaw = atan(direction.y / direction.x);
        }

        RETURN_IF_FAILED(m_PoseStructGoalId.writePin(m_WriterLocalGoal, (void *) &stop, outputTime));

        feedback.ui32FeedbackStatus = FB_LS_IN_PARKSPOT_TRANS;
        RETURN_IF_FAILED(m_FeedbackId.writePin(m_WriterFeedback, (void *) &feedback, outputTime));
    }
    else
    {
        feedback.ui32FeedbackStatus = FB_LS_IN_PARKSPOT_LONG;
        RETURN_IF_FAILED(m_FeedbackId.writePin(m_WriterFeedback, (void *) &feedback, outputTime));
    }

    RETURN_NOERROR;
}

// ProcessIntersection
tResult LineSpecifierAdapter::ProcessIntersection(cv::Mat &debugImage, cv::Point2f frameCoord,
                                                  tFloat32 nearPlane, tTimeStamp outputTime)
{
    Point2f image_yawDirection;
    Point2f image_stopPosition = m_cIntersectionDetector->DetectIntersection(m_MatImage, &debugImage, METER_TO_PIXEL(_prioritySign.distance),
                                                                             METER_TO_PIXEL(nearPlane), &image_yawDirection,
                                                                             &_lastIntersection.badCrossingType, outputTime);

    if ((image_stopPosition.x > 0 && image_stopPosition.y > 0))
    {
        m_cIntersectionDetector->SetTrackingMode(true);
        m_cSpeedController->detectionModeActive = 5; // 5 frames
    }

    if (m_cIntersectionDetector->IsInTrackingMode())
    {
        Point2f localStopPosition = m_cCarPoses->InterpolatePoses(m_cIntersectionDetector->GetDetectedStopPositions(), frameCoord);

        if (localStopPosition.x > offset.stopTransmissionDistance)
        {
            RETURN_NOERROR;
        }

        if (localStopPosition.x == 0 && localStopPosition.y == 0)
        {
            if (m_cCarPoses->IsEmpty())
            {
                //fallback, if no car poses in the cache
                localStopPosition = ImageUtils::ConvertToWorldCoordinates(image_stopPosition, frameCoord);
            }
            else if (m_cIntersectionDetector->GetDetectedStopPositions().size() > 1)
            {
                //fix bug
                localStopPosition = m_cIntersectionDetector->GetDetectedStopPositions().front().pos;


                if (localStopPosition.x == 0 && localStopPosition.y == 0)
                {
                    localStopPosition.x = offset.stopTransmissionDistance - 0.1;
                }
                image_yawDirection = Point2f(0, -1);

                RETURN_NOERROR;
            }
            else
            {
                RETURN_NOERROR;
            }
        }

        TPoseStruct::Data stop = DetermineWorldStopPoint(localStopPosition, image_yawDirection);

        if (stop.f32PosX == 0 && stop.f32PosY == 0)
        {
            RETURN_NOERROR;
        }

        m_cIntersectionDetector->SetTrackingMode(false);
        m_cIntersectionDetector->ClearDetectedStopPositions();

        if (_lastIntersection.badCrossingType)
        {
            if (stop.f32PosX < 0)
            {
                stop.f32PosX = 0;
                stop.f32PosY = 0;
            }
        }
        else
        {
            if (stop.f32PosX < 0)
            {
                stop.f32PosX = 0;
                stop.f32PosY = 0;
            }
        }
        if (m_Action.ui32Command == AC_LS_SLOW_RIGHTLANE || m_Action.ui32Command == AC_LS_NORMAL_RIGHTLANE)
        {
            // TRANSMITTING stop Line
            _prioritySign.distance = -1;

            // brake before loosing control
            if (stop.f32PosX < 0)
            {
                if (_lastIntersection.badCrossingType)
                {
                    stop.f32PosX *= 1.2;
                }
                else
                {
                    stop.f32PosX = 0;
                    stop.f32PosY = 0;
                }

                TSignalValue::Data speedSignalValue;
                RETURN_IF_FAILED(m_SpeedSignalId.writePin(m_WriterSpeed, (void *) &speedSignalValue, outputTime));
            }
            else if (stop.f32PosX < 0.3)
            {
                TSignalValue::Data speedSignalValue;
                speedSignalValue.f32Value = 0.2f;
                RETURN_IF_FAILED(m_SpeedSignalId.writePin(m_WriterSpeed, (void *) &speedSignalValue, outputTime));
            }
            else
            {
                TSignalValue::Data speedSignalValue;
                speedSignalValue.f32Value = m_cSpeedController->speedConfig.min;
                RETURN_IF_FAILED(m_SpeedSignalId.writePin(m_WriterSpeed, (void *) &speedSignalValue, outputTime));
            }


            RETURN_IF_FAILED(m_PoseStructGoalId.writePin(m_WriterLocalGoal, (void *) &stop, outputTime));

            if (!m_cCarPoses->IsEmpty())
            {
                //save last intersection
                _lastIntersection.Save(m_cCarPoses->GetGlobalPose(Point2f(stop.f32PosX, stop.f32PosY),
                                                               stop.f32Yaw), outputTime);
            }

            m_cSpeedController->isInCurve = 0;

            TFeedbackStruct::Data feedback;
            feedback.ui8FilterId        = F_LINE_SPECIFIER;
            feedback.ui32FeedbackStatus = FB_LS_CROSSING_WAIT_POSITION;

            RETURN_IF_FAILED(m_FeedbackId.writePin(m_WriterFeedback, (void *) &feedback, outputTime));
        }
    }

    RETURN_NOERROR;
}

// DetermineWorldStopPoint
TPoseStruct::Data LineSpecifierAdapter::DetermineWorldStopPoint(Point2f stopPosition, Point2f image_yawDirection)
{
    Point2f yawDirection = Point2f(-image_yawDirection.y, -image_yawDirection.x);

    TPoseStruct::Data stop;
    if (yawDirection.x == 0)
    {
        return stop;
    }
    else if (yawDirection.x < 0)
    {
        stop.f32Yaw = atan(yawDirection.y / (-yawDirection.x));
    }
    else
    {
        stop.f32Yaw = atan(yawDirection.y / yawDirection.x);
    }

    stopPosition += offset.stopline.x * yawDirection
            + offset.stopline.y * CVMath::RotateCW90(-yawDirection);

    if (stopPosition.y * stopPosition.x < -offset.maxXYRation_Stopline)
    {
        stopPosition.y = -offset.maxXYRation_Stopline * stopPosition.x;
    }
    else if (stopPosition.y * stopPosition.x > offset.maxXYRation_Stopline)
    {
        stopPosition.y = offset.maxXYRation_Stopline * stopPosition.x;
    }

    if (stopPosition.x == 0)
    {
        stopPosition.x = 0.005;
    }
    else if (stopPosition.x < 0)
    {
        stopPosition.y = 0;
    }

    stop.f32PosX  = stopPosition.x;
    stop.f32PosY  = stopPosition.y;
    stop.f32Yaw  *= offset.yawScale;

    return stop;
}

// ProcessBackwards
tResult LineSpecifierAdapter::ProcessBackwards(cv::Mat &debugImage, Point2f frameCoord,
                                               cv::Point2f image_destination, tTimeStamp outputTime)
{
    Point2f image_yawDirection;
    Point2f image_DestPosition = m_cSpeedController->Backwards(m_MatImage, &debugImage, image_destination, outputTime,
                                                              &image_yawDirection, ImageUtils::MeterToPixel(frameCoord.x) - BINARY_IMAGE_HEIGHT);

    Point2f destPosition = ImageUtils::ConvertToWorldCoordinates(image_DestPosition, frameCoord);
    Point2f yawDirection = Point2f(-image_yawDirection.y, -image_yawDirection.x);

    TPoseStruct::Data stop;
    if (yawDirection.x == 0)
    {
        stop.f32Yaw = 0;
    }
    else if (yawDirection.x < 0)
    {
        stop.f32Yaw = atan(yawDirection.y / (-yawDirection.x));
    }
    else
    {
        stop.f32Yaw = atan(yawDirection.y / yawDirection.x);
    }

    destPosition += offset.backwards.x * yawDirection
            + offset.backwards.y * CVMath::RotateCW90(-yawDirection);

    stop.f32PosX = destPosition.x;
    stop.f32PosY = destPosition.y;

    RETURN_IF_FAILED(m_PoseStructGoalId.writePin(m_WriterLocalGoal, (void *) &stop,m_pClock->GetStreamTime()));

    // done
    RETURN_NOERROR;
}

// ChangeType
tResult LineSpecifierAdapter::ChangeType(adtf::streaming::cDynamicSampleReader& reader, adtf::streaming::tStreamImageFormat& inputFormat,
    const adtf::streaming::ant::IStreamType& oType)
{
    if (oType == adtf::streaming::stream_meta_type_image())
    {
        adtf::ucom::object_ptr<const adtf::streaming::IStreamType> pTypeInput;
        // get pType from input reader
        reader >> pTypeInput;
        adtf::streaming::get_stream_type_image_format(inputFormat, *pTypeInput);
    }
    else
    {
        RETURN_ERROR(ERR_INVALID_TYPE);
    }

    RETURN_NOERROR;
}

