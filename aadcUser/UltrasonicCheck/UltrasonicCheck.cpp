/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/

#include <mutex>
#include "stdafx.h"
#include "UltrasonicCheck.h"
#include "ScmCommunication.h"
#include <ADTF3_helper.h>

#define ERROR_RATE_THRESHOLD "General::Error Rate Threshold"

// This will define the filter and expose it via plugin class factory.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
        CID_ULTRASONIC_CHECK_FILTER,                         // references to header file
        "UltrasonicCheck",                                   // label
        UltrasonicCheck,                                      // class
        adtf::filter::pin_trigger({"ultrasonicInput", "laserScannerInput"}));	// set trigger pin


// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
UltrasonicCheck::UltrasonicCheck()
{

    // ------------------------------------------
    SetName("UltrasonicCheck Constructor");

    // -----------------------------------------
    // set pins
    o_ActionStruct.registerPin(this     , m_ReaderAction    , "actionInput"     );
    o_UltrasonicStruct.registerPin(this , m_ReaderUltrasonic, "ultrasonicInput" );
    o_LaserScannerData.registerPin(this , m_ReaderLaserScanner, "laserScannerInput" );
    o_FeedbackStruct.registerPin(this   , m_WriterFeedback  , "feedbackOutput"  );

    // -----------------------------------------
    // set property variables
    RegisterPropertyVariable("General::Enable Debug"                        , m_propBDebugModeEnabled               );
    RegisterPropertyVariable("General::Number Of Samples"                   , m_propUI32NumberOfSamples                 );
    RegisterPropertyVariable("General::Number Of Samples Overtaking"        , m_propUI32NumberOfSamplesOvertaking      );
    RegisterPropertyVariable("General::No Obstacle Threshold"               , m_propF32NoObstacleThreshold             );
    RegisterPropertyVariable(ERROR_RATE_THRESHOLD                           , m_propF32ErrorRateThreshold                );
    RegisterPropertyVariable("Distances::Parking Space Long Threshold"      , m_propF32ParkingSpaceLongThreshold      );
    RegisterPropertyVariable("Distances::Parking Space Trans Threshold"     , m_propF32ParkingSpaceTransThreshold     );
    RegisterPropertyVariable("Distances::Giveway Left Threshold"            , m_propF32GivewayThreshold                 );
    RegisterPropertyVariable("Distances::Overtaking Obstacle Threshold"     , m_propF32OvertakingThreshold              );

}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult UltrasonicCheck::Configure()
{
    //init timestamps
    m_ui32TimeStampUS = 0;
    m_ui32TimeStampAction = 0;
    m_bRunning = tFalse; // set running state
    m_ui32Command = 0; // received command
    m_ui32SampleCounter = 0; // reset sample counter
    m_ui32NoObstacleCounter = 0; // count samples without obstacles
    m_ui32ErrorCounter = 0; // count samples with US distance < 0.03

    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    // done
    RETURN_NOERROR;
}


// ------------------------------
// ---------- PROCESS -----------
// ------------------------------
// process() is always called each time a trigger occured (new data sample arrive).
// IMPORTANT: Make sure to read ALL samples of ALL readers!
tResult UltrasonicCheck::Process(tTimeStamp tmTimeOfTrigger)
{
    boost::lock_guard<boost::mutex> lock(criticalSection_OnPinEvent);


    // if new signal is US

    TUltrasonicStruct::Data tmp_dataUltrasonicSignal;
    if(IS_OK(o_UltrasonicStruct.readPin(m_ReaderUltrasonic, (void *) & tmp_dataUltrasonicSignal, m_ui32TimeStampUS))){

        m_ui32TimeStampUS = tmp_dataUltrasonicSignal.tSideLeft.ui32ArduinoTimestamp;

        // process data
        if (GetRunningState())
        {
            ProcessUltrasonicInput(tmp_dataUltrasonicSignal, m_dataLaserSignal);
        }

    }


    // if new signal is Action
    TActionStruct::Data tmp_dataActionSignal;
    if(IS_OK(o_ActionStruct.readPin(m_ReaderAction, (void *) & tmp_dataActionSignal,m_ui32TimeStampAction))){

        m_ui32TimeStampAction = tmp_dataActionSignal.ui32ArduinoTimestamp;

        // process data
        RETURN_IF_FAILED(ProcessActionInput(tmp_dataActionSignal));

    }

    // if new signal is LS
    if(IS_OK(o_LaserScannerData.readPin(m_ReaderLaserScanner, (void *) & m_dataLaserSignal))){
    }

    // done
    RETURN_NOERROR;
}
// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------


tBool UltrasonicCheck::GetRunningState()
{
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionRunningStateAccess);
    return m_bRunning;
}

tResult UltrasonicCheck::SetRunningState(tBool tmp_bState)
{
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionRunningStateAccess);

    m_bRunning = tmp_bState;
    RETURN_NOERROR;
}

tResult UltrasonicCheck::ProcessActionInput(TActionStruct::Data tmp_dataAction)
{

    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionActionAccess);
    // set running state
    if(!GetRunningState() && tmp_dataAction.bEnabled && tmp_dataAction.bStarted)
    {
        m_ui32SampleCounter = 0; // reset sample counter
        m_ui32NoObstacleCounter = 0; // count samples without obstacles
        m_ui32ErrorCounter = 0; // count samples with US distance < 0.03
        SetRunningState(tTrue);
        m_ui32Command = tmp_dataAction.ui32Command;
        RETURN_NOERROR;
    }

    // send disable information when in running mode
    if (!tmp_dataAction.bEnabled && GetRunningState())
    {
        SetRunningState(tFalse);
    }

    RETURN_NOERROR;
}

tResult UltrasonicCheck::ProcessUltrasonicInput(TUltrasonicStruct::Data tmp_dataUltrasonic, TLaserScannerData::Data tmp_dataLaserScanner)
{
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionUltrasonicInput);
    // get ultrasonic sensor input

    tmp_dataUltrasonic.tRearCenter.f32Value /= 100;
    tmp_dataUltrasonic.tRearLeft.f32Value /= 100;
    tmp_dataUltrasonic.tRearRight.f32Value /= 100;
    tmp_dataUltrasonic.tSideLeft.f32Value /= 100;
    tmp_dataUltrasonic.tSideRight.f32Value /= 100;

    TPolarCoordiante::Data tmp_dataLaserFrontLeft           = GetClosestPoint(tmp_dataLaserScanner, -90, -30);
    TPolarCoordiante::Data tmp_dataLaserFrontCenterLeft     = GetClosestPoint(tmp_dataLaserScanner, -30, -10);
    TPolarCoordiante::Data tmp_dataLaserFrontCenter         = GetClosestPoint(tmp_dataLaserScanner, -10, 10);
    TPolarCoordiante::Data tmp_dataLaserFrontCenterRight    = GetClosestPoint(tmp_dataLaserScanner, 10, 30);
    TPolarCoordiante::Data tmp_dataLaserFrontRight          = GetClosestPoint(tmp_dataLaserScanner, 30, 90);


    switch (m_ui32Command)
    {
    case AC_UC_CHECK_OVERTAKING_OBSTACLE:


        if(tmp_dataUltrasonic.tSideRight.f32Value > m_propF32OvertakingThreshold)
        {
            m_ui32NoObstacleCounter++;


        } else if (tmp_dataUltrasonic.tSideRight.f32Value < 0.03)
        {

            //m_ui32ErrorCounter++; //TODO check this!
            if(m_propBDebugModeEnabled)
            {
                LOG_WARNING(cString::Format("Ultrasonic Check Overtaking: error in sensor side right value: %f", tmp_dataUltrasonic.tSideRight.f32Value));
            }
        }
        //        else
        //        { LOG_INFO("obstacle detected!");
        //        }
        break;

    case AC_UC_CHECK_PARKING_SPACE_LONG:
        if(tmp_dataUltrasonic.tSideRight.f32Value > m_propF32ParkingSpaceLongThreshold)
        {
            LOG_INFO("no obstacle");
            m_ui32NoObstacleCounter++;
        } else if (tmp_dataUltrasonic.tSideRight.f32Value < 0.03)
        {
            m_ui32ErrorCounter++;
            if(m_propBDebugModeEnabled)
            {
                LOG_WARNING(cString::Format("Ultrasonic Check Parking Space Long: error in sensor side right value: %f", tmp_dataUltrasonic.tSideRight.f32Value));
            }
        }
        //        else
        //        { LOG_INFO("obstacle detected!");
        //        }
        break;

    case AC_UC_CHECK_PARKING_SPACE_TRANS:
        if(tmp_dataUltrasonic.tSideRight.f32Value > m_propF32ParkingSpaceTransThreshold)
        {
            m_ui32NoObstacleCounter++;
        } else if (tmp_dataUltrasonic.tSideRight.f32Value < 0.03)
        {
            m_ui32ErrorCounter++;
            if(m_propBDebugModeEnabled)
            {
                LOG_WARNING(cString::Format("Ultrasonic Check Parking Space Trans: error in sensor side right value: %f", tmp_dataUltrasonic.tSideRight.f32Value));
            }
        }
        //        else
        //        { LOG_INFO("obstacle detected!");
        //        }
        break;




    case AC_UC_CHECK_GIVEWAY_LEFT:


        if(tmp_dataLaserFrontLeft.f32Radius > m_propF32GivewayThreshold * 1000) {
            m_ui32NoObstacleCounter++;

        } else if (tmp_dataLaserFrontLeft.f32Radius < 0.03 * 1000) {
            m_ui32ErrorCounter++;
            if(m_propBDebugModeEnabled) {
                LOG_WARNING(cString::Format("LaserScanner Check Giveway Left: error in sensor front left value: %f", tmp_dataLaserFrontLeft.f32Radius));
            }
        }
        //        else
        //        { LOG_INFO("obstacle detected!");
        //        }
        break;


    case AC_UC_CHECK_PARKING_TRANS_FRONT:

        if(tmp_dataLaserFrontLeft.f32Radius > 0.3 * 1000) {
            //LOG_INFO("front left no obstacle");
            m_ui32NoObstacleCounter++;
        }
        //        else{
        //            LOG_INFO("front left obstacle detected!");
        //        }
        if(tmp_dataLaserFrontCenterLeft.f32Radius > 0.3 * 1000) {
            // LOG_INFO("front center left no obstacle");
            m_ui32NoObstacleCounter++;
        }
        //        else{
        //            LOG_INFO("front center left obstacle detected!");
        //        }
        if (tmp_dataLaserFrontLeft.f32Radius < 0.03 * 1000 || tmp_dataLaserFrontCenterLeft.f32Radius < 0.03 * 1000) {
            m_ui32ErrorCounter++;
            if(m_propBDebugModeEnabled) {
                LOG_WARNING(cString::Format("LaserScanner Check Parking Trans Front: error in sensors, front left value: %f, front center left: %f", tmp_dataLaserFrontLeft.f32Radius, tmp_dataLaserFrontCenterLeft.f32Radius));
            }
        }

        break;
    case AC_UC_CHECK_PULL_OUT_LONG:
        LOG_INFO("front left: %f, side left: %f, rear left: %f",tmp_dataLaserFrontLeft.f32Radius,tmp_dataUltrasonic.tSideLeft.f32Value,tmp_dataUltrasonic.tRearLeft.f32Value);
        if(tmp_dataLaserFrontLeft.f32Radius > 0.4 * 1000) {
            m_ui32NoObstacleCounter++;
        }
//        else
//        { LOG_INFO("obstacle detected!");
//        }
        if(tmp_dataUltrasonic.tSideLeft.f32Value > 0.3) {
            m_ui32NoObstacleCounter++;
        }
//        else
//        { LOG_INFO("obstacle detected!");
//        }
        if(tmp_dataUltrasonic.tRearLeft.f32Value > 0.4) {
            m_ui32NoObstacleCounter++;
        }
//        else
//        { LOG_INFO("obstacle detected!");
//        }
        if (tmp_dataLaserFrontLeft.f32Radius < 0.03 * 1000 || tmp_dataUltrasonic.tSideLeft.f32Value < 0.03 || tmp_dataUltrasonic.tRearLeft.f32Value < 0.03) {
            m_ui32ErrorCounter+=3;
            if(m_propBDebugModeEnabled) {
                LOG_WARNING(cString::Format("LaserScanner & Ultrasonic Check: error in sensor front left, side left or rear left; values: %f, %f, %f", tmp_dataLaserFrontLeft.f32Radius, tmp_dataUltrasonic.tSideLeft.f32Value, tmp_dataUltrasonic.tRearLeft.f32Value));
            }
        }

        break;
    case AC_UC_CHECK_PULL_OUT_TRANS:
        if(tmp_dataLaserFrontLeft.f32Radius > 1.0 * 1000) {
            m_ui32NoObstacleCounter++;
        }
        if(tmp_dataLaserFrontCenterLeft.f32Radius > 0.8 * 1000) {
            m_ui32NoObstacleCounter++;
        }
        if(tmp_dataLaserFrontCenter.f32Radius > 0.7 * 1000) {
            m_ui32NoObstacleCounter++;
        }
        if(tmp_dataLaserFrontCenterRight.f32Radius > 0.8 * 1000) {
            m_ui32NoObstacleCounter++;
        }
        if(tmp_dataLaserFrontRight.f32Radius > 1.0 * 1000) {
            m_ui32NoObstacleCounter++;
        }
        if (tmp_dataLaserFrontLeft.f32Radius < 0.03 * 1000 || tmp_dataLaserFrontCenterLeft.f32Radius < 0.03 * 1000|| tmp_dataLaserFrontCenter.f32Radius < 0.03 * 1000 || tmp_dataLaserFrontCenterRight.f32Radius < 0.03 * 1000 || tmp_dataLaserFrontRight.f32Radius < 0.03 * 1000) {
            m_ui32ErrorCounter+=5;
            if(m_propBDebugModeEnabled) {
                LOG_WARNING(cString::Format("LaserScanner Check: error in front sensor(s); values: %f, %f, %f, %f, %f", tmp_dataLaserFrontLeft.f32Radius, tmp_dataLaserFrontCenterLeft.f32Radius, tmp_dataLaserFrontCenter.f32Radius, tmp_dataLaserFrontCenterRight.f32Radius, tmp_dataLaserFrontRight.f32Radius));
            }
        }
//        else
//        { LOG_INFO("obstacle detected!");
//        }
        break;

    default:
        SetRunningState(tFalse);
        LOG_ERROR(cString::Format("Ultrasonic Check: Error occurred in ProcessUltrasonicInput(), command id (%d) not known", m_ui32Command));
    }

    // increase counter
    m_ui32SampleCounter++;




    // evaluation
    if( (m_ui32SampleCounter >= fabsf(m_propUI32NumberOfSamples) && m_ui32Command != fabsf(AC_UC_CHECK_OVERTAKING_OBSTACLE)) || ( m_ui32SampleCounter >= fabsf(m_propUI32NumberOfSamplesOvertaking) && m_ui32Command == fabsf(AC_UC_CHECK_OVERTAKING_OBSTACLE)) )
    {


        if(m_ui32Command == fabsf(AC_UC_CHECK_PARKING_TRANS_FRONT)) m_ui32SampleCounter *= 2;
        else if(m_ui32Command == fabsf(AC_UC_CHECK_PULL_OUT_LONG)) m_ui32SampleCounter *= 3;
        else if (m_ui32Command == fabsf(AC_UC_CHECK_PULL_OUT_TRANS)) m_ui32SampleCounter *= 5;

        // calculate number of free samples (relative to all received samples)
        tFloat32 tmp_f32FreeSamples = m_ui32NoObstacleCounter/m_ui32SampleCounter;
        tFloat32 tmp_f32ErrorSamples = m_ui32ErrorCounter/m_ui32SampleCounter;

        // create feedback
        TFeedbackStruct::Data tmp_dataFeedback;
        tmp_dataFeedback.ui8FilterId = F_ULTRASONIC_CHECK;

        // check error count
        if(tmp_f32ErrorSamples > m_propF32ErrorRateThreshold)
        {
            SetRunningState(tFalse);
            m_ui32ErrorCounter = 0;
            m_ui32SampleCounter = 0;
            tmp_dataFeedback.ui32FeedbackStatus = FB_UC_ERROR_CODE; // error
            //LOG_INFO("FB_UC_ERROR_CODE");
            RETURN_IF_FAILED(o_ActionStruct.writePin(m_WriterFeedback, (void *) &tmp_dataFeedback, m_pClock->GetStreamTime()));

            LOG_ERROR(cString::Format("Ultrasonic Check: error in mode %d, error rate too high: %d out of %d samples, rate: %f", m_ui32Command, m_ui32ErrorCounter, m_ui32SampleCounter, tmp_f32ErrorSamples));
            RETURN_ERROR(ERR_BAD_DEVICE);
        }

        // check free samples: no obstacle -> free
        if(tmp_f32FreeSamples > m_propF32NoObstacleThreshold)
        {
            tmp_dataFeedback.ui32FeedbackStatus = FB_UC_NO_OBSTACLE;
            //LOG_INFO("FB_UC_NO_OBSTACLE");
            RETURN_IF_FAILED(o_ActionStruct.writePin(m_WriterFeedback, (void *) &tmp_dataFeedback, m_pClock->GetStreamTime()));

            SetRunningState(tFalse);
            if(m_propBDebugModeEnabled)
            {
                LOG_WARNING(cString::Format("Ultrasonic Check: Checked command %d; no obstacle counter: %d, samples: %d, rate: %f => free", m_ui32Command, m_ui32NoObstacleCounter, m_ui32SampleCounter, tmp_f32FreeSamples));
            }
            // obstacle
        }
        else
        {
            // parking mode: send feedback obstacle
            if( m_ui32Command == fabsf(AC_UC_CHECK_PARKING_SPACE_LONG) || m_ui32Command == fabsf(AC_UC_CHECK_PARKING_SPACE_TRANS) )
            {
                tmp_dataFeedback.ui32FeedbackStatus = FB_UC_OBSTACLE;
                LOG_INFO("FB_UC_OBSTACLE");
                RETURN_IF_FAILED(o_ActionStruct.writePin(m_WriterFeedback, (void *) &tmp_dataFeedback, m_pClock->GetStreamTime()));

                SetRunningState(tFalse);
                if(m_propBDebugModeEnabled)
                {
                    LOG_WARNING(cString::Format("Ultrasonic Check: Checked command %d; no obstacle counter: %d, samples: %d, rate: %f => occupied", m_ui32Command, m_ui32NoObstacleCounter, m_ui32SampleCounter, tmp_f32FreeSamples));
                }
                // restart cycle (wait until obstacle disappears)
            } else

            {
                LOG_INFO("FB_UC_OBSTACLE");
                if(m_propBDebugModeEnabled)
                {
                    LOG_WARNING(cString::Format("Ultrasonic Check [Restart]: Checked command %d; no obstacle counter: %d, samples: %d, rate: %f => occupied", m_ui32Command, m_ui32NoObstacleCounter, m_ui32SampleCounter, tmp_f32FreeSamples));
                }
                m_ui32SampleCounter = 0;
                m_ui32NoObstacleCounter = 0;
            }
        }
    }

    RETURN_NOERROR;
}

TPolarCoordiante::Data UltrasonicCheck::GetClosestPoint(TLaserScannerData::Data inputScanPoints, tFloat32 startRange, tFloat32 endRange)
{
    TPolarCoordiante::Data scanPoint;
    TPolarCoordiante::Data closestObstacle;
    closestObstacle.f32Angle  =     0.0f;
    closestObstacle.f32Radius = 99999.9f;

    for(uint32_t i = 0; i < (inputScanPoints.ui32Size);++i)
    {
        scanPoint.f32Radius = inputScanPoints.tScanArray[i].f32Radius;
        scanPoint.f32Angle = inputScanPoints.tScanArray[i].f32Angle;

        scanPoint.f32Angle = AngleCompensation(scanPoint.f32Angle);
        // laserscanner limits
        if(scanPoint.f32Angle >= tFloat32(startRange) && scanPoint.f32Angle <= tFloat32(endRange))
        {
            // radius 0 is invalid, get closest obstacle
            if(scanPoint.f32Radius > 0 && closestObstacle.f32Radius > scanPoint.f32Radius)
            {
                closestObstacle = scanPoint;
            }
        }
    }
    return closestObstacle;
}

//to compensate the inconsistante angle range-> output car right 90° to car left -90°
tFloat32 UltrasonicCheck::AngleCompensation(tFloat32 angle)
{
    if (angle >= 0.0 && angle <= 90)
    {
        return angle;
    }
    else if(angle >= 270 && angle <= 360)
    {
        return (angle -360);
    }
    else
    {
        LOG_ERROR("Angle out of range!");
        return 1000;//Error value but without error handeling!
    }
}
