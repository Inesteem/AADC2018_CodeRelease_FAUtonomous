/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**********************************************************************
* This filter checks if the environment has enough place using ultrasonic sensors.

* $Adapted by:: Xiangfei#  $Date:: 2018-08-01 12:44:00#
**********************************************************************/

#pragma once

#define CID_ULTRASONIC_CHECK_FILTER "ultrasonic_check.filter.user.aadc.cid"

// namespace
using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

#include "ADTF3_helper.h"
#include "stdafx.h"
#include <boost/thread.hpp>

// UltrasonicCheck
class UltrasonicCheck : public cTriggerFunction
{
private:
    /*------------ STRUCTS -------------*/
    // create structs to hold information we get from SignalValues
    // coding convention: 	o_******Id

    TUltrasonicStruct o_UltrasonicStruct;
    TActionStruct o_ActionStruct;
    TFeedbackStruct o_FeedbackStruct;
    TLaserScannerData o_LaserScannerData;
    /*-------------- PINS --------------*/
    // create pins for input and output
    // coding convention:	m_Reader******	for input  signals
    //			m_Writer******	for output signals

    // input pins
    cPinReader m_ReaderAction;
    cPinReader m_ReaderUltrasonic;
    cPinReader m_ReaderLaserScanner;
    // output pins

    cPinWriter m_WriterFeedback;
    /*------------ PROPERTIES ----------*/
    // set property variables
    // coding convention:	m_prop******


    adtf::base::property_variable<tBool>    m_propBDebugModeEnabled                 = tBool(tFalse) ;
    adtf::base::property_variable<tUInt32> m_propUI32NumberOfSamples                   = 20      ;
    adtf::base::property_variable<tUInt32> m_propUI32NumberOfSamplesOvertaking        = 5       ;
    adtf::base::property_variable<tFloat32> m_propF32NoObstacleThreshold               = tFloat(0.7f)  ;
    adtf::base::property_variable<tFloat32> m_propF32ErrorRateThreshold                  = tFloat(0.3f)  ;
    adtf::base::property_variable<tFloat32> m_propF32ParkingSpaceLongThreshold        = tFloat(0.3f)  ;
    adtf::base::property_variable<tFloat32> m_propF32ParkingSpaceTransThreshold       = tFloat(0.5f)  ;
    adtf::base::property_variable<tFloat32> m_propF32GivewayThreshold                   = tFloat(0.8f)  ;
    adtf::base::property_variable<tFloat32> m_propF32OvertakingThreshold                = tFloat(0.4f)  ;

    /*------------ VARIABLES -----------*/
    // coding convention:	m_******

    tFloat32 m_f32CarStoppedThreshold;                    // speed in m/s that is seen as 'car stopped'
    object_ptr<adtf::services::IReferenceClock> m_pClock; // clock for timestamps
    std::mutex m_mutexCommandActivatedFlag;               // mutex for read and writes in m_propCommandActivated

    //        tBool m_bDebugModeEnabled;
    //        tUInt32 number_of_samples; // number of samples used for obstacle detection (US: 20 samples/s)
    //        tUInt32 number_of_samples_overtaking;
    //        tFloat32 no_obstacle_threshold; // samples (in percent) that have to be clear
    //        tFloat32 parking_space_long_threshold;
    //        tFloat32 parking_space_trans_threshold;
    //        tFloat32 giveway_threshold;
    //        tFloat32 overtaking_threshold;
    //        tFloat32 errorRateThreshold;

    TLaserScannerData::Data m_dataLaserSignal;

    // flags and values
    tBool m_bRunning; // running state
    tUInt32 m_ui32Command; // received command
    tUInt32 m_ui32SampleCounter; // count received ultrasonic samples
    tUInt32 m_ui32NoObstacleCounter; // count samples without obstacles
    tUInt32 m_ui32ErrorCounter; // count samples with US distance < 0.03

    // critical sections
    boost::mutex 	criticalSection_OnPinEvent;
    boost::mutex	m_oCriticalSectionActionAccess;
    boost::mutex	m_oCriticalSectionRunningStateAccess;
    boost::mutex	m_oCriticalSectionUltrasonicInput;


    //init timestamps
    tUInt32 m_ui32TimeStampUS;
    tUInt32 m_ui32TimeStampAction;
public:
    /*------------ FUNCTIONS -----------*/
    // constructor
    UltrasonicCheck();

    // destructor
    ~UltrasonicCheck() = default;

    // Configure
    virtual tResult Configure() override;

    // Process
    virtual tResult Process(tTimeStamp tmTimeOfTrigger) override;

    // ProcessActionCommand
    tResult ProcessActionInput(TActionStruct::Data inputActionCommand);

    // Running State
    tBool GetRunningState();

    tResult SetRunningState(tBool state);

    // process state machine action input
    tResult ProcessActionInput();

    // process ultrasonic input
    tResult ProcessUltrasonicInput(TUltrasonicStruct::Data inputSignal, TLaserScannerData::Data inputLaser);

    TPolarCoordiante::Data GetClosestPoint(TLaserScannerData::Data inputScanPoints, tFloat32 startRange, tFloat32 endRange);

    //to compensate the inconsistante angle range-> output car right 90° to car left -90°
    tFloat32 AngleCompensation(tFloat32 angle);
}; // UltrasonicCheck
