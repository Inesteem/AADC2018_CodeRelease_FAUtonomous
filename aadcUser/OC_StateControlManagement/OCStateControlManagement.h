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
#define CID_OC_STATE_CONTROL_MANAGEMENT_FILTER "oc_state_control_management.filter.user.aadc.cid"

// namespaces
using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
#include "ADTF3_helper.h"
#include "stdafx.h"
#include <boost/thread.hpp>

// OCStateControlManagement
class OCStateControlManagement : public cTriggerFunction
{
private:
    /*------------ STRUCTS -------------*/
    // create structs to hold information we get from SignalValues
    // coding convention: 	o_******Id

    TActionStruct   o_TActionStruct;
    TFeedbackStruct o_TFeedbackStruct;
    TSignalValue    o_TSignalValue;
    /*-------------- PINS --------------*/
    // create pins for input and output
    // coding convention:	m_Reader******	for input  signals
    //			m_Writer******	for output signals

    // input pins
//    cPinReader m_ReaderFeedbackSCMTestCheckSpeed;
//    cPinReader m_ReaderFeedbackSCMTestLimitSpeed;
    cPinReader m_ReaderTriggerPin;
    cPinReader m_ReaderFeedbackLineSpecifier;           // 01: Line Specifier
    cPinReader m_ReaderFeedbackMoveToPoint;             // 02: Move To Point
    cPinReader m_ReaderFeedbackMarkerDetector;          // 03: Marker Detection
    cPinReader m_ReaderFeedbackTimer;                   // 04: Timer
    cPinReader m_ReaderFeedbackUltrasonicCheck;         // 05: Ultrasonic Check
    cPinReader m_ReaderFeedbackObstacleDetector;        // 06: Obstacle Detection
    cPinReader m_ReaderFeedbackLightControl;            // 07: Light Control
    cPinReader m_ReaderFeedbackSelectSpeed;              // 08: Action Stop
    cPinReader m_ReaderFeedbackUltrasonicACC;           // 09: Ultrasonic ACC
    cPinReader m_ReaderFeedbackJuryComm;                // 10: Jury Communication
    cPinReader m_ReaderFeedbackSpeedController;         // 14: Object Specifier
    cPinReader m_ReaderFeedbackIntersectionDetector;    // 15: Intersection Detector



    // output pins
    cPinWriter m_WriterActionLineSpecifier;           // 01: Line Specifier
    cPinWriter m_WriterActionMoveToPoint;             // 02: Move To Point
    cPinWriter m_WriterActionMarkerDetector;          // 03: Marker Detection
    cPinWriter m_WriterActionTimer;                   // 04: Timer
    cPinWriter m_WriterActionUltrasonicCheck;         // 05: Ultrasonic Check
    cPinWriter m_WriterActionObstacleDetector;        // 06: Obstacle Detection
    cPinWriter m_WriterActionLightControl;            // 07: Light Control
    cPinWriter m_WriterActionSelectSpeed;              // 08: Action Stop
    cPinWriter m_WriterActionUltrasonicACC;           // 09: Ultrasonic ACC
    cPinWriter m_WriterActionJuryComm;                // 10: Jury Communication
    cPinWriter m_WriterActionSpeedController;         // 14: Object Specifier
    cPinWriter m_WriterActionIntersectionDetector;    // 15: Intersection Detector

    /*------------ PROPERTIES ----------*/
    // set property variables
    // coding convention:	m_prop******
    adtf::base::property_variable<cFilename> m_scmFile = cFilename(cString("/home/aadc/AADC/utilities/xiangfei_SCMTesting/SCM_Structure_left.xml"));

    /*------------ VARIABLES -----------*/
    // coding convention:	m_******

    tInt16    m_i16CurrentScmManeuverID;    // current state machine maneuver
    tInt16    m_i16CurrentScmStepID;        // current state machine step
    tInt8     m_i8CurrentActLvlMode;        // Variable to save current activity level (NOT_INITIALIZED = -1, ACTIVE = 1, PASSIVE = 2)
    cFilename m_strStructureFileName;       // .xml file of scm structure list
    tInt32 counter = 0;
    std::vector<sc_Maneuver> m_vecScmStructureList; // list of all maneuvers from the maneuver list
    // coding convention:	m_******
    object_ptr<adtf::services::IReferenceClock> m_pClock; // clock for timestamps

    TFeedbackStruct::Data m_dataFeedbackLineSpecifier;
    TFeedbackStruct::Data m_dataFeedbackMoveToPoint;
    TFeedbackStruct::Data m_dataFeedbackMarkerDetector;
    TFeedbackStruct::Data m_dataFeedbackTimer;
    TFeedbackStruct::Data m_dataFeedbackUltrasonicCheck;
    TFeedbackStruct::Data m_dataFeedbackObstacleDetector;
    TFeedbackStruct::Data m_dataFeedbackLightControl;
    TFeedbackStruct::Data m_dataFeedbackSelectSpeed;
    TFeedbackStruct::Data m_dataFeedbackUltrasonicACC;
    TFeedbackStruct::Data m_dataFeedbackJuryComm;
    TFeedbackStruct::Data m_dataFeedbackSpeedController;
    TFeedbackStruct::Data m_dataFeedbackIntersectionDetector;


//    tUInt32 m_ui32LastTimeStampFeedback = 0;

    boost::mutex m_oCriticalSectionTransmitActionStruct;
    boost::mutex CritSectionCurrentActivityLvlMode;
    boost::mutex CritSectionCurrentScmState;

    tUInt32 timestamp_juryComm = -1;
    tUInt32 timestamp_SelectSpeed = -1;
    tUInt32 timestamp_lineSpecifier = -1;
    tUInt32 timestamp_moveToPoint = -1;
    tUInt32 timestamp_markerDetector = -1;
    tUInt32 timestamp_timer = -1;
    tUInt32 timestamp_usCheck = -1;
    tUInt32 timestamp_obstacleDetector = -1;
    tUInt32 timestamp_lightControl = -1;
    tUInt32 timestamp_usACC = -1;
    tUInt32 timestamp_speedController = -1;
    tUInt32 timestamp_ISD = -1;
    tBool m_bSpecialCase;
    tUInt32 m_ui32CurrentManeuver;


public:
    /*------------ FUNCTIONS -----------*/
    /*------------ FROM AUDI -----------*/
    // constructor
    OCStateControlManagement();

    // destructor
    ~OCStateControlManagement() = default;

    // Configure
    virtual tResult Configure() override;

    // Process
    virtual tResult Process(tTimeStamp tmTimeOfTrigger) override;

    // ---------- FROM USER ------------*/
    /* Function that is called for processing the data in activityLevel:Active of current SCM state;
     *  calls a transmit function to transmit necessary activation signals & commands to corresponding filters */
    tResult ExecAction();

    /* Function that is called for processing the data in activityLevel:Passive of current SCM state;
     *  sets/changes the current SCM state, sets i8CurrentActLvlMode, calls ExecAction afterwards
     *  with corresponding argument of new SCM state
     *   ** actual LOGIC ** : decodes the commands in feedback and executes the corresponding action */
    tResult ExecRequest(tUInt32 inputCommand);

    /* Function that is called for processing the data at every pin input event of FeedbackPin;
     *  checks if current state is in passive mode: then feedback-input is requested and accepted;
     *  checks if filterID is in list of feedback that is requested at the moment;
     *  takes corresponding action by calling function*/
    tResult ProcessFeedback(TFeedbackStruct::Data );


    /* Function to check if received feedback data is relevant for current status (data that is waited for);
     *  if filterID is in list: method returns the command read from SCM-structure for (redundant) comparison (fault-detection) */
    tUInt32 CheckRequestRelevance(TFeedbackStruct::Data feedback);

    /* Function to change current step ID*/
    tResult ChangeCurrentScmStepId(tUInt32 numberOfSteps);

    /* Function to jump to step with scmStepID in current maneuver */
    tResult JumpToStep(tUInt32 inputScmStepId);

    /* Function to jump to maneuver with scmManeuverID */
    tResult JumpToManeuver(tUInt32 inputScmManeuverId);

    /* reads the xml file which is set in the filter properties */
    tResult LoadSCMStructureData();

}; // OCStateControlManagement
