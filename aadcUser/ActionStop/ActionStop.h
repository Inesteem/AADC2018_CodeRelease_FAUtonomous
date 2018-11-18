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

#pragma once

#define CID_ACTION_STOP_FILTER "action_stop.filter.user.aadc.cid"

// namespace
using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::streaming;
using namespace adtf::base;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

#include "ADTF3_helper.h"
#include "stdafx.h"
#include "aadc_create_mediadescription.h"
#include "aadc_transmit_samples.h"
#include "aadc_read_samples.h"

// ActionStop
class ActionStop : public cTriggerFunction
{
private:
    /*------------ STRUCTS -------------*/
    // create structs to hold information we get from SignalValues
    // coding convention: 	o_******Id
    /*
        struct speedSignalValueStruct
        {
                tSize ui32Timestamp;
                tSize f32Value;
        } o_SpeedSignalId;
        */
    TSignalValue m_SpeedSignal;
    /*
        struct actionCommandStruct
        {
                tSize command;
        } o_ActionCommandStructId;
        */
    TActionStruct m_ActionCommand;
    /*
        struct feedbackStruct
        {
                tSize filterId;
                tSize feedbackStatus;
        } o_FeedbackStructId;
        */
    TFeedbackStruct m_FeedbackStruct;


    /*-------------- PINS --------------*/
    // create pins for input and output
    // coding convention:	m_Reader******	for input  signals
    //			m_Writer******	for output signals

    // input pins
    cPinReader m_ReaderSpeed;
    cPinReader m_ReaderAction;

    // output pins
    cPinWriter m_WriterSpeed;
    cPinWriter m_WriterFeedback;


    /*------------ PROPERTIES ----------*/
    // set property variables
    // coding convention:	m_prop******

    adtf::base::property_variable<tFloat32> m_propPercentageActiveBraking = tFloat(0.00f);  // percentage that should be used for active breaking
    adtf::base::property_variable<tBool> m_propDebugActivted = tBool(tFalse);  // percentage that should be used for active breaking

    /*------------ VARIABLES -----------*/
    // coding convention:	m_******
    tBool m_propCommandActivated = tFalse;
    tFloat32 m_f32CarStoppedThreshold;                    // speed in m/s that is seen as 'car stopped'
    object_ptr<adtf::services::IReferenceClock> m_pClock; // clock for timestamps
    std::mutex m_mutexCommandActivatedFlag;               // mutex for read and writes in m_propCommandActivated
    tUInt32 counter_speed = 0;
    tUInt32 counter_action = 0;
    tUInt32 counter_process = 0;
    tBool feedback_sent = tFalse;
    TSignalValue::Data speedSignal;
    tUInt32 m_timestampAction = 0;
    TActionStruct::Data tmp_actionStruct;
public:
    /*------------ FUNCTIONS -----------*/
    // constructor
    ActionStop();

    // destructor
    ~ActionStop() = default;

    // Configure
    virtual tResult Configure() override;

    // Process
    virtual tResult Process(tTimeStamp tmTimeOfTrigger) override;

    // TransmitSpeed
    tResult TransmitSpeed(TSignalValue::Data inputSpeed);

    // TransmitFeedback
    tResult TransmitFeedback(TFeedbackStruct::Data inputFeedback);


    // ProcessActionCommand
    tResult ProcessActionCommand(TActionStruct::Data inputActionCommand);


}; // ActionStop
