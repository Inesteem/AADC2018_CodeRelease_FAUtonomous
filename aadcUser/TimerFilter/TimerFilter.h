﻿/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**********************************************************************
* This filter limits output speed and steering.

* $Adapted by:: Xiangfei#  $Date:: 2018-08-01 12:44:00# status: adapted, not tested
**********************************************************************/

#pragma once

#define CID_TIMER_FILTER_FILTER "timer_filters.filter.user.aadc.cid"

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

// TimerFilter
class TimerFilter : public cTriggerFunction
{
private:
    /*------------ STRUCTS -------------*/
    // create structs to hold information we get from SignalValues
    // coding convention: 	o_******Id
    TSignalValue o_TSignalValue;
    TActionStruct o_TActionStruct;
    TFeedbackStruct o_TFeedbackStruct;

    /*-------------- PINS --------------*/
    // create pins for input and output
    // coding convention:	m_Reader******	for input  signals
    //			m_Writer******	for output signals

    // input pins
    cPinReader m_ReaderTriggerTime;
    cPinReader m_ReaderAction;

    // output pins
    cPinWriter m_WriterFeedback;


    /*------------ PROPERTIES ----------*/
    // set property variables
    // coding convention:	m_prop******
    adtf::base::property_variable<tUInt32>    m_propUI32SamplesPerSec        = tUInt32(70000);  // number of samples per second
    //    /*------------ VARIABLES -----------*/
    // coding convention:	m_******
    object_ptr<adtf::services::IReferenceClock> m_pClock; // clock for timestamps

    TActionStruct::Data m_dataActionIn;


    TFeedbackStruct::Data m_dataFeedbackOut;

    // init TimeStamps
    tUInt32 m_ui32LastTimeStampAction;
    tUInt32 m_ui32LastTimeStampTimeTrigger;
    tBool m_bActionStarted; // true if action received; false if elapsed normally or broken by a new action signal
    tBool m_bLastActionElapsed;
    tFloat32 m_f32WaitFactor;
    tUInt32 counterWish;
    tUInt32 counter;
    tUInt32 time;
    tBool m_noReset;
public:
    /*------------ FUNCTIONS -----------*/
    // constructor
    TimerFilter();

    // destructor
    ~TimerFilter() = default;

    // configure
    virtual tResult Configure() override;

    // process
    virtual tResult Process(tTimeStamp tmTimeOfTrigger) override;

    // ProcessCarSpeedInput

    tResult ProcessActionInput(TActionStruct::Data );

    tResult ProcessTimeTriggerInput();
}; // TimerFilter
