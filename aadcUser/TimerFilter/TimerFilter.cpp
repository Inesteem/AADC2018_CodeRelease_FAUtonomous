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
#include "TimerFilter.h"
#include "ScmCommunication.h"

// This will define the filter and expose it via plugin class factory.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
        CID_TIMER_FILTER_FILTER,                         // references to header file
        "TimerFilter",                                   // label
        TimerFilter,                                      // class
        adtf::filter::timer_trigger(200000));	// set trigger pin


// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
TimerFilter::TimerFilter()
{

    // ------------------------------------------
    SetName("TimerFilter Constructor");

    // -----------------------------------------
    // set pins
    o_TSignalValue.registerPin(this, m_ReaderTriggerTime    , "time_trigger_pin"     );
    o_TActionStruct.registerPin(this, m_ReaderAction    , "action"     );

    o_TFeedbackStruct.registerPin(this, m_WriterFeedback   , "feedback");


    // -----------------------------------------
    // set property variables
    RegisterPropertyVariable("time step between trigger signals", m_propUI32SamplesPerSec       );
    //    RegisterPropertyVariable("Steering limited", m_propF32LimitSteeringControllerOutput       );
    LOG_INFO("CONSTRUCTOR DONE!");
}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult TimerFilter::Configure()
{
    counter = 0;
    counterWish = 0;
    time = 0;
    m_bActionStarted = tFalse;
    m_bLastActionElapsed = tTrue;
    m_f32WaitFactor = 1.0;
    m_ui32LastTimeStampAction = -10;
    m_noReset = tFalse;

    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    LOG_INFO("CONFIG DONE!");
    // done
    RETURN_NOERROR;
}


// ------------------------------
// ---------- PROCESS -----------
// ------------------------------
// process() is always called each time a trigger occured (new data sample arrive).
// IMPORTANT: Make sure to read ALL samples of ALL readers!
tResult TimerFilter::Process(tTimeStamp tmTimeOfTrigger)
{

    TActionStruct::Data tmp_action;
    if(IS_OK(o_TActionStruct.readPin(m_ReaderAction, (void *) & tmp_action, m_ui32LastTimeStampAction)))
    {
        m_ui32LastTimeStampAction = tmp_action.ui32ArduinoTimestamp;
        if(tmp_action.ui8FilterId == 4)
        {
            LOG_INFO("timer action received, about to process");
            m_dataActionIn = tmp_action;

            m_noReset = tFalse;

            if(tmp_action.ui32Command == 4001) m_f32WaitFactor = 0.001;
            else if(tmp_action.ui32Command == 4002) m_f32WaitFactor = 0.002;
            else if(tmp_action.ui32Command == 4003) m_f32WaitFactor = 0.003;
            else if(tmp_action.ui32Command == 4004) m_f32WaitFactor = 0.004;
            else if(tmp_action.ui32Command == 4005) m_f32WaitFactor = 0.005;
            else if(tmp_action.ui32Command == 4006) m_f32WaitFactor = 0.006;
            else if(tmp_action.ui32Command == 4007) m_f32WaitFactor = 0.007;
            else if(tmp_action.ui32Command == 4008) m_f32WaitFactor = 0.008;
            else if(tmp_action.ui32Command == 4009) m_f32WaitFactor = 0.009;

            else if(tmp_action.ui32Command == 4010) m_f32WaitFactor = 1;
            else if(tmp_action.ui32Command == 4020) m_f32WaitFactor = 2;
            else if(tmp_action.ui32Command == 4030) m_f32WaitFactor = 3;
            else if(tmp_action.ui32Command == 4040) m_f32WaitFactor = 4;
            else if(tmp_action.ui32Command == 4050) m_f32WaitFactor = 5;
            else if(tmp_action.ui32Command == 4060) m_f32WaitFactor = 6;
            else if(tmp_action.ui32Command == 4070) m_f32WaitFactor = 7;
            else if(tmp_action.ui32Command == 4080) m_f32WaitFactor = 8;
            else if(tmp_action.ui32Command == 4090) m_f32WaitFactor = 9;
            else if(tmp_action.ui32Command == 4100) m_f32WaitFactor = 10;
            else if(tmp_action.ui32Command == 4110) m_f32WaitFactor = 11;
            else if(tmp_action.ui32Command == 4120) m_f32WaitFactor = 12;
            else if(tmp_action.ui32Command == 4130) m_f32WaitFactor = 13;
            else if(tmp_action.ui32Command == 4140) m_f32WaitFactor = 14;
            else if(tmp_action.ui32Command == 4150) m_f32WaitFactor = 15;
            else if(tmp_action.ui32Command == 4160) m_f32WaitFactor = 16;
            else if(tmp_action.ui32Command == 4170) m_f32WaitFactor = 17;
            else if(tmp_action.ui32Command == 4180) m_f32WaitFactor = 18;
            else if(tmp_action.ui32Command == 4190) m_f32WaitFactor = 19;
            else if(tmp_action.ui32Command == 4200) m_f32WaitFactor = 20;
            // no reset
            else if(tmp_action.ui32Command == 4510) m_f32WaitFactor = 1;
            else if(tmp_action.ui32Command == 4520) m_f32WaitFactor = 2;
            else if(tmp_action.ui32Command == 4530) m_f32WaitFactor = 3;
            else if(tmp_action.ui32Command == 4540) m_f32WaitFactor = 4;
            else if(tmp_action.ui32Command == 4550) m_f32WaitFactor = 5;
            else if(tmp_action.ui32Command == 4560) m_f32WaitFactor = 6;
            else if(tmp_action.ui32Command == 4570) m_f32WaitFactor = 7;
            else if(tmp_action.ui32Command == 4580) m_f32WaitFactor = 8;
            else if(tmp_action.ui32Command == 4590) m_f32WaitFactor = 9;
            else if(tmp_action.ui32Command == 4600) m_f32WaitFactor = 10;
            else if(tmp_action.ui32Command == 4610) m_f32WaitFactor = 11;
            else if(tmp_action.ui32Command == 4620) m_f32WaitFactor = 12;
            else if(tmp_action.ui32Command == 4630) m_f32WaitFactor = 13;
            else if(tmp_action.ui32Command == 4640) m_f32WaitFactor = 14;
            else if(tmp_action.ui32Command == 4650) m_f32WaitFactor = 15;
            else if(tmp_action.ui32Command == 4660) m_f32WaitFactor = 16;
            else if(tmp_action.ui32Command == 4670) m_f32WaitFactor = 17;
            else if(tmp_action.ui32Command == 4680) m_f32WaitFactor = 18;
            else if(tmp_action.ui32Command == 4690) m_f32WaitFactor = 19;
            else if(tmp_action.ui32Command == 4700) m_f32WaitFactor = 20;

            if(tmp_action.ui32Command >= 4500 &&tmp_action.ui32Command <=4700) m_noReset = tTrue;

            if(m_bLastActionElapsed == tFalse && m_noReset == tFalse)
            {
                counter = 0;
            }
            counterWish = (tUInt32)(m_propUI32SamplesPerSec * m_f32WaitFactor);
            //LOG_INFO("counterwish: %d", counterWish);
//            counterWish = 0;
            m_bActionStarted = tTrue;

        }
    }

    TSignalValue::Data tmp_trigger;
    m_ui32LastTimeStampTimeTrigger = tmp_trigger.ui32ArduinoTimestamp;
    if(m_bActionStarted)
    {
        counter ++;
        //LOG_INFO("counter %d, counter limit %d" , counter, counterWish);
        if(counter > counterWish)
        {
            counter = 0;
            m_bActionStarted = tFalse;
            m_bLastActionElapsed = tTrue;
            TFeedbackStruct::Data tmp_feedback;
            tmp_feedback.ui8FilterId = 4;
            tmp_feedback.ui32FeedbackStatus = 4001;
            LOG_INFO("timer elapsed, feedback should be sent");
            RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedback, (void *) &tmp_feedback, m_pClock->GetStreamTime()));
        }
    }
    // process data
    // done
    RETURN_NOERROR;
}
// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------
