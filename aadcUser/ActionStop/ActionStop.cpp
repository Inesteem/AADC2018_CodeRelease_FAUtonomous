/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

This filter is used for braking.
It takes speed and an tActionCommand as pin input and will brake with brakingSpeed = (-1) * (currentSpeed * m_propPercentageActiveBraking).
Braking will only be triggerm_f32CarStoppedThresholded if actionCommand == AC_SA_STOP_CAR.

<<<<<<< HEAD
Ported by Pham, improved (use classes to shorten and simplify) and tested by Illmer Status:doing (still untested and not apdapted to the new ActionCommand
=======
Ported by Pham, improved (use classes to shorten and simplify) and tested (not done yet) by Illmer Status:doing (still untested and not apdapted to the new ActionCommand) adapted to the new classes
>>>>>>> 9883f71022b214329859460838f04dd23b994a10

**********************************************************************/

#include <mutex>
#include "stdafx.h"
#include "ActionStop.h"
#include "ScmCommunication.h"
#include <ADTF3_helper.h>


// This will define the filter and expose it via plugin class factory.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
        CID_ACTION_STOP_FILTER,                         // references to header file
        "cActionStop",                                   // label
        ActionStop,                                     // class
        adtf::filter::pin_trigger({"actionInput"}));	// set trigger pin


// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
ActionStop::ActionStop()
{
    // ------------------------------------------
    // initialize variables
    // set threshold
    m_f32CarStoppedThreshold = tFloat(0.5f);


    // -----------------------------------------
    // set pins
    // input
    m_SpeedSignal.registerPin(this,      m_ReaderSpeed,     "speedInput");
    m_ActionCommand.registerPin(this,    m_ReaderAction,    "actionInput");

    // output

    m_SpeedSignal.registerPin(this,     m_WriterSpeed,      "speedOutput");
    m_FeedbackStruct.registerPin(this,  m_WriterFeedback,   "feedbackOutput");

    // -----------------------------------------
    // set property variables
    RegisterPropertyVariable("Percentage for braking [%]" , m_propPercentageActiveBraking);
    RegisterPropertyVariable("Debugging activted"         , m_propDebugActivted          );//

}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult ActionStop::Configure()
{
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
tResult ActionStop::Process(tTimeStamp tmTimeOfTrigger)
{

    if(IS_OK(m_ActionCommand.readPin(m_ReaderAction, (void *) &tmp_actionStruct, m_timestampAction )))
    {

        m_timestampAction = tmp_actionStruct.ui32ArduinoTimestamp;
        if (tmp_actionStruct.ui8FilterId != 8)
        {
            RETURN_NOERROR;
        }

        TFeedbackStruct::Data feedbackStop;
        feedbackStop.ui8FilterId = F_STOP_ACTION;
        feedbackStop.ui32FeedbackStatus = FB_SA_STOPPED;

        RETURN_IF_FAILED(m_FeedbackStruct.writePin(m_WriterFeedback, (void *) &feedbackStop, m_pClock->GetStreamTime()));

        RETURN_NOERROR;
        //    }//here
    }

    //    // -----------------------------------------
    //    // get the last speed sample

    //    static tTimeStamp lasttmspeed = 0;

    //    counter_process += 1;
    ////    LOG_INFO("action stop counter process: %d", counter_process);

    //    if(IS_OK(m_SpeedSignal.readPin(m_ReaderSpeed, (void *) &speedSignal, lasttmspeed)))
    //    {
    //        counter_speed += 1;
    //        LOG_INFO("action stop counter speed: %d", counter_speed);
    //        // if ActionStop command was activated
    //        lasttmspeed = speedSignal.ui32ArduinoTimestamp;
    //        if(m_propCommandActivated)
    //        {
    //            LOG_INFO("stop command from scm received! Car speed: %f", speedSignal.f32Value);
    //            // brake!
    //            speedSignal.f32Value = - (speedSignal.f32Value * m_propPercentageActiveBraking);

    //            // if the car has finally stopped

    //            if(speedSignal.f32Value <= m_f32CarStoppedThreshold)
    //            {
    //                m_propCommandActivated = tFalse;
    //                TFeedbackStruct::Data feedbackStop;
    //                feedbackStop.ui8FilterId = F_STOP_ACTION;
    //                feedbackStop.ui32FeedbackStatus = FB_SA_STOPPED;
    //                LOG_INFO("About to send feedback %d from action stop!", feedbackStop.ui32FeedbackStatus);
    //                RETURN_IF_FAILED(m_FeedbackStruct.writePin(m_WriterFeedback, (void *) &feedbackStop, m_pClock->GetStreamTime()));
    //            }

    //        }
    //        //#endif
    //        // transmit speed data
    //        RETURN_IF_FAILED(m_SpeedSignal.writePin(m_WriterSpeed, (void *) &speedSignal, m_pClock->GetStreamTime()));
    //    }
    //    //}


    //    // get the last action commmand sample

    //    static tTimeStamp lasttmaction = 0;
    //    if(IS_OK(m_ActionCommand.readPin(m_ReaderAction, (void *) &tmp_actionStruct, lasttmaction)))
    //    {
    //        counter_action += 1;
    //        LOG_INFO("action stop counter action: %d", counter_action);
    //        lasttmaction = tmp_actionStruct.ui32ArduinoTimestamp;
    //        if (tmp_actionStruct.ui8FilterId != 8)
    //        {
    //            RETURN_NOERROR;
    //        }
    //        LOG_INFO(cString::Format("ActionStruct: Id: %d, Enabled: %d, Started: %d, Command:  %d /n", tmp_actionStruct.ui8FilterId, tmp_actionStruct.bEnabled, tmp_actionStruct.bStarted, tmp_actionStruct.ui32Command));
    //        // done
    //        // brake if command was made
    //        if(tmp_actionStruct.ui32Command == AC_SA_STOP_CAR)
    //        {
    //            m_propCommandActivated = tTrue;
    //        }
    //        RETURN_NOERROR;
    //        //    }//here
    //    }
    RETURN_NOERROR;
}
// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------
