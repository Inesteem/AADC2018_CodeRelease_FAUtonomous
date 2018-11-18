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
#include "OutputLimiter.h"
#include "ScmCommunication.h"

// This will define the filter and expose it via plugin class factory.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
        CID_OUTPUT_LIMITER_FILTER,                         // references to header file
        "OutputLimiter",                                   // label
        OutputLimiter,                                      // class
        adtf::filter::pin_trigger({"speed_controller"}));	// set trigger pin


// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
OutputLimiter::OutputLimiter()
{

    // ------------------------------------------
    SetName("OutputLimiter Constructor");

    // -----------------------------------------
    // set pins
    o_TSignalValue.registerPin(this, m_ReaderSpeed    , "speed_controller"     );
    o_TSignalValue.registerPin(this, m_ReaderSteering    , "steering_controller"     );

    o_TSignalValue.registerPin(this, m_WriterCarSpeed   , "car_speed");
    o_TSignalValue.registerPin(this, m_WriterCarSteering   , "car_steering");

    // -----------------------------------------
    // set property variables
    RegisterPropertyVariable("Speed 14", m_propF32LimitSpeedControllerOutput       );
    RegisterPropertyVariable("Speed 11", m_propF32LimitSteeringControllerOutput       );
    LOG_INFO("CONSTRUCTOR DONE!");
}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult OutputLimiter::Configure()
{
    m_ui32LastTimeStampSpeed = 0;
    m_ui32LastTimeStampSteering = 0;
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
tResult OutputLimiter::Process(tTimeStamp)
{


    // if new data is speed signal
    if(IS_OK(o_TSignalValue.readPin(m_ReaderSpeed, (void *) & m_dataCarSpeedOut, m_ui32LastTimeStampSpeed)))
    {
        m_ui32LastTimeStampSpeed = m_dataCarSpeedOut.ui32ArduinoTimestamp;

        // process data
        RETURN_IF_FAILED(ProcessSpeedInput(m_dataCarSpeedOut));
    }

    // if new data is steering signal
    if(IS_OK(o_TSignalValue.readPin(m_ReaderSteering, (void *) & m_dataCarSteeringOut, m_ui32LastTimeStampSteering)))
    {
        m_ui32LastTimeStampSteering = m_dataCarSteeringOut.ui32ArduinoTimestamp;
        // process data
        RETURN_IF_FAILED(ProcessSteeringInput(m_dataCarSteeringOut));
    }

    // done
    RETURN_NOERROR;
}
// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------

tResult OutputLimiter::ProcessSpeedInput(TSignalValue::Data inputSignal)
{
    boost::lock_guard<boost::mutex> lock(criticalSectionTransmit);
    TSignalValue::Data tmpSpeed;


    tmpSpeed = inputSignal;

        //LOG_INFO("speed: %f", tmpSpeed.f32Value);


    if(tmpSpeed.f32Value  == 8)
    {
        tmpSpeed.f32Value = 0.35;
    }
else if(tmpSpeed.f32Value == 5)
    {
        tmpSpeed.f32Value = 0.35;

        //do nothing
    }
    else if(tmpSpeed.f32Value == 1)
    {
        tmpSpeed.f32Value = 0.35;

        //do nothing
    }
    else if(tmpSpeed.f32Value == 2)
    {
        tmpSpeed.f32Value = 0.35;

        //do nothing
    }
    else if(tmpSpeed.f32Value == 3)
    {
        tmpSpeed.f32Value = 0.35;

        //do nothing
    }
    else if(tmpSpeed.f32Value == 9)
    {
        tmpSpeed.f32Value = 0.35;

        //do nothing
    }
    else if(tmpSpeed.f32Value == 10)
    {
        tmpSpeed.f32Value = 0.35;

        //do nothing
    }
    else if(tmpSpeed.f32Value == 11)
    {

        tmpSpeed.f32Value = m_propF32LimitSteeringControllerOutput;
        
        //do nothing
    }
    else if(tmpSpeed.f32Value == 12)
    {
        tmpSpeed.f32Value = 0.35;

        //do nothing
    }
    else if(tmpSpeed.f32Value == 13)
    {
        tmpSpeed.f32Value = 0.35;

        //do nothing
    }
    else if(tmpSpeed.f32Value == 14)
    {
        //tmpSpeed.f32Value = 0.8;
        tmpSpeed.f32Value = m_propF32LimitSpeedControllerOutput;

        //do nothing
    }
    else if(tmpSpeed.f32Value == 15)
    {
        tmpSpeed.f32Value = 0.35;

        //do nothing
    }
    else
    {
        //LOG_WARNING(cString::Format("cOutputLimiterTransmit::unspecified value for speedInput %f", tmpSpeed.f32Value));
        tmpSpeed.f32Value = 0;
    }

    //LOG_INFO(cString::Format("outputLimiter: speed is %f", tmpSpeed.f32Value));
    RETURN_IF_FAILED(o_TSignalValue.writePin(m_WriterCarSpeed, (void *) &tmpSpeed, m_pClock->GetStreamTime()));
    RETURN_NOERROR;

}

tResult OutputLimiter::ProcessSteeringInput(TSignalValue::Data inputSignal)
{

    TSignalValue::Data tmpSteering;
    tFloat32 maxControllerOut;

    tmpSteering = inputSignal;

    maxControllerOut = m_propF32LimitSteeringControllerOutput;
    //LOG_WARNING(cString::Format("max steering limited to %f",maxControllerOut ));

    if(tmpSteering.f32Value >= maxControllerOut)
    {
        tmpSteering.f32Value = maxControllerOut;
    }else if(tmpSteering.f32Value <= -maxControllerOut)
    {
        tmpSteering.f32Value = -maxControllerOut;
    }else if(fabs(tmpSteering.f32Value) < maxControllerOut)
    {
        //LOG_WARNING(cString::Format("steering should be %f", tmpSteering.f32Value));

        //do nothing
    }else
    {
        LOG_WARNING(cString::Format("cOutputLimiterTransmit::unspecified value for steeringInput %f", tmpSteering.f32Value));
        tmpSteering.f32Value = 0;
    }

    LOG_INFO(cString::Format("outputLimiter: steering is %f", tmpSteering.f32Value));
    RETURN_IF_FAILED(o_TSignalValue.writePin(m_WriterCarSteering, (void *) &tmpSteering, m_pClock->GetStreamTime()));
    RETURN_NOERROR;

}
