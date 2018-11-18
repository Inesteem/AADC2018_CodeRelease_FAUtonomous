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
#include "UltrasonicStructSplit.h"
#include "ScmCommunication.h"

// This will define the filter and expose it via plugin class factory.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
        CID_ULTRASONIC_STRUCT_SPLIT_FILTER,                         // references to header file
        "UltrasonicStructSplit",                                   // label
        UltrasonicStructSplit,                                      // class
        adtf::filter::pin_trigger({"UltrasonicInput"}));	// set trigger pin


// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
UltrasonicStructSplit::UltrasonicStructSplit()
{

    // ------------------------------------------
    SetName("UltrasonicStructSplit Constructor");

    // -----------------------------------------
    // set pins
    // input
    o_UltrasonicStruct.registerPin(this, m_ReaderUltrasonic    , "UltrasonicInput"     );
    // output
    o_TSignalValue.registerPin(this, m_WriterSideLeft   , "SideLeftOutput");
    o_TSignalValue.registerPin(this, m_WriterSideRight   , "SideRightOutput");
    o_TSignalValue.registerPin(this, m_WriterRearLeft   , "RearLeftOutput");
    o_TSignalValue.registerPin(this, m_WriterRearCenter   , "RearCenterOutput");
    o_TSignalValue.registerPin(this, m_WriterRearRight   , "RearRightOutput");

}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult UltrasonicStructSplit::Configure()
{
    m_ui32TimeStampUS = 0;
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    LOG_INFO(cString::Format("Configuration finished!"));
    // done
    RETURN_NOERROR;
}


// ------------------------------
// ---------- PROCESS -----------
// ------------------------------
// process() is always called each time a trigger occured (new data sample arrive).
// IMPORTANT: Make sure to read ALL samples of ALL readers!
tResult UltrasonicStructSplit::Process(tTimeStamp tmTimeOfTrigger)
{
    TUltrasonicStruct::Data tmp_dataUltrasonicSignal;
    if(IS_OK(o_UltrasonicStruct.readPin(m_ReaderUltrasonic, (void *) & tmp_dataUltrasonicSignal, m_ui32TimeStampUS)))
    {

        m_ui32TimeStampUS = tmp_dataUltrasonicSignal.tSideLeft.ui32ArduinoTimestamp;

        // process data
        RETURN_IF_FAILED(ProcessUSInput(tmp_dataUltrasonicSignal));

        RETURN_IF_FAILED(o_TSignalValue.writePin(m_WriterSideLeft, (void *) &m_SideLeftOut, m_pClock->GetStreamTime()));
        RETURN_IF_FAILED(o_TSignalValue.writePin(m_WriterSideRight, (void *) &m_SideRightOut, m_pClock->GetStreamTime()));
        RETURN_IF_FAILED(o_TSignalValue.writePin(m_WriterRearLeft, (void *) &m_RearLeftOut, m_pClock->GetStreamTime()));
        RETURN_IF_FAILED(o_TSignalValue.writePin(m_WriterRearCenter, (void *) &m_RearCenterOut, m_pClock->GetStreamTime()));
        RETURN_IF_FAILED(o_TSignalValue.writePin(m_WriterRearRight, (void *) &m_RearRightOut, m_pClock->GetStreamTime()));
    }
    // done
    RETURN_NOERROR;
}
// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------

tResult UltrasonicStructSplit::ProcessUSInput(TUltrasonicStruct::Data tmp_dataReceivedUltrasonicData)
{
    m_SideLeftOut.f32Value = tmp_dataReceivedUltrasonicData.tSideLeft.f32Value;
    m_SideLeftOut.ui32ArduinoTimestamp = tmp_dataReceivedUltrasonicData.tSideLeft.ui32ArduinoTimestamp;

    m_SideRightOut.f32Value = tmp_dataReceivedUltrasonicData.tSideRight.f32Value;
    m_SideRightOut.ui32ArduinoTimestamp = tmp_dataReceivedUltrasonicData.tSideRight.ui32ArduinoTimestamp;

    m_RearLeftOut.f32Value = tmp_dataReceivedUltrasonicData.tRearLeft.f32Value;
    m_RearLeftOut.ui32ArduinoTimestamp = tmp_dataReceivedUltrasonicData.tRearLeft.ui32ArduinoTimestamp;

    m_RearCenterOut.f32Value = tmp_dataReceivedUltrasonicData.tRearCenter.f32Value;
    m_RearCenterOut.ui32ArduinoTimestamp = tmp_dataReceivedUltrasonicData.tRearCenter.ui32ArduinoTimestamp;

    m_RearRightOut.f32Value = tmp_dataReceivedUltrasonicData.tRearRight.f32Value;
    m_RearRightOut.ui32ArduinoTimestamp = tmp_dataReceivedUltrasonicData.tRearRight.ui32ArduinoTimestamp;

    RETURN_NOERROR;
}

