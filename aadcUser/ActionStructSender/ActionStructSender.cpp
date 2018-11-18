/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. Team FAUtonomous. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

This Filter is only used for testing purposes!!

**********************************************************************/

#include "stdafx.h"
#include "ActionStructSender.h"
#include "ScmCommunication.h"
// This will define the filter and expose it via plugin class factory.
// Class cTemplateFilter will be embedded to the filter and called by
// data trigger of "speed" by default.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
    CID_ACTION_STRUCT_SENDER_FILTER,		// references to header file
    "ActionStructSender",              // label
    ActionStructSender,                // class
    adtf::filter::pin_trigger({"inputSpeed"}));	// set trigger pin


// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
ActionStructSender::ActionStructSender()
{

    m_TSignalValue.registerPin(this, m_ReaderSpeed   , "inputSpeed");
    m_TActionStruct.registerPin(this, m_WriterAction1   , "ISDstart");
    m_TActionStruct.registerPin(this, m_WriterAction2   , "RSstart");
    m_TActionStruct.registerPin(this, m_WriterAction3   , "outputAction3");
    m_TPoseStruct.registerPin(this, m_WriterCarPose1   , "outputCarPose1");

//    RegisterPropertyVariable("FilterId"      , m_propFilterId);
//    RegisterPropertyVariable("Action Enabled", m_propEnabled);
//    RegisterPropertyVariable("Action Started", m_propStarted);
//    RegisterPropertyVariable("Action Command", m_propCommand);
}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult ActionStructSender::Configure()
{
    m_ui32TimeGapAction = 0.0;
    m_ui32TimeLast = 0;
    m_ui32TimeNow = 0;
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    // done
    RETURN_NOERROR;
}


// ------------------------------
// ---------- PROCESS -----------
// ------------------------------
// process() is always called each time a trigger occured (new data sample arrive).
// IMPORTANT: Make sure to read ALL samples of ALL readers!
// This is how to read a sample
tResult ActionStructSender::Process(tTimeStamp tmTimeOfTrigger)
{
    // -----------------------------------------
    // create pointer for samples
    object_ptr<const ISample> pReadSpeedSample;

    // -----------------------------------------
    // get the last speed sample
    if(IS_OK(m_ReaderSpeed.GetLastSample(pReadSpeedSample)))
    {
        m_action1.bEnabled =tTrue;
        m_action1.bStarted =tTrue;
        m_action1.ui32ArduinoTimestamp =m_pClock->GetStreamTime();
        m_action1.ui32Command =AC_ISD_START;
//        LOG_INFO("tramsmitted id: %d", m_action1.ui8FilterId);
//        LOG_INFO("tramsmitted command: %d", m_action1.ui32Command);

        m_action2.bEnabled =tTrue;
        m_action2.bStarted =tTrue;
        m_action2.ui32ArduinoTimestamp =m_pClock->GetStreamTime();

        m_action2.ui32Command =AC_RS_START;



        m_action3.bEnabled =tTrue;
        m_action3.bStarted =tTrue;
        m_action3.ui32ArduinoTimestamp =m_pClock->GetStreamTime();

        m_action3.ui32Command =AC_RS_START;

        // process data

        RETURN_IF_FAILED(m_TActionStruct.writePin(m_WriterAction1, (void *) &m_action1, m_pClock->GetStreamTime()));
        RETURN_IF_FAILED(m_TActionStruct.writePin(m_WriterAction2, (void *) &m_action2, m_pClock->GetStreamTime()));
        RETURN_IF_FAILED(m_TActionStruct.writePin(m_WriterAction3, (void *) &m_action3, m_pClock->GetStreamTime()));
        m_carPose1.ui32ArduinoTimestamp = m_pClock->GetStreamTime();
        m_carPose1.f32PosX = 0;
        m_carPose1.f32PosY = 0;
        m_carPose1.f32Roll = 0;
        m_carPose1.f32Pitch = 0;
        m_carPose1.f32Yaw = 0.78;
        m_carPose1.f32CarSpeed = 0;
        m_carPose1.f32Radius = 0;

        m_ui32TimeNow =m_pClock->GetStreamTime();
        RETURN_IF_FAILED(m_TPoseStruct.writePin(m_WriterCarPose1, (void *) &m_carPose1, m_pClock->GetStreamTime()));

    }

    // done
    RETURN_NOERROR;
}


// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------

//// TransmitSpeed
//// This is how you write a signal output!!
//tResult ActionStructSender::TransmitAction(tFloat32 inputSpeed)
//{
//    LOG_INFO("Start transmitting action");
//    // -----------------------------------------
//    // create ptr to smple
//    object_ptr<ISample> pWriteSample;

//    tBool   outputEnabled [5];
//    tBool   outputStarted [5];
//    tUInt32 outputCommand [5];
//    tUInt32 outputFilterId[5];

//    for(int i = 0; i < 5; i++)
//    {
//        outputEnabled [i] = false;
//        outputStarted [i] = false;
//        outputCommand [i] = 0;
//        outputFilterId[i] = 0;
//    }

//    // -----------------------------------------
//    // allocate for our sample
//    if(IS_OK(alloc_sample(pWriteSample)))
//    {
//        // create coding for signal
//        auto oCodec = m_ActionSampleFactory.MakeCodecFor(pWriteSample);

//        // save values
//        for(int i = 0; i < 5; i++)
//        {
//           if(inputSpeed > 10)
//            {
//                outputEnabled [i] = tBool  (m_propEnabled);
//                outputStarted [i] = tBool  (m_propStarted);
//                outputCommand [i] = tUInt32(m_propCommand);
//                outputFilterId[i] = tUInt  (m_propFilterId);
//            }
//        }

//        // set values
//        RETURN_IF_FAILED(oCodec.SetElementValue(o_ActionId.action1.ui8FilterId          , outputFilterId[0]));
//        RETURN_IF_FAILED(oCodec.SetElementValue(o_ActionId.action1.subAction.bEnabled   , outputEnabled [0]));
//        RETURN_IF_FAILED(oCodec.SetElementValue(o_ActionId.action1.subAction.bStarted   , outputStarted [0]));
//        RETURN_IF_FAILED(oCodec.SetElementValue(o_ActionId.action1.subAction.ui32Command, outputCommand [0]));
//        RETURN_IF_FAILED(oCodec.SetElementValue(o_ActionId.action2.ui8FilterId          , outputFilterId[1]));
//        RETURN_IF_FAILED(oCodec.SetElementValue(o_ActionId.action2.subAction.bEnabled   , outputEnabled [1]));
//        RETURN_IF_FAILED(oCodec.SetElementValue(o_ActionId.action2.subAction.bStarted   , outputStarted [1]));
//        RETURN_IF_FAILED(oCodec.SetElementValue(o_ActionId.action2.subAction.ui32Command, outputCommand [1]));
//        RETURN_IF_FAILED(oCodec.SetElementValue(o_ActionId.action3.ui8FilterId          , outputFilterId[2]));
//        RETURN_IF_FAILED(oCodec.SetElementValue(o_ActionId.action3.subAction.bEnabled   , outputEnabled [2]));
//        RETURN_IF_FAILED(oCodec.SetElementValue(o_ActionId.action3.subAction.bStarted   , outputStarted [2]));
//        RETURN_IF_FAILED(oCodec.SetElementValue(o_ActionId.action3.subAction.ui32Command, outputCommand [2]));
//        RETURN_IF_FAILED(oCodec.SetElementValue(o_ActionId.action4.ui8FilterId          , outputFilterId[3]));
//        RETURN_IF_FAILED(oCodec.SetElementValue(o_ActionId.action4.subAction.bEnabled   , outputEnabled [3]));
//        RETURN_IF_FAILED(oCodec.SetElementValue(o_ActionId.action4.subAction.bStarted   , outputStarted [3]));
//        RETURN_IF_FAILED(oCodec.SetElementValue(o_ActionId.action4.subAction.ui32Command, outputCommand [3]));
//        RETURN_IF_FAILED(oCodec.SetElementValue(o_ActionId.action5.ui8FilterId          , outputFilterId[4]));
//        RETURN_IF_FAILED(oCodec.SetElementValue(o_ActionId.action5.subAction.bEnabled   , outputEnabled [4]));
//        RETURN_IF_FAILED(oCodec.SetElementValue(o_ActionId.action5.subAction.bStarted   , outputStarted [4]));
//        RETURN_IF_FAILED(oCodec.SetElementValue(o_ActionId.action5.subAction.ui32Command, outputCommand [4]));
//    }

//    // -----------------------------------------
//    // write to sample
//    LOG_INFO(cString::Format("ActionStructSender: send %d%d%d%d/%d%d%d%d/%d%d%d%d/%d%d%d%d/%d%d%d%d",  outputFilterId[0], outputEnabled[0], outputStarted[0], outputCommand[0],
//                                                                                                       outputFilterId[1], outputEnabled[1], outputStarted[1], outputCommand[1],
//                                                                                                       outputFilterId[2], outputEnabled[2], outputStarted[2], outputCommand[2],
//                                                                                                       outputFilterId[3], outputEnabled[3], outputStarted[3], outputCommand[3],
//                                                                                                       outputFilterId[4], outputEnabled[4], outputStarted[4], outputCommand[4]));
//    m_WriterAction << pWriteSample << flush << trigger;

//    // done
//    RETURN_NOERROR;
//}
