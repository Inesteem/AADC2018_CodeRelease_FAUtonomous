/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

This filter is an example filter to learn ADTF.
Filter was presented in first adtf online tutorial by Audi.
It takes 'speed' and the laser scanner as input and will signal an emergency break.

**********************************************************************/

#include "stdafx.h"
#include "cMean.h"
#include "ScmCommunication.h"
#include "ADTF3_helper.h"

//TODO:change the following description
// This will define the filter and expose it via plugin class factory.
// Class FTO_NegateSpeedController will be embedded to the filter and called by
// data trigger of "speed" by default.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
        CID_FTO_MEAN_FILTER,		// references to header file
        "Mean",			// label
        cMean,				// class
        adtf::filter::pin_trigger({"signal_input"}));	// set trigger pin

// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
cMean::cMean()
{
    signal_value.registerPin(this, m_signal_input, "signal_input");
        // -----------------------------------------
        // set property variables
    m_propNumVals = 500;
    m_propIgnoreZeroVals = tTrue;
    RegisterPropertyVariable("NumValsforMean", m_propNumVals);
    RegisterPropertyVariable("IgnoreZeroVals", m_propIgnoreZeroVals);
}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult cMean::Configure()
{
        //get clock object
        RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

        RETURN_NOERROR;
}


// ------------------------------
// ---------- PROCESS -----------
// ------------------------------
// process() is always called each time a trigger occured (new data sample arrive).
// IMPORTANT: Make sure to read ALL samples of ALL readers!

tResult cMean::ProcessSignalValueInput(){
    boost::lock_guard<boost::mutex> lock(cs_signalval);
    static tUInt32 num = 0;

    TSignalValue::Data data;
    tResult res;
    static tTimeStamp last_timestamp = 0;
    if(IS_FAILED(res = signal_value.readPin(m_signal_input, (void *) &data,last_timestamp))){
        RETURN_ERROR(res);
    }
    last_timestamp = data.ui32ArduinoTimestamp;

    LOG_INFO(cString::Format("value is: %f"), data.f32Value);
    RETURN_NOERROR;

    if(m_propIgnoreZeroVals && data.f32Value == 0) RETURN_NOERROR;

    m_acc_signal_values += static_cast<tFloat64>(data.f32Value);
    ++num;
  //  m_signal_value_list.push_back(data.f32Value);
   // if(m_signal_value_list.size() == m_propNumVals){
     if(num == m_propNumVals){
       // m_acc_signal_values -= m_signal_value_list.front();
       // m_signal_value_list.pop_front();
              auto mean = m_acc_signal_values / (double) m_propNumVals;
              LOG_SUCCESS(cString::Format("mean of signal value input : %f ",(float) mean));
              m_acc_signal_values = 0;
              num = 0;
 //           m_signal_value_list.clear();
    }
  //  if(m_signal_value_list.size() == m_propNumVals) {
  //      auto mean = m_acc_signal_values / (double) m_propNumVals;
  //      LOG_SUCCESS(cString::Format("mean of signal value input : %f ",(float) mean));
  //  }

    RETURN_NOERROR;
}

tResult cMean::Process(tTimeStamp tmTimeOfTrigger)
{

    if (IS_OK(ProcessSignalValueInput()))
    {
    }
    RETURN_NOERROR;
}





