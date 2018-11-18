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
#include "UltrasonicMeanFilter.h"
#include "ScmCommunication.h"

// This will define the filter and expose it via plugin class factory.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
    CID_ULTRASONIC_MEAN_FILTER_FILTER,                         // references to header file
    "UltrasonicMeanFilter",                                   // label
    UltrasonicMeanFilter,                                      // class
    adtf::filter::pin_trigger({"UltrasonicInput"}));	// set trigger pin


// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
UltrasonicMeanFilter::UltrasonicMeanFilter()
{

    // ------------------------------------------
    SetName("UltrasonicMeanFilter Constructor");

    // -----------------------------------------
    // set pins
    o_UltrasonicStruct.registerPin(this, m_ReaderUltrasonic    , "UltrasonicInput"     );
    o_UltrasonicStruct.registerPin(this, m_WriterUltrasonic   , "UltrasonicMeanOutput");


    // -----------------------------------------
    // set property variables
    RegisterPropertyVariable("number of measurements for avg"   , m_propIntFilterCount );
    RegisterPropertyVariable("filter enabled?"                  , m_propBoolFilterEnable);
}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult UltrasonicMeanFilter::Configure()
{
    m_ui32TimeStampUS = 0;

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
tResult UltrasonicMeanFilter::Process(tTimeStamp tmTimeOfTrigger)
{
    boost::lock_guard<boost::mutex> lock(m_oProcessUltrasonicInput);



    TUltrasonicStruct::Data tmp_dataOutputUltrasonicSignal;
    TUltrasonicStruct::Data tmp_dataUltrasonicSignal;

    if(IS_OK(o_UltrasonicStruct.readPin(m_ReaderUltrasonic, (void *) & tmp_dataUltrasonicSignal, m_ui32TimeStampUS))){

        m_ui32TimeStampUS = tmp_dataUltrasonicSignal.tSideLeft.ui32ArduinoTimestamp;
        // process data
        // if UltrasonicMeanFilter command was activated
        if(m_propBoolFilterEnable)
        {
            tmp_dataOutputUltrasonicSignal = CalcUSMeanValue(tmp_dataUltrasonicSignal);

        }else
        {// else send input as output
            tmp_dataOutputUltrasonicSignal = tmp_dataUltrasonicSignal;

        }
        RETURN_IF_FAILED(o_UltrasonicStruct.writePin(m_WriterUltrasonic, (void *) &tmp_dataOutputUltrasonicSignal, m_pClock->GetStreamTime()));
        // done
    }
    RETURN_NOERROR;
}
// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------

TUltrasonicStruct::Data UltrasonicMeanFilter::CalcUSMeanValue(TUltrasonicStruct::Data tmp_dataReceivedUltrasonicData)
{
    // Push last samples in US lists

    m_listMeanSideLeft.push_back(tmp_dataReceivedUltrasonicData.tSideLeft.f32Value);
    m_listMeanSideRight.push_back(tmp_dataReceivedUltrasonicData.tSideRight.f32Value);
    m_listMeanRearLeft.push_back(tmp_dataReceivedUltrasonicData.tRearLeft.f32Value);
    m_listMeanRearCenter.push_back(tmp_dataReceivedUltrasonicData.tRearCenter.f32Value);
    m_listMeanRearRight.push_back(tmp_dataReceivedUltrasonicData.tRearRight.f32Value);

    // remove first element in lists when count is reached
    if ((int) m_listMeanSideLeft.size() > m_propIntFilterCount)
    {

        m_listMeanSideLeft.pop_front();
        m_listMeanSideRight.pop_front();
        m_listMeanRearLeft.pop_front();
        m_listMeanRearCenter.pop_front();
        m_listMeanRearRight.pop_front();
    }


    // calc mean values for all sensors and replace old values in received data
    // Side Left average
    float sum = 0;
    for (std::list<float>::iterator p = m_listMeanSideLeft.begin(); p != m_listMeanSideLeft.end(); ++p)
    {
        sum += (float)*p;
    }
    tmp_dataReceivedUltrasonicData.tSideLeft.f32Value = sum / m_listMeanSideLeft.size();

    // Side Right average
    sum = 0;
    for (std::list<float>::iterator p = m_listMeanSideRight.begin(); p != m_listMeanSideRight.end(); ++p)
    {
        sum += (float)*p;
    }
    tmp_dataReceivedUltrasonicData.tSideRight.f32Value = sum / m_listMeanSideRight.size();

    // Rear Left average
    sum = 0;
    for (std::list<float>::iterator p = m_listMeanRearLeft.begin(); p != m_listMeanRearLeft.end(); ++p)
    {
        sum += (float)*p;
    }
    tmp_dataReceivedUltrasonicData.tRearLeft.f32Value = sum / m_listMeanRearLeft.size();

    // Rear Center average
    sum = 0;
    for (std::list<float>::iterator p = m_listMeanRearCenter.begin(); p != m_listMeanRearCenter.end(); ++p)
    {
        sum += (float)*p;
    }
    tmp_dataReceivedUltrasonicData.tRearCenter.f32Value = sum / m_listMeanRearCenter.size();

    // Rear Right average
    sum = 0;
    for (std::list<float>::iterator p = m_listMeanRearRight.begin(); p != m_listMeanRearRight.end(); ++p)
    {
        sum += (float)*p;
    }
    tmp_dataReceivedUltrasonicData.tRearRight.f32Value = sum / m_listMeanRearRight.size();

    // US output struct with enw values
    return tmp_dataReceivedUltrasonicData;

}

