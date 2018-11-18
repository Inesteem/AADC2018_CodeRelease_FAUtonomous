/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************
* This filter is used to calculate the average of last three ultrasonic signals.

* $Adapted by:: Xiangfei#  $Date:: 2018-08-01 12:44:00# status: adapted, not tested
**********************************************************************/

#include <mutex>
#include "stdafx.h"
#include "ustest.h"
#include "ScmCommunication.h"

// This will define the filter and expose it via plugin class factory.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
    CID_USTEST_FILTER,                         // references to header file
    "ustest",                                   // label
    ustest,                                     // class
    adtf::filter::pin_trigger({"USI"}));	// set trigger pin


// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
ustest::ustest()
{

    // ------------------------------------------
    SetName("USTEST Constructor");

        // INPUT: US
        createTUltrasonicStructMediaDescription(pUSDataType, m_USSampleFactory, o_USId, "tUltrasonicStruct");
        //createTSignalValueMediaDescription(pTSDataType, m_TSSampleFactory, o_TSId, "tSignalValue");
    // -----------------------------------------
    // set pins
    Register(m_ReaderUSInput       , "USI", pUSDataType       );
    Register(m_WriterUSOutput      , "USO", pUSDataType       );

    // -----------------------------------------
    // set property variables
    //RegisterPropertyVariable("number of measurements for avg", m_propFilterCount);
    //RegisterPropertyVariable("filter enabled?", m_propFilterEnable);
}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult ustest::Configure()
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
tResult ustest::Process(tTimeStamp tmTimeOfTrigger)
{
    // -----------------------------------------
    // create pointer for samples
    object_ptr<const ISample> pReadUltrasonicSample;


    // -----------------------------------------
    // get the last speed sample
    if(IS_OK(m_ReaderUSInput.GetLastSample(pReadUltrasonicSample)))
    {

        // decode last sample
        auto decoder = m_USSampleFactory.MakeDecoderFor(*pReadUltrasonicSample);
        RETURN_IF_FAILED(decoder.IsValid());
        // get values

        tUltrasonicStruct ultrasonicSignal;
        LOG_WARNING("about to decode!");
        RETURN_IF_FAILED(decoder.GetElementValue(o_USId.tSideLeft.f32Value                  , &ultrasonicSignal.tSideLeft.f32Value              ));
        RETURN_IF_FAILED(decoder.GetElementValue(o_USId.tSideLeft.ui32ArduinoTimestamp      , &ultrasonicSignal.tSideLeft.ui32ArduinoTimestamp  ));
        LOG_SUCCESS("decode successfully! sideLeft value: %f", ultrasonicSignal.tSideLeft.f32Value);
        RETURN_IF_FAILED(decoder.GetElementValue(o_USId.tSideRight.f32Value                  , &ultrasonicSignal.tSideRight.f32Value              ));
        RETURN_IF_FAILED(decoder.GetElementValue(o_USId.tSideRight.ui32ArduinoTimestamp      , &ultrasonicSignal.tSideRight.ui32ArduinoTimestamp  ));

        RETURN_IF_FAILED(decoder.GetElementValue(o_USId.tRearLeft.f32Value                  , &ultrasonicSignal.tRearLeft.f32Value              ));
        RETURN_IF_FAILED(decoder.GetElementValue(o_USId.tRearLeft.ui32ArduinoTimestamp      , &ultrasonicSignal.tRearLeft.ui32ArduinoTimestamp  ));

        RETURN_IF_FAILED(decoder.GetElementValue(o_USId.tRearRight.f32Value                  , &ultrasonicSignal.tRearRight.f32Value              ));
        RETURN_IF_FAILED(decoder.GetElementValue(o_USId.tRearRight.ui32ArduinoTimestamp      , &ultrasonicSignal.tRearRight.ui32ArduinoTimestamp  ));

        RETURN_IF_FAILED(decoder.GetElementValue(o_USId.tRearCenter.f32Value                  , &ultrasonicSignal.tRearCenter.f32Value              ));
        RETURN_IF_FAILED(decoder.GetElementValue(o_USId.tRearCenter.ui32ArduinoTimestamp      , &ultrasonicSignal.tRearCenter.ui32ArduinoTimestamp  ));


        // process data

        RETURN_IF_FAILED(transmitUSStruct(ultrasonicSignal));
    }
        //tUltrasonicStruct output_Ultrasonic_Struct;

        // if ustest command was activated

//        if(m_propFilterEnable)
//        {
//                output_Ultrasonic_Struct = CalcUSMeanValue(ultrasonicSignal);
//        }else{// else send input as output
//                output_Ultrasonic_Struct = ultrasonicSignal;
//            }
//            RETURN_IF_FAILED(transmitUSStruct(output_Ultrasonic_Struct));
//    }
    // done
    RETURN_NOERROR;
}
// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------


// transmit ultrasonic signals
tResult ustest::transmitUSStruct(tUltrasonicStruct inputSignal)
{
    object_ptr<ISample> pWriteSample;
    // allocate for our sample
    RETURN_IF_FAILED(alloc_sample(pWriteSample));

    // create coding for signal
    auto codec = m_USSampleFactory.MakeCodecFor(pWriteSample);

    // set values
    RETURN_IF_FAILED(codec.SetElementValue(o_USId.tSideLeft.f32Value , inputSignal.tSideLeft.f32Value));
    RETURN_IF_FAILED(codec.SetElementValue(o_USId.tSideLeft.ui32ArduinoTimestamp , inputSignal.tSideLeft.ui32ArduinoTimestamp));

    RETURN_IF_FAILED(codec.SetElementValue(o_USId.tSideRight.f32Value , inputSignal.tSideRight.f32Value));
    RETURN_IF_FAILED(codec.SetElementValue(o_USId.tSideRight.ui32ArduinoTimestamp , inputSignal.tSideRight.ui32ArduinoTimestamp));

    RETURN_IF_FAILED(codec.SetElementValue(o_USId.tRearLeft.f32Value , inputSignal.tRearLeft.f32Value));
    RETURN_IF_FAILED(codec.SetElementValue(o_USId.tRearLeft.ui32ArduinoTimestamp , inputSignal.tRearLeft.ui32ArduinoTimestamp));

    RETURN_IF_FAILED(codec.SetElementValue(o_USId.tRearRight.f32Value , inputSignal.tRearRight.f32Value));
    RETURN_IF_FAILED(codec.SetElementValue(o_USId.tRearRight.ui32ArduinoTimestamp , inputSignal.tRearRight.ui32ArduinoTimestamp));

    RETURN_IF_FAILED(codec.SetElementValue(o_USId.tRearCenter.f32Value , inputSignal.tRearCenter.f32Value));
    RETURN_IF_FAILED(codec.SetElementValue(o_USId.tRearCenter.ui32ArduinoTimestamp , inputSignal.tRearCenter.ui32ArduinoTimestamp));

    // write to sample
    m_WriterUSOutput << pWriteSample << flush << trigger;

    // done
    RETURN_NOERROR;
}

