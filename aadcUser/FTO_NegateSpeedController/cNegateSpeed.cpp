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
#include "cNegateSpeed.h"
#include "ScmCommunication.h"
#include "ADTF3_helper.h"

//TODO:change the following description
// This will define the filter and expose it via plugin class factory.
// Class FTO_NegateSpeedController will be embedded to the filter and called by
// data trigger of "speed" by default.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
        CID_FTO_NEGATE_SPEED_CONTROLLER_FILTER,		// references to header file
        "FTO_NegateSpeedController",			// label
        cNegateSpeed,				// class
        adtf::filter::pin_trigger({"speed_controller"}));	// set trigger pin

// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
cNegateSpeed::cNegateSpeed()
{
        // ------------------------------------------
        // create pointers for adtf streamtypes
        // coding convention:	p******DataType
        object_ptr<IStreamType> pSpeedDataType;


        // ------------------------------------------
        // load the streamtypes
        // speed signal value
//        if(IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pSpeedDataType, m_SpeedSampleFactory)))
//        {
//                // if loading was successful then get index from aadc description html file
//                adtf_ddl::access_element::find_index(m_SpeedSampleFactory, cString("ui32ArduinoTimeStamp"), o_SpeedSignalId.timestamp);
//                adtf_ddl::access_element::find_index(m_SpeedSampleFactory, cString("f32Value"            ), o_SpeedSignalId.value    );
//        }
//        else
//        {
//                // did not find datatype in description file
//                LOG_INFO("no mediadescription for tSignalValue found! :(");
//        }

        // -----------------------------------------
        // set pins
        Register(m_ReaderSpeedController,   "speed_controller", pSpeedDataType);
        Register(m_WriterCarSpeed,          "speed_car",        pSpeedDataType);

        // -----------------------------------------
        // set property variables
        RegisterPropertyVariable("Max_Speed_Controller_Out", m_propMaxSpeedControllerOut);
}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult cNegateSpeed::Configure()
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
tResult cNegateSpeed::Process(tTimeStamp tmTimeOfTrigger)
{
        // -----------------------------------------
        // create pointer for samples
        object_ptr<const ISample> pReadSpeedSample;

        // -----------------------------------------
        // get the last speed sample
        if(IS_OK(m_ReaderSpeedController.GetLastSample(pReadSpeedSample)))
        {

                // get values
                tSignalValue speedSignal;
                tFloat32 outputSpeed = 0;
                tUInt32 outputTimestamp = 0;

                // decode last sample
                LOG_SUCCESS(cString::Format("Log_N1 ReadSpeedSample: %f", pReadSpeedSample));
                auto o_Decoder = m_SpeedSampleFactory.MakeDecoderFor(*pReadSpeedSample);
                RETURN_IF_FAILED(o_Decoder.IsValid());
                LOG_SUCCESS(cString::Format("Log_N2 DecoderValid"));
                RETURN_IF_FAILED(o_Decoder.GetElementValue(o_SpeedSignalId.ui32Timestamp, &outputTimestamp));
                LOG_SUCCESS(cString::Format("Log_N3 GetTimestamp"));
                RETURN_IF_FAILED(o_Decoder.GetElementValue(o_SpeedSignalId.f32Value    , &outputSpeed));
                LOG_SUCCESS(cString::Format("Log_N4 GetSpeed"));
                speedSignal.f32Value = outputSpeed;
                LOG_SUCCESS(cString::Format("Log_N5 OutputSpeed: &f", outputSpeed));
                speedSignal.ui32ArduinoTimestamp = outputTimestamp;

                LOG_SUCCESS(cString::Format("Log_N6 OutputTimestamp: &f", outputTimestamp));

                // process data
                ProcessCarSpeedInput(speedSignal);
                TransmitSpeed();
                //transmitSignalValue(m_WriterSpeed, m_pClock->GetStreamTime(), m_SpeedSampleFactory, o_SpeedSignalId.timestamp, 0, o_SpeedSignalId.value, outputSpeed);
                //RETURN_IF_FAILED(TransmitSpeed(speedSignal));
        }


        // done
        RETURN_NOERROR;
}


// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------

//ProcessCarSpeedInput
tResult cNegateSpeed::ProcessCarSpeedInput(tSignalValue speed)
{
    speedOut = speed;
    RETURN_NOERROR;
}

// Transmitspeed
// writing processed samples
tResult cNegateSpeed::TransmitSpeed()
{
    TSignalValue::Data tmp_speed;
    tFloat32 maxControllerOut = m_propMaxSpeedControllerOut;
    tmp_speed.f32Value = -speedOut.f32Value;


    if(tmp_speed.f32Value > maxControllerOut){
        tmp_speed.f32Value = maxControllerOut;
    }else if(tmp_speed.f32Value < -maxControllerOut){
        tmp_speed.f32Value = -maxControllerOut;
    }
    carSpeedOut.Transmit(&m_WriterCarSpeed, tmp_speed);
    RETURN_NOERROR;
}







