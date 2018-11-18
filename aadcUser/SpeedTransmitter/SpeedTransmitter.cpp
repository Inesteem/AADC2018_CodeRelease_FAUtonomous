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

#include "stdafx.h"
#include "SpeedTransmitter.h"


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_SPEEDTRANSMITTER_DATA_TRIGGERED_FILTER,
    "SpeedTransmitter",
    SpeedTransmitter,
    adtf::filter::pin_trigger({"input"}));


SpeedTransmitter::SpeedTransmitter()
{
    //DO NOT FORGET TO LOAD MEDIA DESCRIPTION SERVICE IN ADTF3 AND CHOOSE aadc.description
    object_ptr<IStreamType> pTypeTemplateData;
    if(IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeTemplateData, m_SpeedSampleFactory)))
    {
        (adtf_ddl::access_element::find_index(m_SpeedSampleFactory, cString("f32Value"), o_SpeedSignalId.f32Value));
        (adtf_ddl::access_element::find_index(m_SpeedSampleFactory, cString("ui32ArduinoTimeStamp"), o_SpeedSignalId.ui32Timestamp));
    }
    else
    {
        LOG_WARNING("No mediadescription for tTemplateData found!");
    }

    //DO NOT FORGET TO LOAD MEDIA DESCRIPTION SERVICE IN ADTF3 AND CHOOSE aadc.description
    object_ptr<IStreamType> pLight;
    if(IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tBoolSignalValue", pLight, m_BoolSignalValueSampleFactory)))
    {
        (adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory, cString("tBool"), o_LightValue.bValue));
        (adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory, cString("ui32ArduinoTimeStamp"), o_LightValue.ui32Timestamp));
    }
    else
    {
        LOG_WARNING("No mediadescription for tTemplateData found!");
    }

    Register(m_oReader, "input" , pTypeTemplateData);

    Register(m_oWriter, "speedout", pTypeTemplateData);
    Register(lightwriter, "lightout", pLight);
}


//implement the Configure function to read ALL Properties
tResult SpeedTransmitter::Configure()
{
    RETURN_NOERROR;
}

tResult SpeedTransmitter::Process(tTimeStamp tmTimeOfTrigger)
{

    object_ptr<const ISample> pReadSample;

    tFloat32 inputData;
    tUInt32 inputTS;

    if (IS_OK(m_oReader.GetLastSample(pReadSample)))
    {
        auto oDecoder = m_SpeedSampleFactory.MakeDecoderFor(*pReadSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        // retrieve the values (using convenience methods that return a variant)
        RETURN_IF_FAILED(oDecoder.GetElementValue(o_SpeedSignalId.f32Value, &inputData));
        RETURN_IF_FAILED(oDecoder.GetElementValue(o_SpeedSignalId.ui32Timestamp, &inputTS));

    }

    LOG_INFO(cString::Format("Reading done, values %f / %d", inputData, inputTS));
    // Do the Processing
    tFloat32 outputData = inputData;
    tFloat32 outputTS = inputTS;

    object_ptr<ISample> pWriteSample;

    if (IS_OK(alloc_sample(pWriteSample)))
    {

        auto oCodec = m_SpeedSampleFactory.MakeCodecFor(pWriteSample);

        if(outputData > 20)
            outputData = 20;
        if(outputData < -15)
            outputData = -15;


        RETURN_IF_FAILED(oCodec.SetElementValue(o_SpeedSignalId.f32Value, outputData));
        RETURN_IF_FAILED(oCodec.SetElementValue(o_SpeedSignalId.ui32Timestamp, outputTS));

    }
    m_oWriter << pWriteSample << flush << trigger;


    object_ptr<ISample> pLigthSample;
    tBool lightOn = true;
    if (IS_OK(alloc_sample(pLigthSample)))
    {

        auto oCodec = m_BoolSignalValueSampleFactory.MakeCodecFor(pLigthSample);


        RETURN_IF_FAILED(oCodec.SetElementValue(o_LightValue.bValue, lightOn));
        RETURN_IF_FAILED(oCodec.SetElementValue(o_LightValue.ui32Timestamp, outputTS));

    }


    lightwriter << pLigthSample << flush << trigger;

    
    RETURN_NOERROR;
}
