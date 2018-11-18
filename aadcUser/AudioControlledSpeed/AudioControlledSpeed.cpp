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
//#include <mutex>
#include "stdafx.h"

//#define FIXED_POINT 32

#include "AudioControlledSpeed.h"
//#include <ADTF3_helper.h>
#include "ScmCommunication.h"
#include "common_helper.h"
#include <fstream>
#include "../../../include/aadc_types/aadc_type_interface.h"


#include <iomanip>
#include <complex>
#include <cmath>
#include <vector>



using namespace std;





//*************************************************************************************************

    tBool debugToFile = true;
    tBool debugToConsole = false;

    cFilename debugFileDirectorySine = add_date_to_filename("/home/aadc/AADC/src/aadcUser/AudioControlledSpeed/debug/sine","txt").c_str();
    cFilename debugFileDirectoryDisplayBuffer = add_date_to_filename("/home/aadc/AADC/src/aadcUser/AudioControlledSpeed/debug/display_buffer","txt").c_str();
    cFilename debugFileDirectoryMagnitude = add_date_to_filename("/home/aadc/AADC/src/aadcUser/AudioControlledSpeed/debug/magnitude","txt").c_str();

    cFilename debugFileDirectoryFft = add_date_to_filename("/home/aadc/AADC/src/aadcUser/AudioControlledSpeed/debug/fft","txt").c_str();

//load compare signal for cross correlation from file
cFilename fileDirectoryRawCompareSignal = "/home/aadc/AADC/src/aadcUser/AudioControlledSpeed/rawCompareSignal_siren.txt";


ADTF_PLUGIN("Audio Controlled Speed Plugin", AudioControlledSpeed)

AudioControlledSpeed::AudioControlledSpeed() : m_chartView(nullptr), m_series(nullptr)
{
    // ------------------------------------------
    // create pointers for adtf streamtypes
    // coding convention:       p******DataType
    //object_ptr<IStreamType> pSpeedDataType;
    // ------------------------------------------
    object_ptr<IStreamType> pType = make_object_ptr<cStreamType>(stream_meta_type_audio());


    // -- Get Media Descriptions (for create_pin type) --
    //pTypeEcdActionStruct
    object_ptr<IStreamType> pTypeEcdActionStruct;
    cString structNameAction = "pTypeEcdActionStruct";
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(structNameAction.GetPtr(), pTypeEcdActionStruct, m_ecdActionStructSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_ecdActionStructSampleFactory, cString("ui8FilterId"), m_ddlEcdActionStructId.ui8FilterId));
        (adtf_ddl::access_element::find_index(m_ecdActionStructSampleFactory, cString("ui32Command"), m_ddlEcdActionStructId.ui32Command));
    }
    else
    {
        LOG_WARNING(cString::Format("No mediadescription for %s found!", structNameAction.GetPtr()));
    }

    //pTypeEcdFeedbackStruct
    object_ptr<IStreamType> pTypeEcdFeedbackStruct;
    cString structNameFeedback = "pTypeEcdFeedbackStruct";

    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(structNameFeedback.GetPtr(), pTypeEcdFeedbackStruct, m_ecdFeedbackStructSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_ecdFeedbackStructSampleFactory, cString("ui8FilterId"),        m_ddlEcdFeedbackStructId.ui8FilterId));
        (adtf_ddl::access_element::find_index(m_ecdFeedbackStructSampleFactory, cString("ui32FeedbackStatus"), m_ddlEcdFeedbackStructId.ui32FeedbackStatus));
    }
    else
    {
        LOG_WARNING(cString::Format("No mediadescription for %s found!", structNameFeedback.GetPtr()));
    }

    //pTypeAcsTSignalValueStruct
    object_ptr<IStreamType> pTypeAcsTSignalValueStruct;
    cString structNameTSignalValue = "pTypeAcsTSignalValueStruct";

    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(structNameTSignalValue.GetPtr(), pTypeAcsTSignalValueStruct, m_acsTSingalValueStructSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_acsTSingalValueStructSampleFactory, cString("ui32ArduinoTimestamp"),        m_acsTSignalValue.ui32ArduinoTimestamp));
        (adtf_ddl::access_element::find_index(m_acsTSingalValueStructSampleFactory, cString("f32Value"), m_acsTSignalValue.f32Value));
    }
    else
    {
        LOG_WARNING(cString::Format("No mediadescription for %s found!", structNameFeedback.GetPtr()));
    }

    create_pin(*this, m_WriterFeedback, "feedback_struct", pTypeEcdFeedbackStruct);
    create_pin(*this, m_ReaderAction, "action_struct", pTypeEcdActionStruct);
    create_pin(*this, m_inAudio, "audio", pType);
    create_pin(*this, m_WriterFFt, "fft", pType);
    //create_pin(*this, m_WriterSpeed, "fft", pType);
    //create_pin(*this, m_WriterCorrelation, "correlation", pType);
    create_pin(*this, m_WriterSpeed, "outputSpeed", pTypeAcsTSignalValueStruct);


    // -----------------------------------------
    // set property variables
    RegisterPropertyVariable("Disable Filter", m_propBDisableFilter);//m_propBDisableFilter
    RegisterPropertyVariable("Debug", m_propBDebugToConsole);
    RegisterPropertyVariable("Debug to file", m_propBDebugToFile);

    //Initialize global Variables
    ui32_fftBufferCounter = 0;
    ui32_processingCounter = 0;
    ui32_inc = 0;

    m_feedbackEmergencyCarDetected.ui8FilterId = F_EMERGENCY_CAR_DETECTION;
    m_feedbackEmergencyCarDetected.ui32FeedbackStatus = FB_ECD_EMERGENCY_CAR_DETECTED;



//    kiss_fft_cpx test[M_SAMPLES];
//    LoadCompareSignalFromFile(M_SAMPLES, test);
    if(m_propBDebugToConsole == tTrue)LOG_INFO("Config finished");

    //get clock object
    (_runtime->GetObject(m_pClock));

}
//--------------- END OF CONSTRUCTOR ---------------------------
//--------------------------------------------------------------


AudioControlledSpeed::~AudioControlledSpeed()
{
}

QWidget* AudioControlledSpeed::CreateView()
{
    m_series = new QLineSeries();

    QChart *chart = new QChart();
    chart->legend()->hide();
    chart->addSeries(m_series);
    chart->createDefaultAxes();


    QValueAxis* axisX = new QValueAxis;
    QValueAxis *axisY = new QValueAxis;

        chart->setTitle("Audio Input");
        axisX->setLabelFormat("%g");
        axisX->setTitleText("Samples");
        axisX->setRange(0, m_nrSamplesToDisplayAudio);

        axisY->setRange(-1.5, 1.5);
        axisY->setTitleText("Audio level");


    chart->setAxisX(axisX, m_series);
    chart->setAxisY(axisY, m_series);

    m_chartView = new QChartView(chart);
    m_chartView->setRenderHint(QPainter::Antialiasing);

    return m_chartView;
}

tVoid AudioControlledSpeed::ReleaseView()
{
    delete m_chartView;
    m_chartView = nullptr;
    m_series = nullptr;
}


tResult AudioControlledSpeed::OnTimer()
{

    tFloat32 f32_newSamplesIn[N_SAMPLES]={};

    //this will empty the Reader queue and return the last sample received.
    //If no sample was during the time between the last execution of Process the "old sample" is return.
    object_ptr<const ISample> pSampleAudio;

    tInt32 my_i = 0;
    //Sample Action Reader
    object_ptr<const ISample> pSampleAction;
    while (IS_OK(m_ReaderAction.GetNextSample(pSampleAction)))
    {
        auto oDecoder = m_ecdActionStructSampleFactory.MakeDecoderFor(*pSampleAction);
        RETURN_IF_FAILED(oDecoder.IsValid());
        TActionStruct::Data actionInput;
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlEcdActionStructId.ui32ArduinoTimestamp,
                         &actionInput.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlEcdActionStructId.ui8FilterId,
                         &actionInput.ui8FilterId));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlEcdActionStructId.bEnabled,
                         &actionInput.bEnabled));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlEcdActionStructId.bStarted,
                         &actionInput.bStarted));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlEcdActionStructId.ui32Command,
                         &actionInput.ui32Command));

        if(actionInput.ui32Command == AC_ECD_START_DETECTION)
        {
            m_propBDisableFilter = tFalse;
        }
    }


    while (IS_OK(m_inAudio.GetNextSample(pSampleAudio)))
    {
        adtf::ucom::object_ptr_shared_locked<const adtf::streaming::ISampleBuffer> pBuffer;
        pSampleAudio->Lock(pBuffer);
        //get number of audio samples in media samples
        const tInt32 availableSamples = tInt32(pBuffer->GetSize()) / m_codecPackageSizeInBytes;
        //if we have nothing displayed yet
        if (m_buffer.isEmpty())
        {
            //init buffer for audio In
            m_buffer.reserve(m_nrSamplesToDisplayAudio);
            for (tInt32 i = 0; i < m_nrSamplesToDisplayAudio; ++i)
                m_buffer.append(QPointF(i, 0));
            //init buffer for visualization (other than audio out)
            m_bufferAbs.reserve(m_nrSamplesToDisplayCrossCorr);
            for (tInt32 i = 0; i < m_nrSamplesToDisplayCrossCorr; ++i)
                m_bufferAbs.append(QPointF(i, 0));
        }

        //find start index in current buffer
        tInt32 start = 0;
        if (availableSamples < m_nrSamplesToDisplayAudio)
        {
            start = m_nrSamplesToDisplayAudio - availableSamples;
            for (tInt32 s = 0; s < start; ++s)
            {
                m_buffer[s].setY(m_buffer.at(s + availableSamples).y()); // shift y values by size availableSamples
            }
        }

        //copy data to local variabe to get non-const char pointer
        std::vector<uchar> audioData;
        audioData.resize(pBuffer->GetSize() / sizeof(uchar));
        memcpy(audioData.data(), pBuffer->GetPtr(), audioData.size());

        uchar* data = static_cast<uchar*>(audioData.data());

        //always fill m_buffer with microphone input! All functions read from this buffer
        for (tInt32 s = start; s < m_nrSamplesToDisplayAudio; ++s, data += m_codecPackageSizeInBytes)
        {
            m_buffer[s].setY(qreal(*data - 128) / qreal(128));           
//                    m_buffer[s].setY(*data);
//            if(m_propBDebugToConsole) LOG_INFO("ECD - Audio[%d]: %f", s, m_buffer[s].y());
            if(m_propBDebugToConsole) LOG_INFO("ECD - dataPointerValues[%d]: %f", s, qreal(*data - 128) / qreal(128));
        }

        for(tInt32 x = 0; x < m_nrSamplesToDisplayAudio; ++x)
        {
            if(m_buffer[x].y() < 0.0)
            {
                m_bufferAbs[x].setY(0); //cut negative values
                //m_bufferAbs[x].setY(-m_buffer[x].y()); //calculate abs() (set negative values to positive)
                //m_bufferAbs[x].setY(m_buffer[x].y()); //just copy without changing minus values
            }
            else
            {
                m_bufferAbs[x].setY(m_buffer[x].y());
            }
        }


            m_series->replace(m_bufferAbs); //update data for ui
            pBuffer->Unlock();

            //update data for ui
            m_series->replace(m_bufferAbs);
            pBuffer->Unlock();
    }

    if(m_propBDisableFilter != tTrue)
    {
        //Call function for FFT processing of the samples
        tUInt32 ui32_BufferFull = ( N_SAMPLES / (200*(SAMPLEFREQUENCY/8000)) ) + 20;//Every count the buffer gets filled with about 200 samples (for microphone sampling with 8kHz)
        if(ui32_fftBufferCounter == ui32_BufferFull) //Call FFT function only when buffer is about to be full
        {
            #ifndef BOOL_RECORD_COMPARE_SIGNAL
                ui32_fftBufferCounter = 20; //reset counter
            #endif
            //Read out last N_SAMPLES from buffer and save it to f32_newSamplesIn
            for (tInt32 u= m_nrSamplesToDisplayAudio-N_SAMPLES; u < m_nrSamplesToDisplayAudio; ++u)
            {
                tInt32 i = u-(m_nrSamplesToDisplayAudio-N_SAMPLES); //counting from 0
                f32_newSamplesIn[i] = m_bufferAbs[u].y();
            }

            RETURN_IF_FAILED(Processing(f32_newSamplesIn));
        }
        ui32_fftBufferCounter ++;
    }

    RETURN_NOERROR;
}


tResult AudioControlledSpeed::Processing(const tFloat32 f32_newSamplesIn[])
{

    tFloat32 f32_speedOut = 0.0;



//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Main Function calls

    AudioToSpeed(f32_newSamplesIn, N_SAMPLES, f32_speedOut, ACS_MIN_SPEED, ACS_MAX_SPEED, ACS_LEVEL_MIN_SPEED, ACS_LEVEL_MAX_SPEED);

    TransmitSpeed(f32_speedOut);
    LOG_INFO("ACS - f32_speedOut: %f", f32_speedOut);
    //TransmitSpeed(20.0);
//    TransmitSpeed(ACS_MIN_SPEED + (tInt32)(ui32_processingCounter/1));
//    if(ui32_processingCounter > ACS_MAX_SPEED - ACS_MIN_SPEED ) ui32_processingCounter = 0;
//    if(ui32_inc%2){ui32_processingCounter++;}
//    ui32_inc++;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


    RETURN_NOERROR;
}



void AudioControlledSpeed::AudioToSpeed(const tFloat32 f32_SamplesIn[], tInt32 i32_sampleLength, tFloat32 &f32_speedOut, tFloat32 f32_minSpeed, tFloat32 f32_maxSpeed, tFloat32 f32_levelMinSpeed, tFloat32 f32_levelMaxSpeed)
{
    tFloat32 f32_audioLevelSum = 0, f32_audioLevelMean = 0, f32_normedAudioLevelMean = 0, f32_tempSpeedOut = 0;

    //calculate mean audio level
    for(tInt32 i = 0; i < i32_sampleLength; ++i)
    {
        f32_audioLevelSum += f32_SamplesIn[i];
    }
    f32_audioLevelMean = f32_audioLevelSum / i32_sampleLength;

    f32_normedAudioLevelMean = (f32_audioLevelMean-f32_levelMinSpeed)/(f32_levelMaxSpeed-f32_levelMinSpeed);
    //LOG_INFO("ACS - f32_normedAudioLevelMean: %f", f32_normedAudioLevelMean);
    f32_tempSpeedOut = f32_minSpeed + (f32_maxSpeed - f32_minSpeed) * f32_normedAudioLevelMean;
    LOG_INFO("ACS - f32_tempSpeedOut: %f", f32_tempSpeedOut);
    //calculate output speed



    //Limit speed output
    if(f32_tempSpeedOut > ACS_MAX_SPEED)
    {
        f32_speedOut = ACS_MAX_SPEED;
    }
    if(f32_tempSpeedOut < ACS_MIN_SPEED)
    {
        f32_speedOut = 0.0;
    }
    else
    {
        f32_speedOut = f32_tempSpeedOut;
    }
}

tResult AudioControlledSpeed::Init(tInitStage eStage)
{
    RETURN_IF_FAILED(adtf::ui::cQtUIFilter::Init(eStage));

    RETURN_NOERROR;
}

tResult AudioControlledSpeed::TransmitFeedbackCarDetected()
{
    TFeedbackStruct::Data feedbackAudioControlledSpeed = m_feedbackEmergencyCarDetected;
    tTimeStamp lastTimestamp = 0;
    RETURN_IF_FAILED(m_feedback.writePin(m_WriterFeedback, (void *) &feedbackAudioControlledSpeed, lastTimestamp));
    if(m_propBDebugToConsole)LOG_INFO("Transmitted feedback emergency car detected");

    RETURN_NOERROR;
}

tResult AudioControlledSpeed::TransmitSpeed(tFloat32 speed_float)
{
    TSignalValue::Data speed;
    speed.f32Value = speed_float;
    LOG_INFO("ACS - TransmitSpeed: %f", speed_float);
    //tTimeStamp lastTimestamp = 0;
    RETURN_IF_FAILED(m_signalValue.writePin(m_WriterSpeed, (void *) &speed, m_pClock->GetStreamTime()));
    if(m_propBDebugToConsole)LOG_INFO("Transmitted speed");

    RETURN_NOERROR;
}


