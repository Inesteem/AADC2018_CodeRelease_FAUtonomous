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
#include "./kiss_fft130/kiss_fft.h"

#include "EmergencyCarDetection.h"
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

    cFilename debugFileDirectorySine = add_date_to_filename("/home/aadc/AADC/src/aadcUser/EmergencyCarDetection/debug/sine","txt").c_str();
    cFilename debugFileDirectoryDisplayBuffer = add_date_to_filename("/home/aadc/AADC/src/aadcUser/EmergencyCarDetection/debug/display_buffer","txt").c_str();
    cFilename debugFileDirectoryMagnitude = add_date_to_filename("/home/aadc/AADC/src/aadcUser/EmergencyCarDetection/debug/magnitude","txt").c_str();

    cFilename debugFileDirectoryFft = add_date_to_filename("/home/aadc/AADC/src/aadcUser/EmergencyCarDetection/debug/fft","txt").c_str();

//load compare signal for cross correlation from file
cFilename fileDirectoryRawCompareSignal = "/home/aadc/AADC/src/aadcUser/EmergencyCarDetection/rawCompareSignal_siren.txt";


ADTF_PLUGIN("Emergency Car Detection Plugin", EmergencyCarDetection)

EmergencyCarDetection::EmergencyCarDetection() : m_chartView(nullptr), m_series(nullptr)
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

    create_pin(*this, m_WriterFeedback, "feedback_struct", pTypeEcdFeedbackStruct);
    create_pin(*this, m_ReaderAction, "action_struct", pTypeEcdActionStruct);
    create_pin(*this, m_inAudio, "audio", pType);
    create_pin(*this, m_WriterFFt, "fft", pType);
    //create_pin(*this, m_WriterCorrelation, "correlation", pType);


    // -----------------------------------------
    // set property variables
    RegisterPropertyVariable("Disable Filter", m_propBDisableFilter);//m_propBDisableFilter
    RegisterPropertyVariable("Debug", m_propBDebugToConsole);
    RegisterPropertyVariable("Debug to file", m_propBDebugToFile);

    //Initialize global Variables
    ui32_fftBufferCounter = 0;

    m_feedbackEmergencyCarDetected.ui8FilterId = F_EMERGENCY_CAR_DETECTION;
    m_feedbackEmergencyCarDetected.ui32FeedbackStatus = FB_ECD_EMERGENCY_CAR_DETECTED;


    //Properties for detection
    m_propFreq[0] = 1115;     //set frequency (form: 436,5Hz => write 4365)
    m_propVariation[0] = 25;  //set variation of frequency (form: +-1,5Hz => write 15)
    m_propThreshold[0] = 120;  //set threshold (form: 35,2 => write 352)

    m_propFreq[1] = 1520;     //set frequency (form: 436,5Hz => write 4365)
    m_propVariation[1] = 20;  //set variation of frequency (form: +-1,5Hz => write 15)
    m_propThreshold[1] = 200;//390;  //set threshold (form: 35,2 => write 352)

    m_propFreq[2] = 7275;     //set frequency (form: 436,5Hz => write 4365)
    m_propVariation[2] = 25;  //set variation of frequency (form: +-1,5Hz => write 15)
    m_propThreshold[2] = 90;   //set threshold (form: 35,2 => write 352)

    m_propFreq[3] = 7525;     //set frequency (form: 436,5Hz => write 4365)
    m_propVariation[3] = 10;  //set variation of frequency (form: +-1,5Hz => write 15)
    m_propThreshold[3] = 50;  //set threshold (form: 35,2 => write 352)


    m_propFreq[4] = 3290;     //set frequency (form: 436,5Hz => write 4365)
    m_propVariation[4] = 15;  //set variation of frequency (form: +-1,5Hz => write 15)
    m_propThreshold[4] = 430;  //set threshold (form: 35,2 => write 352)

    m_propFreq[5] = 8235;     //set frequency (form: 436,5Hz => write 4365)
    m_propVariation[5] = 20;  //set variation of frequency (form: +-1,5Hz => write 15)
    m_propThreshold[5] = 50;  //set threshold (form: 35,2 => write 352)

    m_propFreq[6] = 8590;     //set frequency (form: 436,5Hz => write 4365)
    m_propVariation[6] = 10;  //set variation of frequency (form: +-1,5Hz => write 15)
    m_propThreshold[6] = 40;  //set threshold (form: 35,2 => write 352)

    m_propFreq[7] = 9535;     //set frequency (form: 436,5Hz => write 4365)
    m_propVariation[7] = 20;  //set variation of frequency (form: +-1,5Hz => write 15)
    m_propThreshold[7] = 190;  //set threshold (form: 35,2 => write 352)-


//    kiss_fft_cpx test[M_SAMPLES];
//    LoadCompareSignalFromFile(M_SAMPLES, test);
    if(m_propBDebugToConsole == tTrue)LOG_INFO("Config finished");
}
//--------------- END OF CONSTRUCTOR ---------------------------
//--------------------------------------------------------------


EmergencyCarDetection::~EmergencyCarDetection()
{
}

QWidget* EmergencyCarDetection::CreateView()
{
    m_series = new QLineSeries();

    QChart *chart = new QChart();
    chart->legend()->hide();
    chart->addSeries(m_series);
    chart->createDefaultAxes();


    QValueAxis* axisX = new QValueAxis;
    QValueAxis *axisY = new QValueAxis;

   //visualize depdent on setting of VISUALIZE_DATA define
    if(0 == VISUALIZE_DATA) //set axes for audio signal
    {
        chart->setTitle("Audio Input");
        axisX->setLabelFormat("%g");
        axisX->setTitleText("Samples");
        axisX->setRange(0, m_nrSamplesToDisplayAudio);

        axisY->setRange(-1, 1);
        axisY->setTitleText("Audio level");
    }
    else if(1 == VISUALIZE_DATA) //set axes for fft
    {
        chart->setTitle("Calculated FFT");
        axisX->setLabelFormat("%g");
        axisX->setTitleText("Samples");
        axisX->setRange(0, m_nrSamplesToDisplayFft);

        axisY->setRange(0, 100);
        axisY->setTitleText("fft level");
    }
    else if(2 == VISUALIZE_DATA) //set axes for fft magnitude
    {
        chart->setTitle("Calculated FFT Magnitude");
        axisX->setLabelFormat("%g");
        axisX->setTitleText("Samples");
        axisX->setRange(0, m_nrSamplesToDisplayFftMagn);

        axisY->setRange(0, 100);
        axisY->setTitleText("fft magnitude level");
    }
    else if(3 == VISUALIZE_DATA) //set axes for CrossCorrelation
    {
        chart->setTitle("Calculated CrossCorrelation");
        axisX->setLabelFormat("%g");
        axisX->setTitleText("Samples");
        axisX->setRange(0, m_nrSamplesToDisplayCrossCorr);

        axisY->setRange(0, 10000);
        axisY->setTitleText("CrossCorrelation level");
    }

    chart->setAxisX(axisX, m_series);
    chart->setAxisY(axisY, m_series);

    m_chartView = new QChartView(chart);
    m_chartView->setRenderHint(QPainter::Antialiasing);

    return m_chartView;
}

tVoid EmergencyCarDetection::ReleaseView()
{
    delete m_chartView;
    m_chartView = nullptr;
    m_series = nullptr;
}


tResult EmergencyCarDetection::OnTimer()
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
            m_bufferFft.reserve(m_nrSamplesToDisplayCrossCorr);
            for (tInt32 i = 0; i < m_nrSamplesToDisplayCrossCorr; ++i)
                m_bufferFft.append(QPointF(i, 0));
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

        // Choose which signal to visualize:
        if(0 == VISUALIZE_DATA) // send audio signal to visualization
        {
            m_series->replace(m_buffer); //update data for ui
            pBuffer->Unlock();
        }
        else if(1 == VISUALIZE_DATA) // send fft to visualization
        {
            for (tInt32 s = 0; s < m_nrSamplesToDisplayFft; ++s)
            {
                m_bufferFft[s].setY(cpxFftAudioSignal[s].r);
                if(m_propBDebugToConsole) LOG_INFO("ECD - FFT[%d]: %f", s, m_bufferFft[s].y());
            }
            m_series->replace(m_bufferFft); //update data for ui
            pBuffer->Unlock();
        }
        else if(2 == VISUALIZE_DATA) // send fft magnitude to visualization
        {
            for (tInt32 s = 0; s < m_nrSamplesToDisplayFftMagn; ++s)
            {
                m_bufferFft[s].setY(f32_MagnitudeOut[s]);
                if(m_propBDebugToConsole) LOG_INFO("ECD - FFT[%d]: %f", s, m_bufferFft[s].y());
            }
            m_series->replace(m_bufferFft); //update data for ui
            pBuffer->Unlock();
        }
        else if(3 == VISUALIZE_DATA) // send CrossCorrelation to visualization
        {
            for (tInt32 s = 0; s < m_nrSamplesToDisplayCrossCorr; ++s)
            {
                m_bufferFft[s].setY(cpxCrossCorrelated[s].r);
                if(m_propBDebugToConsole) LOG_INFO("ECD - FFT[%d]: %f", s, m_bufferFft[s].y());
            }
            //update data for ui
            m_series->replace(m_bufferFft);
            pBuffer->Unlock();
        }

        if(m_propBDebugToFile == tTrue)
        {
            fstream file_fft;
            file_fft.open(debugFileDirectoryDisplayBuffer, ios::out | ios::app);
            file_fft << "EmergencyCarDetection: m_buffer, samples: " << availableSamples << ", GetNextSample-Durchlauf (Reset by function call):"<< my_i << "\n \n";

            for (tInt32 i = 0; i < m_nrSamplesToDisplayAudio; ++i, data += m_codecPackageSizeInBytes)
            {
                file_fft << i << "\t x:" << m_buffer[i].x() << "\t y:" << m_buffer[i].y() << "\n";
            }

            file_fft << "\n \n" << "--------------------------------------" << "\n\n\n";
            file_fft.close();
        my_i = my_i + 1;
        }
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
                f32_newSamplesIn[i] = m_buffer[u].y();
            }

            RETURN_IF_FAILED(Processing(f32_newSamplesIn));
        }
        ui32_fftBufferCounter ++;
    }

    RETURN_NOERROR;
}


tResult EmergencyCarDetection::Processing(const tFloat32 f32_newSamplesIn[])
{
    tFloat32 f32_windowedAudioSignal[N_SAMPLES], f32_windowedCompareSignal[N_SAMPLES], f32RawCompareSignal[N_SAMPLES]; // f32debugSignal[N_SAMPLES];
    tInt32 ZeroPaddedLength = N_SAMPLES + M_SAMPLES - 1; //doubled length for ZeroPadding
//    tInt32 ZeroPaddedLengthCompareSignal = M_SAMPLES * 2;
    kiss_fft_cpx cpxWindowAudioSignal[N_SAMPLES], cpxWindowCompareSignal[N_SAMPLES];
    kiss_fft_cpx cpxZeroPaddedAudioSignal[ZeroPaddedLength], cpxZeroPaddedCompareSignal[ZeroPaddedLength]={};

#ifdef BOOL_RECORD_COMPARE_SIGNAL
  {
      LOG_INFO(" ");
      LOG_INFO("!!! ATTENTION: EmergencyCarDetection: Recording Compare Signal !!!");
      LOG_INFO("!!! ATTENTION: For detection of emergency cars do the the following: !!!");
      LOG_INFO("!!! ATTENTION: Comment the #define BOOL_RECORD_COMPARE_SIGNAL or ask Nico !!!");
      LOG_INFO(" ");
      fstream file_fft;
      file_fft.open(fileDirectoryRawCompareSignal, ios::out | ios::app);

      tInt32 i;
      for (i = 0; i < N_SAMPLES; i++)
      {
          file_fft <<  f32_newSamplesIn[i]  << "\n";
      }
      file_fft.close();
  }
#endif


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Main Function calls
//    LoadCompareSignalFromFile(N_SAMPLES, f32debugSignal, 0); //only debugging
//    Windowing_Hanning(f32debugSignal, f32_windowedAudioSignal, N_SAMPLES); //only debugging
    Windowing_Hanning(f32_newSamplesIn, f32_windowedAudioSignal, N_SAMPLES);
    FormatToCpx(f32_windowedAudioSignal, cpxWindowAudioSignal, N_SAMPLES);
    cpxZeroPadding(cpxWindowAudioSignal, cpxZeroPaddedAudioSignal,N_SAMPLES, ZeroPaddedLength);
    ComputeFft(cpxZeroPaddedAudioSignal, cpxFftAudioSignal, ZeroPaddedLength);
    Magnitude(cpxFftAudioSignal, f32_MagnitudeOut, N_SAMPLES);

#ifndef BOOL_RECORD_COMPARE_SIGNAL
    LoadCompareSignalFromFile(M_SAMPLES, f32RawCompareSignal, M_OFFSET);
    Windowing_Hanning(f32RawCompareSignal, f32_windowedCompareSignal, M_SAMPLES);
    FormatToCpx(f32_windowedCompareSignal, cpxWindowCompareSignal, M_SAMPLES);
    cpxZeroPadding(cpxWindowCompareSignal, cpxZeroPaddedCompareSignal, M_SAMPLES, ZeroPaddedLength);
    ComputeFft(cpxZeroPaddedCompareSignal, cpxFftCompareSignal, ZeroPaddedLength);

    CrossCorrelation(cpxFftAudioSignal, cpxFftCompareSignal, cpxCrossCorrelated, ZeroPaddedLength, ZeroPaddedLength);
#endif
    Detection(f32_MagnitudeOut, N_SAMPLES);

//    FormatToReal(f32CorrelationOut)
//    writeData(f32_MagnitudeOut, (qint64)N_SAMPLES);
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


    RETURN_NOERROR;
}


void EmergencyCarDetection::Windowing_Hanning(const tFloat32 signalBufferIn[], tFloat32 dataOut[], tInt32 length)
{
    for (tInt32 i = 0; i < length; i++) {
        double multiplier = 0.5 * (1 - cos(2*M_PI*i/(length-1)));
        dataOut[i] = multiplier * signalBufferIn[i];
    }
}


void EmergencyCarDetection::FormatToCpx(const float in[], kiss_fft_cpx cpxOut[], tInt32 length)
{
    for (tInt32 i = 0; i < length; i++)
    {
        cpxOut[i].r = in[i];
        cpxOut[i].i = 0;
    }
}

void EmergencyCarDetection::FormatToReal(kiss_fft_cpx cpxIn[], float out[],  tInt32 length)
{
    for (tInt32 i = 0; i < length; i++)
    {
        out[i] = cpxIn[i].r;
    }
}


void EmergencyCarDetection::cpxZeroPadding(kiss_fft_cpx cpxIn[], kiss_fft_cpx cpxZeroPaddedOut[], tInt32 lengthIn, tInt32 lengthOut)
{
    for (tInt32 i = 0; i < lengthIn ; i++)
    {
        cpxZeroPaddedOut[i].r = cpxIn[i].r;
        cpxZeroPaddedOut[i].i = cpxIn[i].i;
    }
    for (tInt32 i = lengthIn; i < lengthOut; i++) //Zero Padding
    {
        cpxZeroPaddedOut[i].r = 0;
        cpxZeroPaddedOut[i].i = 0;
    }
}


void EmergencyCarDetection::LoadCompareSignalFromFile(tInt32 readLength, tFloat32 f32RawCompareSignal[], tInt32 offset) //Readout compare signal values from file
{
    fstream file_compareSignal;
    string line ={};

    file_compareSignal.open(fileDirectoryRawCompareSignal, ios::in | ios::app);

    float tmp[readLength]={};
    tInt32 i = 0, k = 0;

    while (getline(file_compareSignal, line) && i<readLength)
    {
        float value;
        k = 0;
        stringstream ss(line);

        while (ss >> value)
        {
            tmp[i] = value;
            ++k;
        }
        ++i;
    }
    for(tInt32 u=0; u < readLength ; ++u)
    {
//        if(m_propBDebugToConsole == tTrue){LOG_INFO("ReadFromFile: %f, %f", tmp[u][0], tmp[u][1]);}
        f32RawCompareSignal[u] = tmp[u+offset];
    }
//    LOG_INFO("ECD: Read compare signal from file!");
    file_compareSignal.close();
}


void EmergencyCarDetection::ComputeFft(const kiss_fft_cpx cpxIn[], kiss_fft_cpx cpxFftOut[], tInt32 lengthIn)
{
     kiss_fft_cfg cfg;
     kiss_fft_cpx cpxFftTemp[lengthIn]; //does contain all calculated elements (second half is unrelevant)

  if ((cfg = kiss_fft_alloc(lengthIn, 0/*is_inverse_fft*/, NULL, NULL)) != NULL)
  {

    kiss_fft(cfg, cpxIn, cpxFftTemp);
    free(cfg);

    for(tInt32 u=0; u < lengthIn; ++u) ////cut away unrelevant second half of elements -> u < lengthIn / 2
    {
        cpxFftOut[u].r = cpxFftTemp[u].r;
        cpxFftOut[u].i = cpxFftTemp[u].i;
    }
    if(m_propBDebugToConsole == tTrue)
    {
            size_t i;
          for (i = 0; i < (size_t)lengthIn; i++)
          printf(" in[%2zu] = %+f , %+f    "
                     "out[%2zu] = %+f , %+f\n",
                     i, cpxIn[i].r, cpxIn[i].i,
                     i, cpxFftOut[i].r, cpxFftOut[i].i);
    }

  }
  else
  {
    printf("not enough memory?\n");
    exit(-1);
  }
if(m_propBDebugToFile)
{
    fstream file_fft;
    file_fft.open(debugFileDirectoryFft, ios::out | ios::app);
    file_fft << "Compute FFT "<< "\n \n";

    tInt32 i;
    for (i = 0; i < lengthIn/2; i++)
    {
        file_fft << i << "\t\t\t\t\t\t" << cpxIn[i].r << "\t\t\t\t\t\t" << cpxIn[i].i << "\t\t\t\t\t\t" << cpxFftOut[i].r << "\t\t\t\t\t\t" << cpxFftOut[i].i << "\n";
    }
    file_fft << "\n \n" << "--------------------------------------" << "\n\n\n";
    file_fft.close();
}
}



void EmergencyCarDetection::ComputeIfft(const kiss_fft_cpx cpxFftIn[], kiss_fft_cpx cpxOut[], tInt32 lengthIn)
{
    kiss_fft_cfg cfg2;

    if ((cfg2 = kiss_fft_alloc(lengthIn, 1/*is_inverse_fft*/, NULL, NULL)) != NULL)
    {
        kiss_fft(cfg2, cpxFftIn, cpxOut);
        free(cfg2);
    }
    else
    {
        printf("not enough memory?\n");
        exit(-1);
    }
}


void EmergencyCarDetection::Magnitude(const kiss_fft_cpx fftIn[], float magnitudeOut[], tInt32 length)

{
    for (int i = 0; i < length; i++)
    {
        magnitudeOut[i] = sqrt( (fftIn[i].r * fftIn[i].r)  +  (fftIn[i].i * fftIn[i].i) );
    }
    if(m_propBDebugToFile == tTrue)
    {
        {
            fstream file_mag;

            file_mag.open(debugFileDirectoryMagnitude, ios::out | ios::app);
            file_mag << "Magnitude - FFT Frequency Resolution: " << ((float)SAMPLEFREQUENCY/(float)length)<< "\n \n";

            int i;
            for (i = 0; i < length; i++)
            {
                file_mag << (float)i * (1024.0/(float)N_SAMPLES) << " \t" << magnitudeOut[i] << "\n";
        //        (float)SAMPLEFREQUENCY/(float)N_SAMPLES)
            }
            //file_mag << "\n \n" << "--------------------------------------" << "\n\n\n";

            file_mag.close();
        //        file_mag_csv.close();
        }

    }
}


void EmergencyCarDetection::CrossCorrelation(kiss_fft_cpx fftSampledAudioIn[],  kiss_fft_cpx fftCompareSignalIn[], kiss_fft_cpx correlationOut[], tInt32 lengthSampleAudioIn, tInt32 lengthCompareSignalIn)
{

// Implement   corr(a, b) = ifft(fft(a_and_zeros) * fft(b_and_zeros[reversed])) => O(n log n)
//        or   corr(a, b) = ifft(fft(a_and_zeros) * conj(fft(b_and_zeros))) =>  O(n) -> is faster
    correlationOut[CORR_SAMPLES]={};
    kiss_fft_cpx conjugatedFftCompareSignal[lengthCompareSignalIn]={};
    kiss_fft_cpx resElementMult[lengthCompareSignalIn]={};
//    kiss_fft_cpx tmpCompSignal[(lengthSampleAudioIn + lengthCompareSignalIn)], tmpAudioIn[(lengthSampleAudioIn + lengthCompareSignalIn)];
    //get the complex conjugate -> conj(b_and_zeros)
    for(tInt16 i= 0; i < lengthCompareSignalIn ; ++i)
    {
        conjugatedFftCompareSignal[i].i =  -fftCompareSignalIn[i].i;
    }
    //cpxZeroPadding(conjugatedFftCompareSignal, tmpCompSignal, lengthCompareSignalIn, (lengthSampleAudioIn + lengthCompareSignalIn));
    //cpxZeroPadding(fftSampledAudioIn, tmpAudioIn, lengthSampleAudioIn, (lengthSampleAudioIn + lengthCompareSignalIn));
    //elementwise multiplication of a_and_zeros and conj(b_and_zeros)
//    for(tInt16 u=0; u < (lengthSampleAudioIn + lengthCompareSignalIn)-1; ++u)
    for(tInt16 u=0; u < lengthSampleAudioIn; ++u)
    {
        resElementMult[u].r =  (fftSampledAudioIn[u].r * conjugatedFftCompareSignal[u].r) - (fftSampledAudioIn[u].i * conjugatedFftCompareSignal[u].i);
        resElementMult[u].i =  (fftSampledAudioIn[u].r * conjugatedFftCompareSignal[u].i) + (fftSampledAudioIn[u].i * conjugatedFftCompareSignal[u].r);
    }
    for(tInt16 u=N_SAMPLES; u < CORR_SAMPLES; ++u) //special zero padding to fill to CORR_SAMPLES (which should be a power of 2)
    {
        resElementMult[u].r =  0;
        resElementMult[u].i =  0;
    }

    //compute cross correlation
    ComputeIfft(resElementMult,correlationOut,CORR_SAMPLES);

    //build absoulte value of real-part (only positive)
    for(tInt16 u=0; u < CORR_SAMPLES; ++u)
    {
        if(correlationOut[u].r < 0) correlationOut[u].r = -correlationOut[u].r;
    }


    if(m_propBDebugToFile == tTrue)
    {
        cFilename debugFileDirectoryCorrelation = add_date_to_filename("/home/aadc/AADC/src/aadcUser/EmergencyCarDetection/debug/correlation","txt").c_str();
        fstream file_correlation;
        file_correlation.open(debugFileDirectoryCorrelation, ios::out | ios::app);
        file_correlation << "Compute Correlation "<< "\n \n";

        tInt32 i;
        for (i = 0; i < CORR_SAMPLES; i++)
        {
            file_correlation << i << "\t\t" << correlationOut[i].r << "\t\t" << correlationOut[i].i << "\n";
        }
        file_correlation << "\n \n" << "--------------------------------------" << "\n\n\n";
        file_correlation.close();
    }
}

void EmergencyCarDetection::Detection(tFloat32 f32_MagnitudeIn[], tInt32 i32_sampleLength)
{
    tFloat32 ValuesExceeded[8] = {};
    tFloat32 f32_sumExceededH = 0.0, f32_sumExceededL = 0.0;
    tUInt32  countThresholdExceededSirenH = 0, countThresholdExceededSirenL = 0;

    //check if thresholds are exceeded -> ValuesExceeded > 0.0
    checkIfThresholdExceeded(0, ValuesExceeded, f32_MagnitudeIn, i32_sampleLength);
    checkIfThresholdExceeded(1, ValuesExceeded, f32_MagnitudeIn, i32_sampleLength);
    checkIfThresholdExceeded(2, ValuesExceeded, f32_MagnitudeIn, i32_sampleLength);
    checkIfThresholdExceeded(3, ValuesExceeded, f32_MagnitudeIn, i32_sampleLength);

    checkIfThresholdExceeded(4, ValuesExceeded, f32_MagnitudeIn, i32_sampleLength);
    checkIfThresholdExceeded(5, ValuesExceeded, f32_MagnitudeIn, i32_sampleLength);
    checkIfThresholdExceeded(6, ValuesExceeded, f32_MagnitudeIn, i32_sampleLength);
    checkIfThresholdExceeded(7, ValuesExceeded, f32_MagnitudeIn, i32_sampleLength);

    //debug strings
    string debug_sL="", debug_sH="";

    //count how many of the 4 thresholds of SirenLow and SirenHigh are Exceeded
    for(int i = 0; i < 4; ++i)
    {
        if(ValuesExceeded[i] > 0.0)
        {
            countThresholdExceededSirenL++;
            f32_sumExceededL += ValuesExceeded[i];

            debug_sL += to_string(i);
            debug_sL +=",";
        }
    }
    for(int u = 4 ; u < 8; ++u)
    {
        if(ValuesExceeded[u] > 0.0)
        {
            countThresholdExceededSirenH++;
            f32_sumExceededH += ValuesExceeded[u];

            debug_sH += to_string(u);
            debug_sH +=",";
        }
    }


    //check if threshold exceeded count > 2 => detected (use information that only SirenH or SirenL can be detected)
    if( ((countThresholdExceededSirenL > 1 ) && (ValuesExceeded[0] || ValuesExceeded[1])) ) //|| (countThresholdExceededSirenH > 1) )
    {
        //Emergency car IS DETECTED!!!
        LOG_INFO("ECD - Emergency Car DETECTED!!! :) (countExcL:%d[%s], countExcH:%d[%s], sumExcL:%f, sumExcH:%f)", countThresholdExceededSirenL, debug_sL.c_str(), countThresholdExceededSirenH, debug_sH.c_str(), f32_sumExceededL, f32_sumExceededH);
        //TODO: Do stuff / transmit Feedback "DETECTED" to SCM!!!!

        //    If emergency car is detected -> Send Feedback detected
        //    TransmitFeedbackCarDetected();
    }
    else
    {
        //no emergency car detected
        LOG_INFO("ECD - NO emergency car detected :( (countExcL:%d[%s], countExcH:%d[%s], sumExcL:%f, sumExcH:%f)", countThresholdExceededSirenL, debug_sL.c_str(), countThresholdExceededSirenH, debug_sH.c_str(), f32_sumExceededL, f32_sumExceededH);
    }


}

void EmergencyCarDetection::checkIfThresholdExceeded(const tUInt8 ui8_frequency, tFloat32 valueExceeded[], tFloat32 magnitude[], tInt32 i32_length)
{
    tInt32 i32_freqMin, i32_freqMax;
    tFloat32 f32_threshold, maxValue_temp;
    valueExceeded[ui8_frequency] = 0.0;

    //calc maximum frequency
        //if calculated frequency has decimals, increase maxFreqency to the next higher one
    if( (((m_propFreq[ui8_frequency] + m_propVariation[ui8_frequency])*i32_length)%(1024*10)) > 0)
    {
        i32_freqMax = ((m_propFreq[ui8_frequency] + m_propVariation[ui8_frequency])*i32_length)/(1024*10) + 1;
    }
    else
    {
        i32_freqMax = (tInt32)(((m_propFreq[ui8_frequency] + m_propVariation[ui8_frequency])*i32_length)/(1024*10));
    }
    //calc minimum frequency
    i32_freqMin = (tInt32)((m_propFreq[ui8_frequency] - m_propVariation[ui8_frequency])*i32_length)/(1024*10);
    //calc threshold
    f32_threshold = (tFloat32)m_propThreshold[ui8_frequency]/10.0;

    maxValue_temp = 0.0;
    for(tInt32 v = i32_freqMin; v <= i32_freqMax; ++v)
    {
        if(magnitude[v] > maxValue_temp)
            maxValue_temp = magnitude[v];
    }
    //check if threshold exceeded
    if(maxValue_temp > f32_threshold)
        valueExceeded[ui8_frequency] = maxValue_temp - f32_threshold;
}

tResult EmergencyCarDetection::Init(tInitStage eStage)
{
    RETURN_IF_FAILED(adtf::ui::cQtUIFilter::Init(eStage));

    RETURN_NOERROR;
}

tResult EmergencyCarDetection::TransmitFeedbackCarDetected()
{
    TFeedbackStruct::Data feedbackEmergencyCarDetection = m_feedbackEmergencyCarDetected;
    tTimeStamp lastTimestamp = 0;
    RETURN_IF_FAILED(m_feedback.writePin(m_WriterFeedback, (void *) &feedbackEmergencyCarDetection, lastTimestamp));
    if(m_propBDebugToConsole)LOG_INFO("Transmitted feedback emergency car detected");

    RETURN_NOERROR;
}

qint64 EmergencyCarDetection::writeData(const tFloat32 *data, qint64 maxSize)
{
    object_ptr<ISample> pSample;

    if (IS_OK(alloc_sample(pSample)))
    {
        object_ptr_locked<ISampleBuffer> pBuffer;
        pSample->SetTime(m_pClock->GetStreamTime());
        if (IS_OK(pSample->WriteLock(pBuffer, maxSize)))
        {
            pBuffer->Write(adtf_memory_buffer<const tVoid>(data, maxSize));
            //Not needed as in demo_virtual_clock?
            //pBuffer->Unlock();
        }
    }

    m_WriterFFt << pSample << trigger;
    return maxSize;
}
