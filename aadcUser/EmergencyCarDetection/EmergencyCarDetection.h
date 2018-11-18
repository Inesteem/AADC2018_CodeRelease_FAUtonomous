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

#pragma once

#include "stdafx.h"



#define CID_EMERGENCY_CAR_DETECTION  "emergency_car_detection.filter.user.aadc.cid"

//#define BOOL_RECORD_COMPARE_SIGNAL  //if not defined normal mode; if defined records and saves compare signal (stops fft and correlation computing)

#define VISUALIZE_DATA 3// 0 for AudioSignal, 1 for Fft, 2 for Magnitude of fft, 3 for CrossCorrelation

#define N_SAMPLES   2048          // samples of audio input & visualization buffer (used for buffering audio_in signal)
#define M_SAMPLES   2048      // samples of compare signal, that should be searched for..
#define M_OFFSET    0         // offset from where to read out compare signal data
#define CORR_SAMPLES 4096   //only equal to N_SAMPLES + M_SAMPLES if both are 2048 samples long

#define SAMPLEFREQUENCY 8000


#define PROP_DISABLE_FILTER tFalse
#define PROP_DEBUG          tFalse
#define PROP_DEBUG_TOFILE   tTrue

#ifndef M_PI
#define M_PI 3.1415926
#endif

//----- Globale Variablen -----
tUInt32 ui32_fftBufferCounter;
//-----------------------------

//if(m_propBDebugToConsole == tTrue)
//{
//    tBool debugToFile, debugToConsole;
//    //cFilename debugFileDirectory;
//}





/*! the main class for the EmergencyCarDetection filter. */
class EmergencyCarDetection : virtual public cQtUIFilter
{

public:
    ADTF_CLASS_ID_NAME(EmergencyCarDetection, CID_EMERGENCY_CAR_DETECTION, "Emergency Car Detection");
    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::ui::ant::IQtXSystem),
        REQUIRE_INTERFACE(adtf::services::IReferenceClock));

private:

    /*------------ STRUCTS -------------*/
    // create structs to hold information we get from SignalValues
    // coding convention: 	o_******Id

    TFeedbackStruct::Data m_feedbackEmergencyCarDetected;
    TFeedbackStruct m_feedback;

    struct tEcdActionStructId
    {
        tSize ui32ArduinoTimestamp;
        tSize ui8FilterId;
        tSize bEnabled;
        tSize bStarted;
        tSize ui32Command;
    } m_ddlEcdActionStructId;

    struct tEcdFeedbackStructId
    {
       tSize ui8FilterId;
       tSize ui32FeedbackStatus;
       tSize ui32ArduinoTimestamp;
    } m_ddlEcdFeedbackStructId;


    struct tEcdFftData
    {
        tUInt32 ui32ArduinoTimestamp;
        tFloat32 f32ValueArray[];
    } m_EcdFftData;


    /*------- SAMPLE FACTORIES ---------*/
    // a factory contains all data samples, which we can load into a tSignalValue
    // plus, it is responsible for (de)code the samples
    // coding convention:	m_******SampleFactory

    cSampleCodecFactory m_ecdActionStructSampleFactory;
    cSampleCodecFactory m_ecdFeedbackStructSampleFactory;
    cSampleCodecFactory m_ecdFftStructSampleFactroy;


    /*-------------- PINS --------------*/
    // create pins for input and output
    // coding convention:	m_Reader******	for input  signals
    //			m_Writer******	for output signals

    // input pins
     cPinReader m_inAudio;
     cPinReader m_ReaderAction;
    //cPinReader m_ReaderSpeed;

    // output pins
    //cPinWriter m_WriterFeedback;
     cPinWriter m_WriterFeedback;
     cPinWriter m_WriterFFt;
     cPinWriter m_WriterCorrelation;


    /*------------ PROPERTIES ----------*/
    // set property variables
    // coding convention:	m_prop******
    adtf::base::property_variable<tBool> m_propBDisableFilter = PROP_DISABLE_FILTER;	//Disable Filter (has to set DisableFilter = tTrue! (tFalse only for debugging))
    adtf::base::property_variable<tBool> m_propBDebugToConsole = PROP_DEBUG;         //Output debugging messages and LOG's
    adtf::base::property_variable<tBool> m_propBDebugToFile = PROP_DEBUG_TOFILE;	//Output debugging data to files


    //Properties for xml:
    tUInt32 m_propFreq[8];
    tUInt32 m_propVariation[8];
    tUInt32 m_propThreshold[8];


    /*------------ VARIABLES -----------*/
    // coding convention:	m_******

    tFloat32 f32_MagnitudeOut[N_SAMPLES];
    kiss_fft_cpx cpxCrossCorrelated[CORR_SAMPLES];
    kiss_fft_cpx cpxFftAudioSignal[CORR_SAMPLES-1];
    kiss_fft_cpx cpxFftCompareSignal[CORR_SAMPLES-1];

    /*! The chart view */
    QChartView* m_chartView;

    /*! The buffer for audio samples */
    QVector<QPointF> m_buffer;
    QVector<QPointF> m_bufferFft;

    /*! The series used in qchart */
    QLineSeries* m_series;


    /*! The nr samples to display */
    const int m_nrSamplesToDisplayAudio = N_SAMPLES; //length of DisplayBuffer
    const int m_nrSamplesToDisplayFft = N_SAMPLES; //length of DisplayBuffer
    const int m_nrSamplesToDisplayFftMagn = N_SAMPLES; //length of DisplayBuffer
    const int m_nrSamplesToDisplayCrossCorr = CORR_SAMPLES; //length of DisplayBuffer

    /*! The codec package size in bytes */
    const int m_codecPackageSizeInBytes = 4;

    /*! The reference clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

public:
    /*------------ FUNCTIONS -----------*/
    /*! Default constructor. */
    EmergencyCarDetection();

    /*! Destructor. */
    virtual ~EmergencyCarDetection();

protected: // Implement cBaseQtFilter
    QWidget * CreateView() override;
    tVoid    ReleaseView() override;
    tResult  OnTimer() override;
    tResult  Init(tInitStage eStage) override;

    //Own functions
    tResult ProcessSpeedInput();

    tResult Processing(const tFloat32 f32_newSamplesIn[]);

    void AudioIn();

    qint64 writeData(const tFloat32 *data, qint64 maxSize);// override;

    void Windowing_Hanning(const tFloat32 signalBufferIn[], tFloat32 windowOut[], tInt32 length);

    void FormatToCpx(const float in[], kiss_fft_cpx cpxOut[], tInt32 length);

    void FormatToReal(kiss_fft_cpx cpxIn[], float out[],  tInt32 length);

    void cpxZeroPadding(kiss_fft_cpx cpxIn[], kiss_fft_cpx cpxZeroPaddedOut[], tInt32 lengthIn, tInt32 lengthOut);

    void LoadCompareSignalFromFile(tInt32 readLength, tFloat32 f32RawCompareSignal[], tInt32 offset);

    void ComputeFft(const kiss_fft_cpx cpxIn[], kiss_fft_cpx cpxFftOut[], tInt32 lengthIn);

    void ComputeIfft(const kiss_fft_cpx cpxIn[], kiss_fft_cpx cpxFftOut[], tInt32 lengthIn);

    void Magnitude(const kiss_fft_cpx fftIn[], float magnitudeOut[], tInt32 length);

    void CrossCorrelation(kiss_fft_cpx fftSampledAudioIn[], kiss_fft_cpx fftCompareSignalIn[], kiss_fft_cpx correlationOut[], tInt32 lengthSampleAudioIn, tInt32 lengthCompareSignalIn);

    void Detection(tFloat32 f32_MagnitudeIn[], tInt32 i32_sampleLength);

    void checkIfThresholdExceeded(const tUInt8 ui8_frequency, tFloat32 valueExceeded[], tFloat32 magnitude[], tInt32 i32_length);

    tResult TransmitFeedbackCarDetected();
};
