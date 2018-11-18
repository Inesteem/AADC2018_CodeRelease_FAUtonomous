/*********************************************************************
    Copyright (c) 2018
    Audi Autonomous Driving Cup. All rights reserved.

    Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
    3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
    4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    $from the frist video session enhanced by Illmer and Xiangfei 21.08.2018
    Annotation: this filer is based on the filter from the video session completed by the solution of the last year team and own ideas
    **********************************************************************/

#pragma once

#define CID_OBJECT_SPECIFIER_FILTER "object_specifier.filter.user.aadc.cid"

// namespace
using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;

using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

#include <boost/thread.hpp>

// ObjectSpecifier
class ObjectSpecifier : public cTriggerFunction
{
private:
    /*------------ STRUCTS -------------*/
    // create structs to hold information we get from SignalValues
    // coding convention: 	o_******Id

    TLaserScannerData o_LaserScanner;

    TActionStruct o_ActionStruct;

    TFeedbackStruct o_FeedbackStruct;

    /*------- SAMPLE FACTORIES ---------*/
    // a factory contains all data samples, which we can load into a tSignalValue
    // plus, it is responsible for (de)code the samples
    // coding convention:	m_******SampleFactory



    /*-------------- PINS --------------*/
    // create pins for input and output
    // coding convention:	m_Reader******	for input  signals
    //			m_Writer******	for output signals

    // input pins
    cPinReader m_ReaderLaserScanner;
    cPinReader m_ReaderAction;
    cPinReader m_ReaderVideo;

    // output pins
    cPinWriter m_WriterFeedback;
    cPinWriter m_WriterDebugVideo;

    /*------------ PROPERTIES ----------*/
    // set property variables
    // coding convention:	m_prop******
    //LIDAR
    adtf::base::property_variable<tBool> m_propBDebugLidar = tBool(tFalse);
    adtf::base::property_variable<tFloat32> m_propF32ObjectThresholdDetection = 4.5;
    adtf::base::property_variable<tFloat32> m_propF32ObjectThresholdRadius = 15.0;//m_propF32ObjectThresholdRadius
    adtf::base::property_variable<tFloat32> m_propF32ROIDollXmin = 100.0;
    adtf::base::property_variable<tFloat32> m_propF32ROIDollXmax = 500.0;
    adtf::base::property_variable<tFloat32> m_propF32ROIDollYmin = 0.0;
    adtf::base::property_variable<tFloat32> m_propF32ROIDollYmax = 800.0;
    adtf::base::property_variable<tFloat32> m_propF32AdultDollWidth = 100.0;//
    adtf::base::property_variable<tFloat32> m_propF32ThresholdDollAnotherObject = 150.0;
    adtf::base::property_variable<tFloat32> m_propF32ThresholdRelevantObjectLidarProcessing = 2000.0;


    /*------------ VARIABLES -----------*/
    // coding convention:	m_******

    adtf::streaming::tStreamImageFormat m_sInputImageFormat;
    adtf::streaming::tStreamImageFormat m_sOutputImageFormat;
    // input image
    Mat m_MatInputImage;
    Mat m_MatOutputImage;

    struct LidarOneAngleObstacle
    {
        tFloat32 f32Radius;
        tFloat32 f32Angle;
        tUInt32 ui32ObstacleCounter;
        LidarOneAngleObstacle()
        {
            f32Radius = 0.0f;
            f32Angle = 0.0f;
            ui32ObstacleCounter = 0.0f;
        }
    };

    struct LidarPoint
    {
        tUInt32 ui32Counter;
        tFloat32 f32Radius;
        tFloat32 f32Angle;
        tFloat32 f32x;
        tFloat32 f32y;
        LidarPoint()
        {
            ui32Counter = 0;
            f32Radius = 0.0f;
            f32Angle = 0.0f;
            f32x = 0.0f;
            f32y = 0.0;
        }

    };

    struct LidarObstacles
    {
        tUInt32 ui32Size;
        LidarOneAngleObstacle tScanArrayEval[360];

        LidarObstacles()
        {
            ui32Size = 0;
        }

    };

    /*! The reference clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    TActionStruct::Data m_action;
    tBool m_bTransmitFeedbackFlag = tTrue;

public:
    /*------------ FUNCTIONS -----------*/
    // constructor
    ObjectSpecifier();

    // destructor
    ~ObjectSpecifier() = default;

    // Configure
    virtual tResult Configure() override;

    // Process
    virtual tResult Process(tTimeStamp tmTimeOfTrigger) override;

    //boost::lock_guard<boost::mutex> lock(criticalSection_TransmitFeedback);
    boost::mutex  criticalSection_TransmitFeedback;

    tResult ProcessActionInput();

    //ObjectSpecifier::LidarObstacles ObstacleDetectionWithLidar(TLaserScannerData::Data lasersample);
    tBool CheckForDoll(TLaserScannerData::Data lasersample);
    TLaserScannerData::Data ProcessLidar(TLaserScannerData::Data lasersample);

    tFloat32 AngleCompensation(tFloat32 angle);

    /*Transmit feedback*/
    tResult TransmitFeedbackAdultDollDetected();
    tResult TransmitFeedbackChildDollDetected();
    // ---------- FROM USER ------------*/
    // setTypeFromMat
    void setTypeFromMat(adtf::streaming::cSampleWriter& writer, const cv::Mat& outputImage, tBool keepFormat = false);

    // writeMatToPin
    void writeMatToPin(adtf::streaming::cSampleWriter& writer,
                       const cv::Mat& outputImage, tTimeStamp streamTime);

    // ChangeType
    tResult ChangeType(adtf::streaming::cDynamicSampleReader& inputPin,
        const adtf::streaming::ant::IStreamType& oType);

};
