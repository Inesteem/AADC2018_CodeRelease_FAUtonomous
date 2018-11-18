/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. Team FAUtonomous. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/

#include <deque>

#pragma once
#define CID_LINE_FILTER_ADAPTER_FILTER "line_filter_adapter.filter.user.aadc.cid"

// namespaces
using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

class cLineFilter;

// LineFilterAdapter
class LineFilterAdapter : public cTriggerFunction
{
private:

    // input pins
    cPinReader m_ReaderVideo;
    cPinReader m_ReaderPose;
    cPinReader m_ReaderPitch;

    // output pins
    cPinWriter m_WriterVideo;
    cPinWriter m_WriterDebugVideo;

    // The clock
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    TPoseStruct  m_poseStruct;
    TSignalValue m_SignalValuePitchId;

    // image formats for video pins
    adtf::streaming::tStreamImageFormat m_sInputImageFormat;
    adtf::streaming::tStreamImageFormat m_sOutputImageFormat;

    // image error offset in meter of the transformed output picture.
    tFloat32 m_f32EstimationErrorOffset;

    tTimeStamp m_tsTimeOfPrevFrame;

    tBool m_bFirstFrame = tTrue;

    // currently used line filter
    cLineFilter * lineFilter;

    // -------------------------
    // PROPERTIES
    adtf::base::property_variable<tBool>    m_propEnableMedianFiltering      = tBool (   tTrue);
    adtf::base::property_variable<tBool>    m_propEnableAdaptiveThresholding = tBool (  tFalse);
    adtf::base::property_variable<tFloat32> m_propNearPlane                  = tFloat(  0.600f);
    adtf::base::property_variable<tFloat32> m_propCameraOffsetX              = tFloat(  0.285f);
    adtf::base::property_variable<tFloat32> m_propCameraOffsetY              = tFloat(  0.015f);
    adtf::base::property_variable<tFloat32> m_propCameraOffsetZ              = tFloat(  0.220f);
    adtf::base::property_variable<tFloat32> m_propFieldOfViewHorizontal      = tFloat( 59.000f);
    adtf::base::property_variable<tFloat32> m_propFieldOfViewVertical        = tFloat( 45.000f);
    adtf::base::property_variable<tFloat32> m_propThresholdConstant          = tFloat(-20.000f);
    adtf::base::property_variable<tUInt32>  m_propThresholdWidth             = tFloat( 21.000f);
    adtf::base::property_variable<tFloat32> m_propPitch                      = tFloat(  0.000f);
    adtf::base::property_variable<tFloat32> m_propRoll                       = tFloat(  0.000f);
    adtf::base::property_variable<tFloat32> m_propYaw                        = tFloat(  0.000f);
    adtf::base::property_variable<tUInt32>  m_propROIX                       = tUInt32(  175);
    adtf::base::property_variable<tUInt32>  m_propROIY                       = tUInt32(  525);
    adtf::base::property_variable<tUInt32>  m_propROIWidth                   = tUInt32(  575);
    adtf::base::property_variable<tUInt32>  m_propROIHeight                  = tUInt32(  120);
    adtf::base::property_variable<tUInt32>  m_propNewThresholdConstand       = tUInt32(  180);

    // -------------------------
    // STRUCTS
    struct CarPose
    {
        cv::Point2f position;
        float yaw;
        float pitch;
        float roll;
        tTimeStamp adtfTime;
        bool estimated;

        CarPose ()
        {
            this->yaw = 0;
            this->pitch = 0;
            this->roll = 0;
            this->adtfTime = 0;
            this->estimated = false;
        }

        CarPose (cv::Point2f pose, float yaw, float pitch, float roll, tTimeStamp adtfTime, bool estimated)
        {
            this->position = pose;
            this->yaw = yaw;
            this->pitch = pitch;
            this->roll = roll;
            this->adtfTime = adtfTime;
            this->estimated = estimated;
        }
    };

    std::deque<CarPose> carPose;
    
    // output buffer
    Mat m_MatOut;

    // input buffer
    Mat m_MatIn;



public:
    /*------------ FUNCTIONS -----------*/
    /*------------ FROM AUDI -----------*/
    // constructor
    LineFilterAdapter();

    // destructor
    ~LineFilterAdapter() = default;

    // Configure
    virtual tResult Configure() override;

    // Process
    virtual tResult Process(tTimeStamp tmTimeOfTrigger) override;

    // ---------- FROM USER ------------*/
    void setTypeFromMat(adtf::streaming::cSampleWriter& writer, const cv::Mat& outputImage, tBool keepFormat = false);

    // writeMatToPin
	void writeMatToPin(adtf::streaming::cSampleWriter& writer,
					   const cv::Mat& outputImage, tTimeStamp streamTime);
    
    // ChangeType
    tResult ChangeType(adtf::streaming::cDynamicSampleReader& inputPin,
        const adtf::streaming::ant::IStreamType& oType);

    // WriteAsFloat
    tResult WriteAsFloat(cv::Mat& image, tUInt32 bytePosition, tFloat32 value);

}; // LineFilterAdapter
