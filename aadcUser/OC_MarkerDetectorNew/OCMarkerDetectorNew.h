/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************
* This filter receives undistorted video signal and check if there is a road sign in a frame.
* Author: Xiangfei
**********************************************************************/

#pragma once


using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace cv;

#include "stdafx.h"
#include "aruco_helpers.h"
#include "ScmCommunication.h"
#include <boost/thread.hpp>
#include "tinyxml2.h"

#define CID_OCMarkerDetectorNew_FILTER "oc_marker_detector_new.filter.user.aadc.cid"

/*! The main class for the marker detector module */
class OCMarkerDetectorNew : public cTriggerFunction
{
private:
    //Properties
    /*! Where the Camera Calib File is */
    adtf::base::property_variable<cFilename> m_calibFile = cFilename(cString("/home/yu/adtf/AADC_git/configuration_files/binhs_first_calib_test2.yml"));
    /*! Where the Detector Parameter File is */
    adtf::base::property_variable<cFilename> m_fileDetectorParameter = cFilename(cString("/home/yu/adtf/AADC_git/configuration_files/detector_params.yml"));
    /*! size of the markers*/
    adtf::base::property_variable<tFloat32> m_f32MarkerSize = 0.0985f;

    // XY: new property variables START
    /*! X Offset for Region of Interest Rectangular */
    adtf::base::property_variable<tInt32> m_propI32ROIOffsetX;
    /*! Y Offset for Region of Interest Rectangular */
    adtf::base::property_variable<tInt32> m_propI32ROIOffsetY;
    /*! Width of the Region of Interest Rectangular */
    adtf::base::property_variable<tInt32> m_propI32ROIWidth;
    adtf::base::property_variable<tInt32> m_propI32ROIWidthCurve;
    /*! Height of the Region of Interest Rectangular */
    adtf::base::property_variable<tInt32> m_propI32ROIHeight;
    /*! Max Distance of the Region of Interest Rectangular */
    adtf::base::property_variable<tFloat32> m_propF32MaxDist;
    /*! Only process every x frame */
    adtf::base::property_variable<tInt32> m_propI32DropFrame ;


    /*! z calib offset */
    adtf::base::property_variable<tFloat32> m_propF32CalibZOffset = (0.05); // (measured_distance + offset) * slope = real_distance
    /*! z calib slope */
    adtf::base::property_variable<tFloat32> m_propF32CalibZSlope = (0.469);

    // XY: new property variables END

    /*! The image pin reader */
    cPinReader m_oReader;

    /*! The image pin writer */
    cPinWriter m_oImagePinWriter;

    /*! The output pose sample factory for DDL access*/
    //adtf::mediadescription::cSampleCodecFactory m_outputPoseSampleFactory;

    /*! The vector pin writer */
    cPinWriter m_WriterRoadSignExt;

    // XY: new structs
    TRoadSignExt o_TRoadSignExt;
    TFeedbackStruct o_TFeedbackStruct;
    TActionStruct o_TActionStruct;


    // XY: new pins
    cPinReader m_ReaderAction;
    cPinWriter m_WriterFeedback;

    // XY: new variables

    TActionStruct::Data m_dataActionIn;
    TActionStruct::Data m_dataActionCheckSpecialCaseIn;
    TFeedbackStruct::Data m_dataFeedbackOut;
    TRoadSignExt::Data m_dataRoadSignExtOut;
    TRoadSignExt::Data m_dataSpecialSign;
    TFeedbackStruct::Data m_dataLastPrioritySignOut;

    //Stream Formats
    
    /*! The input format */
    adtf::streaming::tStreamImageFormat m_sInputFormat;
    /*! The output format */
    adtf::streaming::tStreamImageFormat m_sOutputFormat;

    //OpenCV Variables
    
    /*! camera matrix from calibration file */
    cv::Mat m_matIntrinsics;
    /*! distorsion coefficients from calibration file */
    cv::Mat m_matDistorsion;
    /*! indicates wheter the camara parameters are loaded or not */
    tBool m_bCamaraParamsLoaded = tFalse;

    //Aruco
    /*! the aruco detector for the markers*/
    Ptr<aruco::DetectorParameters> m_detectorParams;
    /*! marker roi bounding rectangle */
    cv::Rect m_MarkerRoi = cv::Rect();
    /*! function to check if the current roi is valid and limit it to the input image size.
    * \return true if roi is still valid, false otherwise
    */
    tBool checkRoi(void);
    /*! the dictionary for the aruco lib*/
    Ptr<aruco::Dictionary> m_Dictionary;

    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    // XY: new variables
    tBool m_bSpecialCase;
    tBool m_bCheckSpecialCase;

    tBool m_bParkingSignReceived; // save parking sign status
    int m_iFrameCount;

// init timestamps
    tInt32 m_ui32TimeStampAction;


    struct filterProperties
    {
        /*! Offset of the ROI in the Stream*/
        tInt32 ROIOffsetX;
        /*! Offset of the ROI in the Stream*/
        tInt32 ROIOffsetY;
        /*! Width of the ROI*/
        tInt32 ROIWidth;
        /*! Height of the ROI*/
        tInt32 ROIHeight;

        /*Region of Interest - Rotation*/
        tFloat32 TvecOffsetZMin;
        tFloat32 TvecOffsetYMin;
        tFloat32 TvecOffsetXMin;

        tFloat32 TvecOffsetZMax;
        tFloat32 TvecOffsetYMax;
        tFloat32 TvecOffsetXMax;

        /*Region of Interest - Translation*/
        tFloat32 RvecOffsetZMin;
        tFloat32 RvecOffsetYMin;
        tFloat32 RvecOffsetXMin;

        tFloat32 RvecOffsetZMax;
        tFloat32 RvecOffsetYMax;
        tFloat32 RvecOffsetXMax;

        tFloat32 MaxDistance;
        tInt32 DropFrameCounter;
    }
    /*! the filter properties of this class */
    m_filterProperties;




public:

    /*! Default constructor. */
    OCMarkerDetectorNew();

    /*! Destructor. */
    virtual ~OCMarkerDetectorNew() = default;

    /**
    * Overwrites the Configure
    * This is to Read Properties prepare your Trigger Function
    */
    tResult Configure();
    /**
    * Overwrites the Process
    * You need to implement the Reading and Writing of Samples within this function
    * MIND: Do Reading until the Readers queues are empty or use the IPinReader::GetLastSample()
    * This FUnction will be called if the Run() of the TriggerFunction was called.
    */
    tResult Process(tTimeStamp tmTimeOfTrigger);

    tResult ProcessActionInput(TActionStruct::Data);

    tResult ProcessVideo(object_ptr<const ISample> pReadSample);

    tResult LoadProperties();

    tResult GetSpecialSign();



    /*! function to transmit a detected road sign with extedend info
        * \param i16ID ID of the sign
        * \param f32MarkerSize size of the markers sides in meters
        * \param timeOfFrame the timestamp of the frame where the sign was detected
        * \param Tvec the translation vector
        * \param Rvec the rotation vector
        * \return standard ADTF error code
        */
    //tResult sendRoadSignStructExt(const tInt16 &i16ID, const tTimeStamp &timeOfFrame, const Vec3d &Tvec, const Vec3d &Rvec);
    //tResult sendTrafficSign(const tInt16 &i16ID, const tTimeStamp &timeOfFrame, const Vec3d &Tvec, const Vec3d &Rvec);
};
