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

#pragma once
#include "stdafx.h"
#include <boost/thread.hpp>
#define CID_OBSTACLE_DETECTION_FILTER "obstacle_detection.filter.user.aadc.cid"
//#define OID_OBSTACLE_DETECTION_FILTER "adtf.aadc.OBSTACLE_DETECTION"


//Debugging:
//#define DEBUG_MODE_OUTPUT_VIDEO

class ObstacleDetection : public cTriggerFunction
{
private:

    /*------------ Index STRUCTS -------------*/
    // see in aadc_dir/include/aadc_{user_}structs.h
    // coding convention: 	o_******Id
    
    TFeedbackStruct	o_feedbackStruct;
    TActionStruct	o_actionStruct;
    TSignalValue        o_signalValue;
    TLaserScannerData   o_laserScannerData;
//  TVideoStream        o_videoStream;

protected:

    /*-------------- PINS --------------*/
    // create pins for input and output
    // coding convention:	m_Reader******	for input  signals
    //			m_Writer******	for output signals

    // input pins
    cPinReader m_ReaderAction;
    cPinReader m_ReaderTargetSpeed;
    cPinReader m_ReaderSteeringAngle;
    cPinReader m_ReaderLaserScannerData;
    //TODO: cVideoPin or cPinReader for m_ReaderVideoPin?
    cPinReader m_ReaderVideoPin;


    // output pins
    cPinWriter m_WriterFeedback;
    cPinWriter m_WriterTargetSpeed;
//TODO: cVideoPin?
//    cVideoPin m_WriterGCLOutput;
//    cVideoPin m_WriterVideoValidPin;
//    cVideoPin m_WriterDebugOptACCPin;

//TODO: cVideoPin or cPinReader?
//    #ifdef DEBUG_MODE_OUTPUT_VIDEO
//    // output debug depth video
//        cVideoPin m_WriterVideoPin;
//    #endif

private:

    /*********************/
    /***** Variables *****/
    /*********************/

    //Variables that store the read Values
    TSignalValue::Data m_curSpeedControllerInput;
    TLaserScannerData::Data m_curScannerInput;
//  TVideoStream::Data videoStreamInput;
//TODO: Needed????????????????????????????????????????????????????????????????????
    /* location of setting-file where pitch angle and offset are stored,
     * previously created by 'pitch-estimation' */
    cFilename m_filePitch;
    /* flag indicating whether pitch is to be read from file */
    tBool m_bUsePitchFromFile;
    /* variable to save pitch correction factor, accounting for offset/deviation
     *  between rgb-cam (used for calibration) and depth-cam*/
    tFloat32 m_f32PitchCorrection;
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    /* Offset parameter for camera position */
    tFloat32 m_f32CameraOffsetX;
    tFloat32 m_f32CameraOffsetY;
    tFloat32 m_f32CameraOffsetZ;

    tFloat32 m_f32Camera_DepthToRgbOffest;

    /* properties for manipulated view */
    tFloat32 m_f32CarWidth;
    tFloat32 m_f32DistanceThresholdMax;
    tFloat32 m_f32DistanceThresholdMin;
    tFloat32 m_f32HeightThresholdMax;
    tFloat32 m_f32HeightThresholdMin;
    tFloat32 m_f32HeightThresholdValidmask;

    #ifdef DEBUG_TO_FILE_POINTCLOUD
    fstream m_file_pointcloud;
    #endif

    #ifdef DEBUG_TO_FILE_POINTCLOUD_TRANSFORMED
    fstream m_file_pointcloud_trans;
    #endif


    /* global variable to temporarily save the previously received actionSub-structure;
     *  use get and set methods to access data */
    TActionStruct::Data m_curActionStruct;

    /* variable for temporarily saving current steering angle */
    TSignalValue::Data m_curSteeringAngle;

    /* variable for temporarily saving current m_f32TargetSpeedLimit */
    tFloat32 m_f32CurTargetSpeedLimit;


    /* status of optical ACC (has to be disabled for parking) */
    enum operationalStatus{
        OPT_ACC_ENABLED = 1,
        OPT_ACC_DISABLED = 2
    };

    operationalStatus OACC_operational_status;

    /* variables and parameter for OpticACC */
    tFloat32 OACC_wheelbase;
    tFloat32 OACC_carWidth;
    tFloat32 OACC_frontAxisToFrontBumper;
    tFloat32 OACC_obliqueAngleFront;
    tFloat32 OACC_circSegmentDistance;
    tFloat32 OACC_ySearchTolerance;
    tFloat32 OACC_obstacleHeightThreshold_min;
    tFloat32 OACC_obstacleHeightThreshold_max;

    tUInt32 OACC_GridCellOccupiedCounterThreshold;
    tUInt32 OACC_GridCellResolutionIndicator;
    tUInt32 OACC_GridCellResolution;

    tBool b_debug_occupancyGrid;

    /* name of xml-file for parking mode of ACC */
    cFilename m_xml_configFileOpticalACC;

    /*! holds the yValues for the supporting points*/
    vector<tFloat32> o_f32Acc_xml_xValues;
    /*! holds the xValues for the supporting points*/
    vector<tFloat32> o_f32Acc_xml_yValues;

    /*! enable/disable warning at reached borders of the xml-file */
    tBool m_xml_BorderWarningModeEnabled = tFalse;
    tBool m_xml_PrintInitialtable = tFalse;

    tFloat32 m_f32SinDynRange;
    tFloat32 m_f32CosDynRange;
//    TODO: InDoing >>>
    /* flag indicating whether input video format has to be read */
    tBool m_bFirstFrame;


    /* flags used for logging and debug mode */
    tBool m_bPointcloud_to_file;
    tBool m_bTransformed_pointcloud_to_file;
    tBool m_bTransformed_logging_completed;
    tBool m_bTransformed_pointcloud_to_file_one;
    tBool m_bTransformed_logging_completed_one;
    tBool m_bDebugModeEnabled = tFalse;
    tBool m_bExtendeddebugModeEnabled = tFalse;
    tUInt32 m_ui32DebugLogEventToggle;

    #ifdef DEBUG_MODE_OUTPUT_VIDEO
    tUInt32 m_ui32DebugType;
    #endif
    tBool m_bShowGCLDebug = tFalse;
    tBool m_bShowGCLDebug_extendedLog = tFalse;
    tBool m_bDebugACCObstaclepixelcount ;
    tBool m_bShowOaccBinaryImageDebug = tFalse;

    /* variables to store cosinus and sinus values of pitch */
    tFloat32 m_f32CosPitch;
    tFloat32 m_f32SinPitch;

    tBitmapFormat m_inputFormat;

    tBool m_bRunningState;
//TODO: Needed?????????????????????????????????????????????????????????
    struct ProcessedCloudData {
        tFloat32 m_f32TargetSpeedLimit;
        cv::Mat m_cvValidPixelDepthImage;
        ProcessedCloudData(){
            m_f32TargetSpeedLimit = 0.0f;
        }
    };

//    struct GridcellElement{
//        tUInt32 m_ui32ElemCounter;
//        cv::Point2f m_cvCoordinateSum;

//        GridcellElement(){
//            m_ui32ElemCounter = 0;
//        }
//    };

//    struct GridcellElement_GCLdebug{
//            tUInt32 m_ui32ElemCounter;
//            cv::Point2f m_debugCvCoordinateSum;
//            cv::Point2f m_cvMaxValueCoord;
//            cv::Point2f m_cvMinValueCoord;

//            GridcellElement_GCLdebug(){
//                m_ui32ElemCounter = 0;
//            }
//        };

//        struct IntrinsicData {
//            tFloat32 m_f32Fx;
//            tFloat32 m_f32Fy;
//            tFloat32 m_f32Cx;
//            tFloat32 m_f32Cy;
//        } _depthIntrinsicData, _colorIntrinsicData;
//<!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        struct Transformation {
            tFloat32 m_f32Rotation[9];
            tFloat32 m_f32Translation[3];
        } _depthToColor;

    /* structure for region of interest, to be checked for obstacles */
    struct roi {
        cv::Point2f bottomleft_corner;
        tFloat32 m_f32Roi_y_width;
        tFloat32 m_f32Roi_x_height;

        roi(){
            bottomleft_corner.x = 0.0f;
            bottomleft_corner.y = 0.0f;
            m_f32Roi_y_width = 0.0f;
            m_f32Roi_x_height = 0.0f;
        }
    };

    /* status of checked region */
    enum occupancy {
        INITIALILZED = -2,
        ERROR = -1,
        FREE_SPACE = 0,
        OCCUPIED_SPACE = 1,
        STATE_PENDING = 2,
        OCCUPIED_STATIC = 3,
    };


    /* specific regions of interest that need to be checked */
    /* for checking at intersections */
    roi roi_intersection_onComingTraffic;
    roi roi_intersection_crossTrafficRight;

    /* for checking during 'on-the-road' mode*/
    roi roi_overtake_onComingTraffic;
    roi roi_overtake_originalLane;
    roi roi_overtake_ownLaneStraight;
    roi roi_overtake_ownLaneStraight_lefthalf;

    /* for checking before and after parking maneuvers */
    roi roi_parking_cross_onComing;
    roi roi_parking_cross_crossTraffic; //before pull-out

    /* counter indicating the number of free frames that occurred in a non-interrupted sequence */
    tUInt32 m_ui32FreeFrameCounter;
    tUInt32 m_ui32OccupiedFrameCounter;



    tUInt32 m_ui32OccupiedCoordinateCounterUpperBound[6];
    tUInt32 m_ui32OccupiedFrameCounterLowerBound[6];
    tUInt32 m_ui32FreeFrameCounterLowerBound[6];

    /* intersection mode */
    tUInt32 m_ui32Intrsc_freeFrameCounterLowerBound;
    tUInt32 m_ui32Intrsc_occupiedFrameCounterLowerBound;

    //TODO: continue porting here
    /*! The reference clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;


    /**************************/
    /***** set Properties *****/
    /**************************/

//  adtf::base::property_variable<tFloat32> m_propBinhsCoolnessFactor = tFloat(100.0f);

    //Location of the file which is used for storing the pitch angle
    adtf::base::property_variable<tBool> m_propbUsePitchFromFile = tBool(tTrue);

    //If enabled, the camera pitch angle stored in a file will be used
    adtf::base::property_variable<String> m_propstrPitchStorage = "./../../../../../Camera_Pitch.txt";

    //Pitch angle of the camera in degree
    adtf::base::property_variable<tInt32> m_propui32CameraPitch = tInt32(-10);

    //Pitch angle correction factor in degree. Will be ADDED to pitch (pitch + cor_factor)
    adtf::base::property_variable<tInt32> m_propi32CameraPitchCorrection = tInt32(1);

    //X-directional shift of local the coordinate system from origin of camera. Positive value means shift of the origin of the coordinate system in positive x-direction
    adtf::base::property_variable<tFloat32> m_propf32CamPositionOffsetX = tFloat(0.2f);

    //Y-directional shift of local the coordinate system from origin of camera. Positive value means shift of the origin of the coordinate system in positive y-direction
    adtf::base::property_variable<tFloat32> m_propf32CamPositionOffsetY = tFloat(-0.047f);

    //Z-directional shift of local the coordinate system from origin of camera. Positive value means shift of the origin of the coordinate system in positive z-direction
    adtf::base::property_variable<tFloat32> m_propf32CamPositionOffsetZ = tFloat(-0.22f);

    //Horizontal offset of the depth-camera-center from the rgb-camera-center in METER
//    adtf::base::property_variable<tFloat32> m_propf32CamPositionOffsetDepthToRGB = tFloat(0.04)f;

    /* Focal length and center points of camera */
    adtf::base::property_variable<tFloat32> m_propf32CameraFX = tFloat(280.610668533f);
    adtf::base::property_variable<tFloat32> m_propf32CameraFy = tFloat(282.969891862f);
    adtf::base::property_variable<tFloat32> m_propf32CameraCX = tFloat(162.0f);
    adtf::base::property_variable<tFloat32> m_propf32CameraCY = tFloat(112.0f);

    //Scale factor for depth image in meter
    adtf::base::property_variable<tFloat32> m_propf32DepthImageScaleFactor = tFloat(0.0001);

    //Maximal x-distance of points in METER that should be taken into account, from origin of local coordinate system
    adtf::base::property_variable<tFloat32> m_propf32CarMaxFieldOfViewX = tFloat(2.0);

    //Minimal x-distance of points in METER that should be taken into account, from origin of local coordinate system
    adtf::base::property_variable<tFloat32> m_propf32CarMinFieldOfViewX = tFloat(0.05);

    //Maximal height of points in METER that should be taken into account, from origin of local coordinate system
    adtf::base::property_variable<tFloat32> m_propf32CarMaxFieldOfViewZ = tFloat(0.4);

    //Minimal height of points in METER that should be taken into account, from origin of local coordinate system
    adtf::base::property_variable<tFloat32> m_propf32CarMinFieldOfViewZ = tFloat(0.05);

    //Maximal height in METER that is still regarded as 'ground' (can contain lines), above the origin of origin of local coordinate system
    adtf::base::property_variable<tFloat32> m_propf32ValidMaskMinFieldOfViewZ = tFloat(0.03);

    //Number of 'free' frames (without any obstacle detected) that is required to send 'NO_OBSTACLE' feedback
    adtf::base::property_variable<tInt32> m_propi32RoiIntersectionFreeframeLowerbound = tFloat(30);

    //Maximal number of cloud points that are tolerated for 'oncoming traffic roi' without counting area as obstacle
    adtf::base::property_variable<tInt32> m_propi32RoiOccupiedOncomingUpperbound = 100;

    //Maximal number of cloud points that are tolerated for 'cross-traffic-right roi' without counting area as obstacle
    adtf::base::property_variable<tInt32> m_propi32RoiOccupiedCrossrightUpperbound = 100;

    //Minimal number of occupied frames that are necessary for returning 'STATIC_OBSTACLE' feedback
    adtf::base::property_variable<tInt32> m_propi32RoiOccupiedcounterLowerbound = 5;

    //Number of 'free' frames (without any obstacle detected) that is required to send 'NO_OBSTACLE' feedback
    adtf::base::property_variable<tInt32> m_propi32RoiOvertakeFreeframeLowerbound = 5;

    //Maximal number of cloud points that are tolerated for 'oncoming traffic roi' without counting area as obstacle
    adtf::base::property_variable<tInt32> m_propi32RoiOvertakeOccupiedOncomingUpperbound = 100;

    //Number of 'free' frames (without any obstacle detected) that is required to send 'NO_OBSTACLE' feedback
    adtf::base::property_variable<tInt32> m_propi32RoiOvertakeObstaclestraightFreeframeLowerbound = 15;

    //Maximal number of cloud points that are tolerated for static obstacle straight in front without counting area as obstacle
    adtf::base::property_variable<tInt32> m_propi32RoiOvertakeObstaclestraightOccupiedUpperbound = 100;

    //Minimal number of occupied frames that are necessary for returning 'STATIC_OBSTACLE' feedback
    adtf::base::property_variable<tInt32> m_propi32RoiOvertakeObstaclestraightOccupiedcounterLowerbound = 5;

    //Minimal number of occupied frames that are necessary for returning 'STATIC_OBSTACLE' feedback
    adtf::base::property_variable<tInt32> m_propi32RoiOvertakeOncomoinglaneOccupiedcounterLowerbound = 5;


    /* cross parking and pull out afterwards */
    //Maximal number of cloud points that are tolerated for 'oncoming traffic roi' without counting area as obstacle
    adtf::base::property_variable<tInt32> m_propi32RoiParkingOccupiedOncomoingUpperbound = 100;

    //Maximal number of cloud points that are tolerated for 'cross traffic roi' without counting area as obstacle
    adtf::base::property_variable<tInt32> m_propi32RoiParkingOccupiedCrossrightUpperbound = 100;

    //Minimal number of occupied frames that are necessary for returning 'STATIC_OBSTACLE' feedback
    adtf::base::property_variable<tInt32> m_propi32RoiParkingOccupiedcounterLowerbound = 5;

    //Number of 'free' frames (without any obstacle detected) that is required to send 'NO_OBSTACLE' feedback
    adtf::base::property_variable<tInt32> m_propi32RoiParkingCrosstrafficFreeframeLowerbound = 15;

    //Number of 'free' frames (without any obstacle detected) that is required to send 'NO_OBSTACLE' feedback
    adtf::base::property_variable<tInt32> m_propi32RoiParkingOncomingFreeframeLowerbound = 15;


    /* Optical ACC */
    //Length of wheelbase of car in METER
    adtf::base::property_variable<tFloat32> m_propf32AccParWheelbase = 0.37;

    //      //Width of car in METER
    //	adtf::base::property_variable<tFloat32> m_propf32AccParCarwidth = 0.26);

    //Distance between front axis and front bumper-end in METER
    adtf::base::property_variable<tFloat32> m_propf32AccParAxisToBumperFront = 0.13;

    //Oblique angle of front wheels in GRAD
    adtf::base::property_variable<tFloat32> m_propf32AccParObliqueAngleFront = 0.0;

    //Oblique angle of rear wheels
//    adtf::base::property_variable<tFloat32> m_propf32AccParObliqueAngleRear = 0.0;

    //The path of the XML, defining the behaviour of the OpticalACC, has to be set here
    adtf::base::property_variable<string> m_propstringAccConfigXmlFile = "../../../../utilities/ObstacleDetectionACC/ObstacleDetecACC.xml";

    //Length of the circular segment-line to be checked in front of car (distance in METER), starting at front axis
    adtf::base::property_variable<tFloat32> m_propf32AccRoiSegmentLine = 1.30;

    //Half width of area in front of car to be checked, given in METER
    adtf::base::property_variable<tFloat32> m_propf32AccRoiYSearchtolerance = 0.16;

    //Minimal height an object can have to be regarded as obstacle, given in METER
    adtf::base::property_variable<tFloat32> m_propf32AccRoiHeightThresholdMin = 0.05;

    //Maximal height an object can have to be regarded as obstacle, given in METER
    adtf::base::property_variable<tFloat32> m_propf32AccRoiHeightThresholdMax = 0.28;
//    adtf::base::property_variable<tFloat32> m_propf32AccDeptToColorX = -0.058968;
//    adtf::base::property_variable<tFloat32> m_propf32AccDeptToColorY = -0.000043;
//    adtf::base::property_variable<tFloat32> m_propf32AccDeptToColorZ = -0.001139;


    //If enabled values specified by the xml-table are printed to the console during loading
//    adtf::base::property_variable<tBool> m_propbAccDEBUGXmlPrintInitialTable = tFalse);

    //If enabled the count of pixels regarded as obstacle is printed to the console if GREATER than zero (Warning: decreases performance
    adtf::base::property_variable<tBool> m_propbAccDEBUGInfo = tFalse;

    //If enabled values specified by the xml-table are printed to the console during loading
    adtf::base::property_variable<tBool> m_propbAccDEBUGXmlPrintInitialTable = tFalse;

    //If enabled warnings are printed to console if requested value is out of xml-data-range (Warning: decreases performance
    adtf::base::property_variable<tBool> m_propbAccDEBUGXmlBorderWarning = tFalse;


    //If enabled additional debug information is printed to the console (Warning: decreases performance)
    adtf::base::property_variable<tBool> m_propbDEBUGToConsole = tFalse;

    //If enabled additional extended debug information is printed to the console (Warning: decreases performance)
    adtf::base::property_variable<tBool> m_propbExtendedDEBUGToConsole = tFalse;

    adtf::base::property_variable<tInt32> m_propi32DEBUGType = 4;

    //If enabled the GCL-output is activated and info will be printed
    adtf::base::property_variable<tBool> m_propbGclDEBUGInfo = tFalse;

     //If enabled additional log info concerning GCL data will be printed
    adtf::base::property_variable<tBool> m_propbGclDEBUGInfoLog = tFalse;

    //If enabled, current driving corridor will be made available as binary image on output
    adtf::base::property_variable<tBool> m_propbAccDEBUGShowACCBinaryImageDynamic = tFalse;


public:
    /*----------------------------------*/
    /*------------ FUNCTIONS -----------*/
    /*----------------------------------*/


    /*------------ FROM AUDI -----------*/
    // constructor
    ObstacleDetection();

    // destructor
    ~ObstacleDetection() = default;

    // Configure
    virtual tResult Configure() override;

    // Process
    virtual tResult Process(tTimeStamp tmTimeOfTrigger) override;

private:

    // ---------- FROM USER ------------*/
    // TransmitSpeed

    tResult ProcessTargetSpeedInput();
    tResult ProcessSteeringAngleInput();
    tResult ProcessLaserScannerInput();
    tResult ProcessVideoInput();
    tResult ProcessAction(TActionStruct::Data);
    tResult ObstacleDetection::ProcessActionInput();
    TActionStruct::Data ObstacleDetection::GetCurractionStruct();
    tResult ObstacleDetection::SetCurrActionStruct(TActionStruct::Data newActionStruct);
    tResult ObstacleDetection::SetLaserScannerData(TLaserScannerData::Data laserScannerData);

    #ifdef DEBUG_MODE_OUTPUT_VIDEO
    cv::Mat generateDebugDepthFromTransformedCloud(const std::vector<cv::Point3f> &m_cloud);
    #endif

    /* Method printing objects into RGB_image, delivered by referenceSegmentLine to GCL */
    tResult CreateAndTransmitGCL(const std::vector<cv::Point2i> &referenceSegmentLine);

    /* Method printing the dynamic calculated driving lane - check area */
    tResult CreateAndTransmitGCL_dynamicDrive(const TSignalValue::Data steeringAngle);

    tResult ProcessVideo();
//TODO: Umbenennen >>>
    //tResult GetInputDepthImage(cv::Mat &image, IMediaSample* mediaSample);
//TODO: Umbenennen >>>
    //tResult transformPointCloudToCarPerspective(const std::vector<cv::Point3f> &cloud, std::vector<cv::Point3f> &m_bTransformedCloud);

    //occupancy CheckROIforObstacles(roi regionToCheck, regionType type, const std::vector<cv::Point3f> &cloud);
    occupancy CheckROIforObstacles(roi regionToCheck_OnComing, roi regionToCheck_CrossRight, const std::vector<cv::Point3f> &cloud);
    tUInt32 CheckROI(roi regionToCheck, const std::vector<cv::Point3f> &cloud);
    tBool checkROI(const cv::Point3f &point, const roi &roiToCheck) const;

    /* Methods checks for Obstacles in current field of view, returns a reduced speed depending on position of recognized obstacles */
    ProcessedCloudData ProcessOpticalAccAndGenerateValidationImage(const TSignalValue::Data steeringAngle, const std::vector<cv::Point3f> &m_bTransformedCloud, const std::vector<cv::Point3f> &orgCloud);
    cv::Mat generateDebugOutputOpticACC(TSignalValue::Data steeringAngle, const std::vector<cv::Point3f> &cloud);

    tResult ProcessAction(TActionStruct actionStruct);

    tResult TransmitFeedbackNoObstacle();
    tResult TransmitFeedbackStaticObstacle();
    tResult TransmitTargetSpeed(TSignalValue::Data newTargetSpeed_tmp);
//TODO: ToBeChanged >>>
//    TActionStruct GetCurrActionSub();
//TODO: ToBeChanged >>>
//    tResult SetCurrActionSub(TActionStruct actionSub);

    tBool GetRunningState();
    tResult SetRunningState(tBool state);

    TSignalValue::Data GetSteeringAngle();
    tResult SetSteeringAngle(TSignalValue::Data steeringAngle);

    tFloat32 GetTargetSpeedLimit();
        tResult SetTargetSpeedLimit(tFloat32 m_f32TargetSpeedLimit);

    operationalStatus GetOperationalStatus();
    tResult SetOperationalStatus(operationalStatus opStatus);

    tResult ReadFromFile(tFloat32 *pitch);

    /* */
    /** Necessary code for loading an using an xml-data file **/
    /*! reads the xml file which is set in the filter properties */
    tResult LoadConfigurationData(cFilename& m_fileConfig, vector<tFloat32>& m_xValues, vector<tFloat32>& m_yValues);
    /*! checks the loaded configuration data; checks if the x-values are in increasing order*/
    tResult CheckConfigurationData(cFilename m_fileConfig, vector<tFloat32> m_xValues);

    /*! doing the linear interpolation
     @param fl32InputValue the value which should be interpolated
     */
    tFloat32 GetLinearInterpolatedValue(tFloat32 fl32InputValue, vector<tFloat32> m_xValues, vector<tFloat32> m_yValues);


    tUInt32 GetResolutionFromIndicator(tUInt32 OACC_GridCellResolutionIndicator);
//TODO: Needed???????????????????????????????????????????????????????????????????????????????????????????????
//    Point3f transformDepthToColor(const Point3f& point, const Transformation& transformation) const;
//    Point2f projectPointToPixel(const Point3f& point, const IntrinsicData& intrinsic) const;

//    	std::vector<cv::Point3f> _pointCloud;
//<!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


    tFloat32 m_f32PitchInDegree;
    tFloat32 m_f32PitchInRad;
    tFloat32 m_f32DepthImageScaleFactor;

    /**
     * synchronisation of OnPinEvent
     */
        boost::mutex criticalSection_LaserScannerDataAccess;
        boost::mutex criticalSection_VideoDataAccess;
        boost::mutex criticalSection_ActionSubAccess;
        boost::mutex criticalSection_OperationalStatusAccess;
        boost::mutex criticalSection_SteeringAngleAccess;
        boost::mutex criticalSection_TargetSpeedLimitAccess;
        boost::mutex criticalSection_RunningStateAccess;
        boost::mutex criticalSection_TransmitFeedback;
        boost::mutex criticalSection_TransmitSpeed;


}; // ObstacleDetection
