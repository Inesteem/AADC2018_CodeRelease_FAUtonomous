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
* -- no us front signals, should be replaced by lidar signals
* have one warning (‘adtf_util::cDOM::cDOM()’ is deprecated [-Wdeprecated-declarations])

* $Adapted by:: Xiangfei#  $Date:: 2018-08-27 22:44:00#
**********************************************************************/

#pragma once

#define CID_ULTRASONIC_ACC_FILTER "ultrasonic_acc.filter.user.aadc.cid"
//#define boost


// namespace
using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

#include "ADTF3_helper.h"
#include "stdafx.h"
#include "tinyxml2.h"
//#include <mutex>

#include <boost/thread.hpp>
// UltrasonicACC
class UltrasonicACC : public cTriggerFunction
{
private:
    /*------------ STRUCTS -------------*/
    // create structs to hold information we get from SignalValues
    // coding convention: 	o_******Id

    TUltrasonicStruct o_UltrasonicStruct;
    TActionStruct   o_ActionStruct;
    TSignalValue    o_TSignalValue;
    TFeedbackStruct o_FeedbackStruct;
    TLaserScannerData o_LaserScanner;
    TLaneDetectionLineStruct o_LaneDetectionLineStruct;

    /*-------------- PINS --------------*/
    // create pins for input and output
    // coding convention:	m_Reader******	for input  signals
    //			m_Writer******	for output signals

    // input pins
    cPinReader m_ReaderUltrasonic;
    cPinReader m_ReaderAction;
    cPinReader m_ReaderTargetSpeed;
    cPinReader m_ReaderSteeringAngle;
    cPinReader m_ReaderLaserScanner;
    cPinReader m_ReaderLaneDetectionLine;
    cPinReader m_ReaderVideo;

    // output pins
    cPinWriter m_WriterVideo;
    cPinWriter m_WriterVideoDebug;
    cPinWriter m_WriterTargetSpeed;
    cPinWriter m_WriterFeedback;
    cPinWriter m_WriterRelevantLaser;
    cPinWriter m_WriterRampGuardrail;

    /*------------ PROPERTIES ----------*/


    adtf::base::property_variable<cFilename> m_propConfigFileDriving = cFilename(cString("/home/aadc/AADC/utilities/UltrasonicACC/UltrasonicACC_Driving.xml"));
    adtf::base::property_variable<cFilename> m_propConfigFileMtP= cFilename(cString("/home/aadc/AADC/utilities/UltrasonicACC/UltrasonicACC_Bend.xml"));
    adtf::base::property_variable<cFilename> m_propConfigFileBend = cFilename(cString("/home/aadc/AADC/utilities/UltrasonicACC/UltrasonicACC_MtP.xml"));
    adtf::base::property_variable<tBool> m_propBDebugXMLBorderWarning = tFalse;
    adtf::base::property_variable<tBool> m_propBSetUSValuesMax = tFalse;
    adtf::base::property_variable<tBool> m_propBDebugPrintInitialTable = tFalse;
    adtf::base::property_variable<tInt32> m_propI32CounterThresholdNoMove = tInt(2);
    adtf::base::property_variable<tInt32> m_propI32CounterThresholdMoveAgain = tInt(1);
    adtf::base::property_variable<tFloat32> m_propF32LowerBoundSpeed = 6.5;
    adtf::base::property_variable<tFloat32> m_propF32SensorScalingFront = 8;
    adtf::base::property_variable<tFloat32> m_propF32SensorScalingRear = 8;
    adtf::base::property_variable<tFloat32> m_propF32SensorOffsetSteering = 4.5;// to drive straight on the sttering has to be -15 -> add 15

    adtf::base::property_variable<tFloat32> m_propF32SensorFrontCheckLimit = 0.4;
    adtf::base::property_variable<tBool> m_propBDebugOutputToConsole = tFalse;
    adtf::base::property_variable<tBool> m_propBExtDebugOutputToConsole = tFalse;
    adtf::base::property_variable<tBool> m_propBDebugRamp = tFalse;



    adtf::base::property_variable<tBool> m_propBLaserOutputDrive = tTrue;
    adtf::base::property_variable<tBool> m_propBLaserOutputGeneral = tFalse;//noch nicht im xml!!!!

    adtf::base::property_variable<tFloat32> m_propF32ObjectThresholdAngle = 3.0; //[°]
    adtf::base::property_variable<tFloat32> m_propF32ObjectThresholdRadius = 15.0; // [mm]if the distance between two points is less, the points will considerate as one object
    adtf::base::property_variable<tFloat32> m_propF32ObjectThresholdDetection = 2000.0; //[mm]
    //range of value of steering is from -100 to 100; steering *  m_propF32ScaleSteering = angle of wheels
    adtf::base::property_variable<tFloat32> m_propF32ScaleSteering = 1.0;
    adtf::base::property_variable<tBool>    m_propBDebugFirstStart = tFalse;
    adtf::base::property_variable<tFloat32> m_propF32CloseObjectThresholdAngle = 40.0; //[°]
    adtf::base::property_variable<tFloat32> m_propF32ThresholdStraight = 5.0; //[°]
    adtf::base::property_variable<tFloat32> m_propF32OffsetCamLidar = 0.0;//-20.0;//[mm]
    adtf::base::property_variable<tBool>    m_propBDebugROI = tTrue;//m_propBDebugROI
    adtf::base::property_variable<tFloat32> m_propF32wheelbase = 370;//[mm]
    adtf::base::property_variable<tFloat32> m_propF32circSegmentDistance = 1300;//[mm]
    adtf::base::property_variable<tFloat32> m_propF32DistanceAxisToBumper = 120;//[mm] //in 2017: 130
    //adtf::base::property_variable<tFloat32> m_propF32DistanceInnerLaneTo = 120;
    adtf::base::property_variable<tFloat32> m_propF32ThresholdVisualRangeBend = 400.0;
    adtf::base::property_variable<tFloat32> m_propF32ThresholdReducedVisualRangeOneSide = 10.0;//[mm ]
    adtf::base::property_variable<tFloat32> m_propF32OffsetParkingY = 0.0;//[mm ]

    adtf::base::property_variable<tUInt32> m_propUI32FreeROIBoundary = 3;
    adtf::base::property_variable<tUInt32> m_propUI32OccupiedROIBoundary = 3;
    adtf::base::property_variable<tUInt32> m_propUI32TemporaryOccupiedROIBoundary = 3;
    adtf::base::property_variable<tUInt32> m_propUI32EndOfForLoopToCheckROI = 20;

    adtf::base::property_variable<tBool> m_propBDebugCurve = tFalse;
    adtf::base::property_variable<tBool> m_propBDebugActionCommand = tFalse;
    adtf::base::property_variable<tBool> m_propBDebugOvertakingROI = tFalse;
    adtf::base::property_variable<tBool> m_propBDebugStraight = tFalse;//
    adtf::base::property_variable<tBool> m_propBDebugDoll = tTrue;//nicht in xml
    //for driving in the curve
    adtf::base::property_variable<tFloat32> m_propF32InnerRoadway = 440.0f;
    adtf::base::property_variable<tFloat32> m_propF32HighBoxBend = 100.0f;
    adtf::base::property_variable<tFloat32> m_propF32ThresholdRelObstBend = 1200.0f; //Threshold for relevant obstacles in a bend

    adtf::base::property_variable<tFloat32> m_propF32DistanceToStopLine = 20.0f;

    adtf::base::property_variable<tFloat32> m_propF32ThresholdSlopeTooHigh = 0.01f;
    adtf::base::property_variable<tFloat32> m_propF32PixToMM = 3.0f;
    adtf::base::property_variable<tFloat32> m_propF32CamOffsety = 150.0f;//Offset cam to lidar [mm]

    adtf::base::property_variable<tFloat32> m_propF32CarWidth = 360.0f;//#define USACC_CAR_WIDTH 360//[mm] with the exterior mirrors

    adtf::base::property_variable<tFloat32> m_propF32XminOfRamp = -800.0f;
    adtf::base::property_variable<tFloat32> m_propF32XmaxOfRamp = 400.0f;
    adtf::base::property_variable<tFloat32> m_propF32ThresholdSlopeForRamp = tFloat32(1.41);
    adtf::base::property_variable<tFloat32> m_propF32ThresholdDifferenzYNotOnLine = tFloat32(15.0);
    adtf::base::property_variable<tFloat32> m_propF32MinLengthOfTheRamp = tFloat32(900.0);
    adtf::base::property_variable<tUInt32> m_propui32MinNumOfPointsRamp = tUInt32(7);
    adtf::base::property_variable<tUInt32> m_propui32NumberLaneDetectionInputForRamp = tUInt32(5);
    adtf::base::property_variable<tFloat32> m_propF32MaxLengthOfTheRamp = tFloat32(1500.0);
    adtf::base::property_variable<tFloat32> m_propF32YDiffRamp = tFloat32(300.0);
    adtf::base::property_variable<tFloat32> m_propF32XDiffMax = tFloat32(150.0);
    adtf::base::property_variable<tUInt32> m_propui32NumNoGuardrailForOffRamp = tUInt32(15);//nicht im xml file
    adtf::base::property_variable<tFloat32> m_propf32ThresholdRampDrivingY = tFloat32(600);//nicht in xml
    adtf::base::property_variable<tFloat32> m_propf32ThresholdRampDrivingX = tFloat32(750);//nicht in xml

    adtf::base::property_variable<tFloat32> m_propF32IntersectionOncomingXmin = -600.0f;
    adtf::base::property_variable<tFloat32> m_propF32IntersectionOncomingYmin = 900.0f;
    adtf::base::property_variable<tFloat32> m_propF32IntersectionOncomingXmax = -300.0f;
    adtf::base::property_variable<tFloat32> m_propF32IntersectionOncomingYmax = 1900.0f;

    adtf::base::property_variable<tFloat32> m_propF32IntersectionCrosstrafficRightXmin = 200.0f;
    adtf::base::property_variable<tFloat32> m_propF32IntersectionCrosstrafficRightYmin = 600.0f;
    adtf::base::property_variable<tFloat32> m_propF32IntersectionCrosstrafficRightXmax = 600.0f;
    adtf::base::property_variable<tFloat32> m_propF32IntersectionCrosstrafficRightYmax = 950.0f;

    adtf::base::property_variable<tFloat32> m_propF32IntersectionCrosstrafficLeftXmin = -1100.0f;
    adtf::base::property_variable<tFloat32> m_propF32IntersectionCrosstrafficLeftYmin = 550.0f;
    adtf::base::property_variable<tFloat32> m_propF32IntersectionCrosstrafficLeftXmax = -700.0f;
    adtf::base::property_variable<tFloat32> m_propF32IntersectionCrosstrafficLeftYmax = 900.0f;

    adtf::base::property_variable<tFloat32> m_propF32OvertakeOncomingXmin = -550.0f;
    adtf::base::property_variable<tFloat32> m_propF32OvertakeOncomingYmin = 300.0f;
    adtf::base::property_variable<tFloat32> m_propF32OvertakeOncomingXmax = -250.0f;
    adtf::base::property_variable<tFloat32> m_propF32OvertakeOncomingYmax = 1200.0f;

    adtf::base::property_variable<tFloat32> m_propF32OvertakeOriginallaneXmin = 270.0f;
    adtf::base::property_variable<tFloat32> m_propF32OvertakeOriginallaneYmin = 300.0f;
    adtf::base::property_variable<tFloat32> m_propF32OvertakeOriginallaneXmax = 570.0f;
    adtf::base::property_variable<tFloat32> m_propF32OvertakeOriginallaneYmax = 1200.0f;

    adtf::base::property_variable<tFloat32> m_propF32OvertakeObstacleStraightXmin = -150.0f;
    adtf::base::property_variable<tFloat32> m_propF32OvertakeObstacleStraightYmin = 50.0f;
    adtf::base::property_variable<tFloat32> m_propF32OvertakeObstacleStraightXmax = 150.0f;
    adtf::base::property_variable<tFloat32> m_propF32OvertakeObstacleStraightYmax = 950.0f;

    adtf::base::property_variable<tFloat32> m_propF32OvertakeObstacleStraightLeftHalfXmin = -300.0f;
    adtf::base::property_variable<tFloat32> m_propF32OvertakeObstacleStraightLeftHalfYmin = 50.0f;
    adtf::base::property_variable<tFloat32> m_propF32OvertakeObstacleStraightLeftHalfXmax = -100.0f;
    adtf::base::property_variable<tFloat32> m_propF32OvertakeObstacleStraightLeftHalfYmax = 950.0f;

    adtf::base::property_variable<tFloat32> m_propF32ParkingCrossOncomingLaneXmin = -450.0f;
    adtf::base::property_variable<tFloat32> m_propF32ParkingCrossOncomingLaneYmin = 100.0f;
    adtf::base::property_variable<tFloat32> m_propF32ParkingCrossOncomingLaneXmax = -100.0f;
    adtf::base::property_variable<tFloat32> m_propF32ParkingCrossOncomingLaneYmax = 800.0f;

    adtf::base::property_variable<tFloat32> m_propF32ParkingPullOutCrossOncoming1Xmin = -700.0f;
    adtf::base::property_variable<tFloat32> m_propF32ParkingPullOutCrossOncoming1Ymin = 0.0f;
    adtf::base::property_variable<tFloat32> m_propF32ParkingPullOutCrossOncoming1Xmax = 700.0f;
    adtf::base::property_variable<tFloat32> m_propF32ParkingPullOutCrossOncoming1Ymax = 900.0f;

    adtf::base::property_variable<tFloat32> m_propF32CheckCrosswalkXmin = -700.0f;
    adtf::base::property_variable<tFloat32> m_propF32CheckCrosswalkXmax = 200.0f;
    adtf::base::property_variable<tFloat32> m_propF32CheckCrosswalkYmin = 0.0f;//depends where we stop
    adtf::base::property_variable<tFloat32> m_propF32CheckCrosswalkYmax = 1000.0f;
    /*not used*/
    adtf::base::property_variable<tFloat32> m_propF32ParkingPullOutCrossOncoming2Xmin = 0.0f;
    adtf::base::property_variable<tFloat32> m_propF32ParkingPullOutCrossOncoming2Ymin = 0.0f;
    adtf::base::property_variable<tFloat32> m_propF32ParkingPullOutCrossOncoming2Xmax = 1.0f;
    adtf::base::property_variable<tFloat32> m_propF32ParkingPullOutCrossOncoming2Ymax = 1.0f;

    adtf::base::property_variable<tFloat32> m_propF32OvertakeObstacleOnlyRightHandSideXmin = -500.0f;
    adtf::base::property_variable<tFloat32> m_propF32OvertakeObstacleOnlyRightHandSideXmax = 0.0f;
    adtf::base::property_variable<tFloat32> m_propF32OvertakeObstacleOnlyRightHandSideYmin = 0.0f;
    adtf::base::property_variable<tFloat32> m_propF32OvertakeObstacleOnlyRightHandSideYmax = 1000.0f;

    adtf::base::property_variable<tFloat32> m_propF32MergeCheckTargetLaneXmin = -550.0f;
    adtf::base::property_variable<tFloat32> m_propF32MergeCheckTargetLaneYmin = 0.0f;
    adtf::base::property_variable<tFloat32> m_propF32MergeCheckTargetLaneXmax = -250.0f;
    adtf::base::property_variable<tFloat32> m_propF32MergeCheckTargetLaneYmax = 600.0f;

    adtf::base::property_variable<tFloat32> m_propF32ROIDollXmin = -500.0f;//nicht xml
    adtf::base::property_variable<tFloat32> m_propF32ROIDollYmin = 0.0f;//nicht xml
    adtf::base::property_variable<tFloat32> m_propF32ROIDollXmax = -150.0f;//nicht xml
    adtf::base::property_variable<tFloat32> m_propF32ROIDollYmax = 800.0f;//nicht xml

    adtf::base::property_variable<tFloat32> m_propF32DollWidth = 100.0f;//nicht xml
    adtf::base::property_variable<tFloat32> m_propF32ThresholdDollAnotherObject = 50.0f;//nicht xml
    adtf::base::property_variable<tFloat32> m_propF32DistanceToStopNormal = 220.0f;//nicht xml
    adtf::base::property_variable<tFloat32> m_propF32DistanceToStopAfterRight = -100.0f;//nicht xml
    adtf::base::property_variable<tFloat32> m_propF32DistanceToStopAfterLeft = 350.0f;//nicht xml

    // ----- IMAGE PROCESSING ------
    adtf::base::property_variable<tUInt32> m_propUI32Frames = tUInt32(4);
    adtf::base::property_variable<tUInt32> m_propROIPoint1X = tUInt32(440);
    adtf::base::property_variable<tUInt32> m_propROIPoint1Y = tUInt32(500);
    adtf::base::property_variable<tUInt32> m_propROIPoint2X = tUInt32(780);
    adtf::base::property_variable<tUInt32> m_propROIPoint2Y = tUInt32(500);
    adtf::base::property_variable<tUInt32> m_propROIPoint3X = tUInt32(1280);//nicht sicher, ob die zahl stimmt
    adtf::base::property_variable<tUInt32> m_propROIPoint3Y = tUInt32(600);
    adtf::base::property_variable<tUInt32> m_propROIPoint4X = tUInt32(0);
    adtf::base::property_variable<tUInt32> m_propROIPoint4Y = tUInt32(600);
    adtf::base::property_variable<tUInt32> m_propTransformedImageWidth  = tUInt32(640);
    adtf::base::property_variable<tUInt32> m_propTransformedImageHeight = tUInt32(640);


    object_ptr<adtf::services::IReferenceClock> m_pClock; // clock for timestamps

    /*------------ VARIABLES -----------*/
    // coding convention:	m_******

    // image formats for video pins
    adtf::streaming::tStreamImageFormat m_sInputImageFormat;
    adtf::streaming::tStreamImageFormat m_sOutputImageFormat;

    TUltrasonicStruct::Data m_dataUltrasonic;
    TLaserScannerData::Data m_dataLidar;
    TSignalValue::Data m_dataSpeed;
    //TSignalValue::Data m_dataOriginalSpeed;
    TSignalValue::Data m_dataSteering;
    TSignalValue::Data m_dataSpeedOut;
    TActionStruct::Data m_dataAction;
    TFeedbackStruct::Data m_dataFeedback;
    TFeedbackStruct::Data m_dataFeedbackAtRest;
    TFeedbackStruct::Data m_dataFeedbackMovingAgain;
    TFeedbackStruct::Data m_dataFeedbackNoObstacel;
    TFeedbackStruct::Data m_dataFeedbackStaticObstacel;
    TFeedbackStruct::Data m_dataFeedbackRamp;
    TFeedbackStruct::Data m_dataFeedbackOffRamp;

    tUInt32 m_ui32FrameCounter = 0;

    tInt64 m_i64GlobalCounter = 0;

    //for the ramp
    tBool m_bIsOnTheRamp = tFalse;
    tUInt32 m_ui32CounterNoGuardRail = 0;
    //TPolarCoordiante::Data lastLidarPointNegValue;
    //TPolarCoordiante::Data lastLidarPointPosValue;
    tFloat32 lastValidXValue = 0.3;

    //tInt64 m_i64LaneDetectionInputCounter = 0;

    vector <tBool> vecBvalidLane;


    TUltrasonicStruct::Data m_dataUltrasonicTemp; // [m]
    TLaserScannerData::Data m_dataLidarTmp; //[mm]

    TSignalValue::Data m_dataSteeringAngleTemp;

    tUInt32 m_I32DebugLogCounter;

    tUInt32 m_UI32MaxCounter;


    //enums list
    enum currentOperationalMode
    {
        DRIVING_MODE_ACTIVE = 1, // default mode
        MtP_MODE_ACTIVE = 2,
        DRIVING_MODE_CURVE = 3,
        DRIVING_MODE_RAMP =4,
    };

    enum currentIntersectionMode
    {
        NORMAL_DRIVE = 0,
        AFTER_RIGHT = 1,
        AFTER_LEFT = 2,
    };


    struct USweightsRear
    {
        tFloat32 us_rearLeft;
        tFloat32 us_rearCenter;
        tFloat32 us_rearRight;

        USweightsRear()
        {
            us_rearLeft = 0.0f;
            us_rearCenter = 0.0f;
            us_rearRight = 0.0f;
        }
    };

    struct RedPercentRear
    {
        tFloat32 RedPercRearLeft;
        tFloat32 RedPercRearCenter;
        tFloat32 RedPercRearRight;

        RedPercentRear()
        {
            RedPercRearLeft = 0.0f;
            RedPercRearCenter = 0.0f;
            RedPercRearRight = 0.0f;
        }
    };

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
    struct roiXY
    {
        tFloat32 f32x_min;
        tFloat32 f32x_max;
        tFloat32 f32y_min;
        tFloat32 f32y_max;

        roiXY()
        {
            f32x_max = 0.0f;
            f32x_min = 0.0f;
            f32y_max = 0.0f;
            f32y_min = 0.0f;
        }
    };

    struct xyPoints
    {
        tFloat32 x_Point;
        tFloat32 y_Point;

        xyPoints()
        {
            x_Point = 0.0;
            y_Point = 0.0;
        }
    };

    //used for the line in the bend
    //line 1 is on the left hand side
    struct pointOnTwoLanes
    {
        tFloat32 f32x1line1;
        tFloat32 f32y1line1;
        tFloat32 f32x2line1;
        tFloat32 f32y2line1;
        tFloat32 f32x1line2;
        tFloat32 f32y1line2;
        tFloat32 f32x2line2;
        tFloat32 f32y2line2;

        pointOnTwoLanes()
        {
            f32x1line1 = 0.0f;
            f32y1line1 = 0.0f;
            f32x2line1 = 0.0f;
            f32y2line1 = 0.0f;
            f32x1line2 = 0.0f;
            f32y1line2 = 0.0f;
            f32x2line2 = 0.0f;
            f32y2line2 = 0.0f;
        }
    };

    //was used in Obstacel Detection 2017
    //Attention x was defined in movment direction and y vertical to it (not like in maths)
    struct roi {
        tFloat32 f32Roi_x_bottom_left_corner;
        tFloat32 f32Roi_y_bottom_left_corner;
        tFloat32 f32Roi_y_width;
        tFloat32 f32Roi_x_height;

        roi(){
            f32Roi_x_bottom_left_corner = 0.0f;
            f32Roi_y_bottom_left_corner = 0.0f;
            f32Roi_y_width = 0.0f;
            f32Roi_x_height = 0.0f;
        }
    };



    /*for intersection checking*/
    /* status of checked region */
    enum occupancy {
        INITIALILZED = -2,
        ERROR = -1,
        FREE_SPACE = 0,
        OCCUPIED_SPACE = 1,
        STATE_PENDING = 2,
        OCCUPIED_STATIC = 3,
    };

    //My Idea: maybe I will do it like this
    /*for intersection checking*/
    /* which region has to be checked */
    /*enum regionToCheck : short {
        NOREGION = 0,
        RIGHTSIDE = 1,
        LEFTSIDE = 2,
        ONCOMINGTRAFFIC = 3,
        RIGHTANDONCOMING = 4,
        LEFTANDRIGHT = 5,
        ALL = 6,
    };*/
    TLaneDetectionLineStruct::Data m_dataLastLane;
    TLaneDetectionLineStruct::Data m_dataDefaultLanes;
    UltrasonicACC::pointOnTwoLanes m_dataLastProcessedPoints;
    currentIntersectionMode m_intersectionMode = NORMAL_DRIVE;

    tUInt32 m_ui32SizeOfOutput = 0;


public:
    /*------------ FUNCTIONS -----------*/
    // constructor
    UltrasonicACC();

    // destructor
    ~UltrasonicACC() = default;

    // configure
    virtual tResult Configure() override;

    // process
    virtual tResult Process(tTimeStamp tmTimeOfTrigger) override;

    /*Process fuctions
     * read the pin and process the readed input or call an additional process function*/
    tResult ProcessUS();
    tResult ProcessAction();
    tResult ProcessSteeringAngle();
    //tResult ProcessOriginalSpeed();
    tResult ProcessTargetSpeed();

    /* functions for processing and transmitting */
    tResult ProcessUSInput(TUltrasonicStruct::Data tmp_USdata);
    tResult ProcessLIDARInput(TLaserScannerData::Data tmp_LIDARdata);
    TUltrasonicStruct::Data GetUSInput();
    TLaserScannerData::Data GetLIDARInput();
    UltrasonicACC::LidarObstacles GetLIDARObstacles();

    tResult SetLIDARInput(TLaserScannerData::Data &LIDARinput);
    tResult SetLIDARObstaclesInput(UltrasonicACC::LidarObstacles &LidarObstaclesInput);

    TSignalValue::Data GetSteeringAngleInput();
    tFloat32 CalcTmpWeight(tInt32 sensorAngleSetting, tFloat32 tmp_steeringAngle, tUInt32 scalingFactor);
    //USweightsSide GetSensorWeightsFront(TSignalValue::Data steeringAngleData);
    USweightsRear GetSensorWeightsRear(TSignalValue::Data steeringAngleData);
    tFloat32 GetReductionPercentage(tFloat32 distance, currentOperationalMode tmp_operationalMode);
    //RedPercentSide GetRedPerscentFront(TUltrasonicStruct::Data USStuct,currentOperationalMode tmp_operationalMode);
    RedPercentRear GetRedPerscentRear(TUltrasonicStruct::Data USStuct,currentOperationalMode tmp_operationalMode);
    //Process Lidar Data to foind objects
    UltrasonicACC::LidarObstacles ObstacleDetectionWithLidar(TLaserScannerData::Data lasersample);
    //helper function for sorting the LaserScannerData; called by ObjectDetectionWithLidar

    tResult TransmitTargetSpeed(TSignalValue::Data mod_targetSpeed);
    tBool CheckIfObstacleIsRamp();
    tBool CheckForGuardRail();
    tBool fitPoints(const std::vector<LidarPoint> &pts, tFloat32 &slope, tFloat32 &yInt);
    tBool GetValidLane();
    tResult SetLastElementValidLane(tBool validLane);

    tResult ProcessDriveOnRamp();


    //tResult TransmitFeedback(TFeedbackStruct::Data feedback);

    currentOperationalMode GetOperationalMode();
    tResult  SetOperationalMode(currentOperationalMode  new_mode);

    currentOperationalMode operationalMode;

    /** Necessary code for loading and using an xml-data file **/

    /*! reads the xml file which is set in the filter properties */
    tResult LoadConfigurationData(cFilename& m_fileConfig, vector<tFloat32>& m_xValues, vector<tFloat32>& m_yValues,currentOperationalMode OpMode);
    /*! checks the loaded configuration data; checks if the xvalues are in increasing order*/
    tResult CheckConfigurationData(cFilename m_fileConfig, vector<tFloat32> m_xValues);

    // set property variables
    // coding convention:	m_prop******
    //    vector<tFloat32> m_vecYValuesDriving {0, 0, 7, 10, 16.0, 16.0};
    //    /*! holds the xValues for the supporting points*/
    //    //vector<tFloat32> m_vecXValuesDriving {0, 0.1, 0.3, 0.4, 0,5, 0,51, 5.0};
    //    vector<tFloat32> m_vecXValuesDriving {0, 300.0, 500.0, 900.0, 5000.0, 10100.0};//[mm]


    //    /*! holds the yValues for the supporting points*/
    //    vector<tFloat32> m_vecYValuesMtP {0, 7, 16, 16, 16, 16};
    //    /*! holds the xValues for the supporting points*/
    //    vector<tFloat32> m_vecXValuesMtP {0, 50.0, 150.0, 250.0, 400.0, 5000.0};

    //    vector<tFloat32> m_vecYValueBend {0,7,10,12,12,12};

    //    vector<tFloat32> m_vecXValueBend {0, 50.0, 150.0, 250.0, 400.0, 5000.0};

    // holds the yValues for the supporting points
    vector<tFloat32> m_vecXValuesDriving={0.0, 100.0, 300.0, 400.0, 500.0, 600.0, 900.0, 5000.0, 5001.0};
    /*! holds the xValues for the supporting points*/
    vector<tFloat32> m_vecYValuesDriving = {0.0, 0.0, 0.0, 0.6, 0.6, 1.41, 3.7, 3.7, 3.7};

    /*! holds the yValues for the supporting points*/
    vector<tFloat32> m_vecYValuesBend = {0.0,0.0,0.0,0.0,2.21,3.1,3.1};
    /*! holds the xValues for the supporting points*/
    vector<tFloat32> m_vecXValuesBend = {0.0, 50.0, 250.0, 450.0, 550.0, 700.0, 5000.0};

    vector<tFloat32> m_vecYValuesMtP = {3.7, 3.7, 3.7, 3.7, 3.7, 3.7};
    vector<tFloat32> m_vecXValuesMtP = {0.0, 50.0, 150.0, 250.0, 400.0, 5000.0};

    /*! doing the linear interpolation
             @param fl32InputValue the value which should be interpolated
             */
    tFloat32 GetLinearInterpolatedValue(tFloat32 fl32InputValue, vector<tFloat32> m_xValues, vector<tFloat32> m_yValues);

    tResult PropertyChanged(const tChar* );
    //TUltrasonicStruct::Data ComputeWeightedDistanceSide(TUltrasonicStruct::Data , USweightsSide);
    TUltrasonicStruct::Data ComputeWeightedDistanceRear(TUltrasonicStruct::Data , USweightsRear);
    //TSignalValue::Data ChooseOutputSpeedSide(RedPercentSide , TSignalValue::Data );
    TSignalValue::Data ChooseOutputSpeedRear(RedPercentRear , TSignalValue::Data
                                             );

    TActionStruct::Data lastInputAction;
    TActionStruct::Data GetLastActionInput();
    tResult SetLastActionInput(TActionStruct::Data lastInput);

    tUInt32 m_I32TimeoutCounter;
    tResult IncreaseTimeoutCounter();
    tResult ResetTimeoutCounter();
    tUInt32 GetTimeoutCounter();

    tResult TransmitFeedbackNoObstacle();
    tResult TransmitFeedbackStaticObstacle();
    tResult TransmitFeedbackIntersectionROIreceived();
    tResult TransmitFeedBackDrivingModeActive();
    tResult TransmitFeedBackMTPMode();
    tResult TransmitFeedbackRamp();
    tResult TransmitFeedbackDoll();
    tResult TransmitFeedBackOffRamp();

    tResult IncreaseSizeOfOutputLaser();
    tUInt32 GetSizeOfOutputLaser();
    tResult ResetSizeOfOutputLaser();

    tBool m_BDeactivateACCFilter;//not used

    UltrasonicACC::LidarObstacles m_dataLidarObstacelesTmp;



    tUInt32 m_UI32FreeROICounter = 0;
    tUInt32 m_UI32OccupiedROICounter = 0;

    tFloat32 m_F32ModifiedSpeedLowerBound;
    tUInt32 m_UI32ObstacleNoMovementCounter;
    tUInt32 m_UI32NoMovementThreshold;
    tUInt32 m_UI32ObstacleMovementAgainCounter;
    tUInt32 m_UI32MovementAgainThreshold;
    tResult IncreaseObstacleNoMovementCounter();
    tResult ResetObstacleNoMovementCounter();
    tUInt32 GetObstacleNoMovementCounter();
    tResult IncreaseObstacleMovementAgainCounter();
    tResult ResetObstacleMovementAgainCounter();
    tUInt32 GetObstacleMovementAgainCounter();

    tResult SetLastDistanceForRampChecking(tFloat32 last_distance);
    tFloat32 SetLastDistanceForRampChecking();
    tResult IncreaseCounterGuardRailsDetected();
    tResult ResetCounterGuardRailsDetected();

    tFloat32 m_F32LastInputTargetSpeed;
    tFloat32 m_F32LastInputOriginalTargetSpeed;
    tFloat32 m_F32LastModTargetSpeed;
    tResult SetLastInputTargetSpeed(tFloat32 inputTargetSpeed);
    tFloat32 GetLastInputTargetSpeed();
    tResult SetLastModifiedTargetSpeed(tFloat32 modifiedTargetSpeed);
    tFloat32 GetLastModifiedTargetSpeed();

    tResult ProcessLaneDetectionLine();
    tResult ProcessValidLane(TLaneDetectionLineStruct::Data lastLane);

    tResult SetLastLaneDetectionLine(TLaneDetectionLineStruct::Data &lastLane);
    TLaneDetectionLineStruct::Data GetLastLaneDetectionLine();

    tBool GetRunningState();
    tResult SetRunningState(tBool new_state);
    tBool m_BRunningState;
    tBool GetRunningStateMoveAgain();
    tResult SetRunningStateMoveAgain(tBool new_state);
    tBool m_BRunningStateMoveAgain;
    tFloat32 m_f32LastDistanceForRampChecking = 0.0;
    tUInt32 m_ui32CounterGuardRailsDetected = 0;
    UltrasonicACC::roiXY defaultROI;

    /*! holds the xml file for the supporting points*/
    /* name of xml-file for driving mode of ACC */
    cFilename m_fileDrivingModeACC;
    cFilename m_fileBendModeACC;

    cFilename m_fileMtPModeACC;
    /*! critical section */
    //boost::lock_guard<boost::mutex> lock(criticalSectionSetSteeringAngle);
    boost::mutex  criticalSectionLatestRecTargetSpeedAccess, criticalSectionSetRunningStateAccess, criticalSectionSetRunningStateMoveAgainAccess, criticalSectionSetLidarInputAccess, criticalSection_TransmitFeedback, criticalSectionSetLidarObstacelsInputAccess, criticalSectionSetSteeringAngle, criticalSectionTimeoutCounter, criticalSectionUS, criticalSectionCheckROIforObstacles, criticalSectionOperationalMode, criticalSectionObstacleMovementAgainCounter, criticalSectionObstacleNoMovementAgainCounter, criticalSectionLastTargetSpeed, criticalSectionModifiedTargetSpeed, criticalSectionLaneDetectionLine, criticalSectionLastProcessedLines, criticalSectionLastInputAction, criticalSectionProcessSteeringAngle, criticalSectionTransmitRelevantLaser, criticalSectionTransmitTragetSpeed, criticalSectionSizeOfOutputLaser, criticalSectionGlobalCounter, criticalSectionValidLane, criticalSectorMergeFlag, criticalSectionIntersectionMode, criticalSectionDistanceForRampChecking, criticalSectionCounterGuardrailsDetected;
    /*! enable/disable warning at reached borders of the xml-file */
    tBool m_bBorderWarningModeEnabled;
    tBool m_bPrintInitialtable;

    const tFloat32 USACC_wheelbase = 370; //[mm]

    /*Converted roi*/
    UltrasonicACC::roiXY roiXYIntersectionOncoming;
    UltrasonicACC::roiXY roiXYIntersectionCrosstraffic;
    UltrasonicACC::roiXY roiXYIntersectionCrosstrafficLeft;
    UltrasonicACC::roiXY roiXYOvertakingOncomingTraffic;
    UltrasonicACC::roiXY roiXYOvertakingOriginalLane;
    UltrasonicACC::roiXY roiXYOvertakingObstacleStraight;
    UltrasonicACC::roiXY roiXYOvertakingObstacleStraightLeftHalf;
    UltrasonicACC::roiXY roiXYParkingOutBox1;
    UltrasonicACC::roiXY roiXYParkingOutBox2;
    UltrasonicACC::roiXY roiXYParkingCrossOncomingTrafficRight;
    UltrasonicACC::roiXY roiXYCrosswalk;
    UltrasonicACC::roiXY roiXYOvertakingCheckIfObstacleIsOnlyRightHandSide;
    UltrasonicACC::roiXY roiXYMergeCheckTargetLane;


    cFilename debugFileDirectory = "/home/aadc/AADC/src/aadcUser/UltrasonicACC/debug/ACC_segfault.dat";
    tBool debugToFile = tFalse;

    /* scaling factor for adapting the weighting function for the US-sensors depending on steering angle */
    tUInt32 m_UI32ScalingFactorUSFront;
    tUInt32 m_UI32ScalingFactorLIDAR;
    tUInt32 m_UI32ScalingFactorUSRear;

    tUInt32 m_ui32DebugLogEventToggle;

    const tFloat32 f32InnerRoadwayLineWidth = 20.0;

    /* US values greater this property are ignored, area should then be checked with 3D camera (values in meter)	 */
    tFloat32 m_f32FrontSensorsCheckLimit;

    tResult SetSteeringAngle(TSignalValue::Data steering);
    /* enable/disable debug messages to console */
    tBool m_bDebugModeEnabled;
    tBool m_bExtendedDebugModeEnabled;
    /* merge -> expect ramp*/
    tBool m_bMergeIntoLane = tFalse;

    tFloat32 GetLinearInterpolatedValueBend(tFloat32 f32InputValue, tFloat32 x1, tFloat32 y1, tFloat32 x2, tFloat32 y2);

    tResult SetLastProcessPoints(UltrasonicACC::pointOnTwoLanes processedLanes);
    UltrasonicACC::pointOnTwoLanes GetLastProcessedPoints();

    tResult SetIntersectionMode(currentIntersectionMode currentMode);
    UltrasonicACC::currentIntersectionMode GetIntersectionMode();

    tFloat32 AngleCompensation(tFloat32 angle);
    tFloat32 CalcClosestRelevantObstacle(UltrasonicACC::LidarObstacles tmp_dataLidarObstacle);
    tFloat32 GetRedPerscentFront(tFloat32 distance, currentOperationalMode tmp_operationalMode);
    tFloat32 ChooseOutputSpeedFront(tFloat32 interpolatedSpeed, TSignalValue::Data targetSpeed);
    tFloat32 CalcAngleFromDistance(tFloat32 distance);
    //UltrasonicACC::LidarROI CamXYtoLidarCoord(UltrasonicACC::roi roi);

   /*Functions needed in process Action*/
    tResult ProcessActionIntersectionCheckAll();
    tResult ProcessActionIntersectionOncomingTraffic();
    tResult ProcessActionIntersectionCrosstrafficRight();//
    tResult ProcessIntersectionCheckOncomingAndRight();
    tResult ProcessActionIntersectionCheckLeft();
    tResult ProcessActionIntersectionNothingToCheck();
    tResult ProcessActionCheckRightAndLeft();

    tResult ProcessCheckDoll();


    tFloat32 CalcAngelPoolar(tFloat32 x, tFloat32 y);
    tFloat32 IsObjectInBox(UltrasonicACC::roiXY roi_XY, UltrasonicACC::LidarObstacles LidarObstacles);
    tUInt32 IsObjectInROI(UltrasonicACC::roiXY roi_XY, UltrasonicACC::LidarObstacles);
    UltrasonicACC::roiXY CalcXYMinAndMax(tFloat32 x_bottom_left_corner, tFloat32 y_bottom_left_corner, tFloat32 x_height, tFloat32 y_width);
    UltrasonicACC::occupancy CheckROIforObstacles(UltrasonicACC::roiXY roi_XY1);
    UltrasonicACC::occupancy CheckROIforObstacles(UltrasonicACC::roiXY roi_XY1, UltrasonicACC::roiXY roi_XY2);
    UltrasonicACC::occupancy CheckROIforObstacles(UltrasonicACC::roiXY roi_XY1, UltrasonicACC::roiXY roi_XY2, UltrasonicACC::roiXY roi_XY3);
    tResult ProcessROI(UltrasonicACC::roiXY roi_XY1);
    tResult ProcessROI(UltrasonicACC::roiXY roi_XY1, UltrasonicACC::roiXY roi_XY2);
    tResult ProcessROI(UltrasonicACC::roiXY roi_XY1, UltrasonicACC::roiXY roi_XY2, UltrasonicACC::roiXY roi_XY3);

    UltrasonicACC::pointOnTwoLanes ProcessLaneDetectionInput(TLaneDetectionLineStruct::Data lineStruct);

    tResult LoadProperties();

	tFloat32 CalcDistanceWithCos(tFloat32 a, tFloat32 b, tFloat32 gammaInDeg);

    tResult SetMergeFlag(tBool flag);
    tBool GetMergeFlag();
    //Getter and setter for global counter
    tResult IncrementGlobalCounter();
    tInt64 GetGlobalCounter();

    tResult TransmitRelevantLaser(TLaserScannerData::Data laserToTransmit);
    tFloat32 AngleBackCompensation(tFloat32 angle);
    //UltrasonicACC::xyPoints TransformCamToMM(tInt32 x_Cam, tInt32 y_Cam);

    // setTypeFromMat
    void setTypeFromMat(adtf::streaming::cSampleWriter& writer, const cv::Mat& outputImage, tBool keepFormat = false);

    // writeMatToPin
    void writeMatToPin(adtf::streaming::cSampleWriter& writer,
                       const cv::Mat& outputImage, tTimeStamp streamTime);

    // ChangeType
    tResult ChangeType(adtf::streaming::cDynamicSampleReader& inputPin,
        const adtf::streaming::ant::IStreamType& oType);

    // ----- IMAGE PROCESSING
    // mat for video processing
    Mat m_MatInput;
    Mat m_MatOutput;
    Mat m_MatOutputDebug;
    Mat m_MatRoi;
    Point2f m_p2fROIVertices[4];
    Point2f m_p2fTransformedImageVertices[4];

    void setRoiOfImage();
    void transformToBirdsEyeView();

}; // UltrasonicACC
