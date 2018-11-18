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
*From the documentation of 2016:
* Hinderniserkennung und Abstandsregelung mittels Ultraschallsensoren und Lidar
* Der Filter Ultrasonic_ACC prüft die Ultraschallsensoren auf Hindernisse. Abhängig vom Lenkwinkel und der Bewegungsrichtung werden unterschiedliche Sensoren
berücksichtigt. Anhand des Abstandes zum nächstgelegenen Hindernis wird die Geschwindigkeit begrenzt (einstellbar über eine XML-Datei). Wenn das Fahrzeug
vor einem statischen Hindernis zum Stehen gekommen ist, wird dies an die State Machine gemeldet
We combined the function of obstacel detection and UltrasonicACC and used the laser scanner (lidar) as the main used sensor
* s

* $Adapted by:: Xiangfei#  $Date:: 2018-08-01 12:44:00#
* Improved by Illmer status: done


**********************************************************************/

#include <mutex>
#include "stdafx.h"
#include "UltrasonicACC.h"
#include "ScmCommunication.h"
#include <stdlib.h>
#include <math.h>
#include "tinyxml2.h"
#include <fstream>
#include <property_structs.h>


#define USACC_CONFIG_DRIVING_FILE "Interpolation/Driving/Configuration File For Driving Mode ACC"
#define USACC_CONFIG_MtP_FILE "Interpolation/MtP/Configuration File For MtP Mode ACC"
#define USACC_CONFIG_BEND_FILE "Interpolation/Bend/Configuration File For Bend Mode ACC"
#define USACC_DEBUG_XML_BORDER_WARNING "Debug/XML Border Warnings to Console"
#define USACC_DEBUG_PRINT_INITIAL_TABLE "Debug/Print initial tables to Console"
#define USACC_DEBUG_OUTPUT_TO_CONSOLE "Debug/Debug Output to Console"
#define USACC_EXT_DEBUG_OUTPUT_TO_CONSOLE "Debug/Extended Debug Output to Console"
#define USACC_COUNTER_THRESHOLD_NOMOVE "Movement-related parameters/Counter/no movement threshold"
#define USACC_COUNTER_THRESHOLD_MOVEAGAIN "Movement-related parameters/Counter/moving again threshold"
#define USACC_LOWERBOUND_SPEED "Movement-related parameters/Lower Bound/speed seen as 'no movement'"
#define USACC_SENSOR_SCALING_FRONT "Sensors/Front US scaling factor for sensor-weighting"
#define USACC_SENSOR_SCALING_REAR "Sensors/Rear US scaling factor for sensor-weighting"
#define USACC_SENSOR_FRONT_CHECK_LIMIT "Sensors/Front Sensors Check Limit"
#define USACC_SET_ULRASONIC_VALUES_MAX "Deactivates ultrasonic acc by setting all ultrasonic values to max"
#define USACC_Object_Detection_LIDAR_THRESHOLD_ANGLE "Object Detection/Threshold max angle difference two values are considered as succeed"
#define USACC_Object_Detection_LIDAR_THRESHOLD_RADIUS "Object Detection/Threshold for which max radius difference two values are considered as part of one object"
#define USACC_Object_Detection_LIDAR_THRESHOLD_DISTANCE "Object Detection/thtreshold for the used lidar data"
#define USACC_SCALE_STEERING "Movement-related parameters/Steering * Scale_Steering = Angle of the wheel (trajectory)"


//not used
//from obstacel detection
/* Regions of interest */
/* INTERSECTION */
/* Region for oncoming traffic at intersection*/
#define USACC_ROI_INTERSECTION_ONCOMING_BOTTOMLEFT_X 0.9f//1.10f
#define USACC_ROI_INTERSECTION_ONCOMING_BOTTOMLEFT_Y 0.6f
#define USACC_ROI_INTERSECTION_ONCOMING_HEIGHT_X 1.0f//0.6f		// magnitude expresses height, sign expresses coordinate direction (positive ->straight ahead)
#define USACC_ROI_INTERSECTION_ONCOMING_WIDTH_Y -0.3f		// magnitude expresses width, sign expresses coordinate direction (positive -> to left hand side)

/* Region for cross-traffic from right hand side at intersection */
#define USACC_ROI_INTERSECTION_CROSSTRAFFIC_BOTTOMLEFT_X 0.6f
#define USACC_ROI_INTERSECTION_CROSSTRAFFIC_BOTTOMLEFT_Y -0.2f
#define USACC_ROI_INTERSECTION_CROSSTRAFFIC_HEIGHT_X 0.35f	// magnitude expresses height, sign expresses coordinate direction (positive ->straight ahead)
#define USACC_ROI_INTERSECTION_CROSSTRAFFIC_WIDTH_Y -0.4f	// magnitude expresses width, sign expresses coordinate direction (positive -> to left hand side)

/* Region for cross-traffic from left hand side at intersection */
#define USACC_ROI_INTERSECTION_CROSSTRAFFIC_LEFT_BOTTOMLEFT_X 0.55f
#define USACC_ROI_INTERSECTION_CROSSTRAFFIC_LEFT_BOTTOMLEFT_Y 0.7f
#define USACC_ROI_INTERSECTION_CROSSTRAFFIC_LEFT_HEIGHT_X 0.35f	// magnitude expresses height, sign expresses coordinate direction (positive ->straight ahead)
#define USACC_ROI_INTERSECTION_CROSSTRAFFIC_LEFT_WIDTH_Y 0.4f	// magnitude expresses width, sign expresses coordinate direction (positive -> to left hand side)

/* OVERTAKING */
/* Region for oncoming traffic during drive-mode */
#define USACC_ROI_OVERTAKE_ONCOMING_BOTTOMLEFT_X 0.3f
#define USACC_ROI_OVERTAKE_ONCOMING_BOTTOMLEFT_Y 0.55f
#define USACC_ROI_OVERTAKE_ONCOMING_HEIGHT_X 0.9f	// magnitude expresses height, sign expresses coordinate direction (positive ->straight ahead)
#define USACC_ROI_OVERTAKE_ONCOMING_WIDTH_Y -0.3f	// magnitude expresses width, sign expresses coordinate direction (positive -> to left hand side)

/* Region for traffic on original lane during overtake, right before going back into original lane */
#define USACC_ROI_OVERTAKE_ORIGINALLANE_BOTTOMLEFT_X 0.3f
#define USACC_ROI_OVERTAKE_ORIGINALLANE_BOTTOMLEFT_Y -0.27f
#define USACC_ROI_OVERTAKE_ORIGINALLANE_HEIGHT_X 0.9f	// magnitude expresses height, sign expresses coordinate direction (positive ->straight ahead)
#define USACC_ROI_OVERTAKE_ORIGINALLANE_WIDTH_Y -0.3f	// magnitude expresses width, sign expresses coordinate direction (positive -> to left hand side)

/* DETECTION OF STATIC OBSTACLE IN FRONT ON STRAIGHT LANE */
/* Region for traffic ahead on own current lane IN STRAIGHT DRIVING, AC_OD_OVERTAKE_CHECK_OWN_LANE_STRAIGHT*/
#define USACC_ROI_OVERTAKE_OBSTACLESTRAIGHT_BOTTOMLEFT_X 0.05f
#define USACC_ROI_OVERTAKE_OBSTACLESTRAIGHT_BOTTOMLEFT_Y 0.15f
#define USACC_ROI_OVERTAKE_OBSTACLESTRAIGHT_HEIGHT_X 0.9f	// magnitude expresses height, sign expresses coordinate direction (positive ->straight ahead)
#define USACC_ROI_OVERTAKE_OBSTACLESTRAIGHT_WIDTH_Y -0.3f	// magnitude expresses width, sign expresses coordinate direction (positive -> to left hand side)

/* DETECTION OF STATIC OBSTACLE IN FRONT LEFT HALF ON STRAIGHT LANE */
/* Region for traffic ahead on own current lane IN STRAIGHT DRIVING, AC_OD_OVERTAKE_CHECK_OWN_LANE_STRAIGHT_LEFTHALF*/
#define USACC_ROI_OVERTAKE_OBSTACLESTRAIGHT_LEFTHALF_BOTTOMLEFT_X 0.05f
#define USACC_ROI_OVERTAKE_OBSTACLESTRAIGHT_LEFTHALF_BOTTOMLEFT_Y 0.3f
#define USACC_ROI_OVERTAKE_OBSTACLESTRAIGHT_LEFTHALF_HEIGHT_X 0.9f	// magnitude expresses height, sign expresses coordinate direction (positive ->straight ahead)
#define USACC_ROI_OVERTAKE_OBSTACLESTRAIGHT_LEFTHALF_WIDTH_Y -0.2f	// magnitude expresses width, sign expresses coordinate direction (positive -> to left hand side)



/* REGIONS RIGHT BEFORE PARKING OR PULL OUT MANEUVERS */
/* Region for traffic on oncoming lane that needs to be checked before starting cross-parking maneuver */
#define USACC_ROI_PARKING_CROSS_ONCOMINGLANE_BOTTOMLEFT_X 0.1f
#define USACC_ROI_PARKING_CROSS_ONCOMINGLANE_BOTTOMLEFT_Y 0.45f
#define USACC_ROI_PARKING_CROSS_ONCOMINGLANE_HEIGHT_X 0.7f	// magnitude expresses height, sign expresses coordinate direction (positive ->straight ahead)
#define USACC_ROI_PARKING_CROSS_ONCOMINGLANE_WIDTH_Y -0.35f	// magnitude expresses width, sign expresses coordinate direction (positive -> to left hand side)

/* Region for traffic on lane in front that needs to be checked before starting pull out from cross-parking maneuver */
//does nmot distinguish between pull out right or left
#define USACC_ROI_PARKING_CROSS_CROSSTRAFFIC_RIGHT_BOTTOMLEFT_X 0.1f
#define USACC_ROI_PARKING_CROSS_CROSSTRAFFIC_RIGHT_BOTTOMLEFT_Y 0.7f
#define USACC_ROI_PARKING_CROSS_CROSSTRAFFIC_RIGHT_HEIGHT_X 0.65f	// magnitude expresses height, sign expresses coordinate direction (positive ->straight ahead)
#define USACC_ROI_PARKING_CROSS_CROSSTRAFFIC_RIGHT_WIDTH_Y -1.4f	// magnitude expresses width, sign expresses coordinate direction (positive -> to left hand side)


//end from obstacel detection

/*Merge into lane*/






/* sensor angles */
//#define USACC_SENSOR_FRONT_LEFT -100
//#define USACC_SENSOR_FRONT_CENTERLEFT -50
//#define USACC_SENSOR_FRONT_CENTER 0
//#define USACC_SENSOR_FRONT_CENTERRIGHT 50
//#define USACC_SENSOR_FRONT_RIGHT 100
#define USACC_SENSOR_REAR_LEFT -100
#define USACC_SENSOR_REAR_CENTER 0
#define USACC_SENSOR_REAR_RIGHT +100

#define USfrequency 40

#define PI 3.1415


std::mutex m;
//sort by angle function
static int compareAngle(const void * l1, const void *l2)
{
    TPolarCoordiante::Data *pc1 = (TPolarCoordiante::Data *) l1;
    TPolarCoordiante::Data *pc2 = (TPolarCoordiante::Data *) l2;
    return pc1->f32Angle < pc2->f32Angle;

}



// This will define the filter and expose it via plugin class factory.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
        CID_ULTRASONIC_ACC_FILTER,                         // references to header file
        "UltrasonicACC",                                   // label
        UltrasonicACC,                                      // class
        //adtf::filter::pin_trigger({"ultrasonicInput", "actionInput", "targetSpeedInput", "originalTargetSpeedInput", "steeringAngleInput", "laserscannerIn"}));	// set trigger pin

        //adtf::filter::pin_trigger({"laserscannerIn"}));
        adtf::filter::timer_trigger(200000));
//adtf::filter::pin_trigger({"ultrasonicInput", "actionInput", "targetSpeedInput", "steeringAngleInput", "laserscannerIn", "laneDetectionLine"}));	// set trigger pin
//adtf::filter::pin_trigger({ "inputVideo"}));

// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
UltrasonicACC::UltrasonicACC()
{

    // ------------------------------------------
    SetName("UltrasonicACC Constructor");

    // -----------------------------------------
    // set pins
    o_UltrasonicStruct.registerPin(this, m_ReaderUltrasonic         , "ultrasonicInput"                 );
    o_ActionStruct.registerPin(this, m_ReaderAction                 , "actionInput"                     );
    o_TSignalValue.registerPin(this, m_ReaderTargetSpeed            , "targetSpeedInput"                );
    o_TSignalValue.registerPin(this, m_ReaderSteeringAngle          , "steeringAngleInput"              );
    o_LaserScanner.registerPin(this, m_ReaderLaserScanner           , "laserscannerIn"                  );
    o_LaneDetectionLineStruct.registerPin(this, m_ReaderLaneDetectionLine             , "laneDetectionLine"                  );

    o_TSignalValue.registerPin(this, m_WriterTargetSpeed            , "targetSpeedOutput"               );
    o_FeedbackStruct.registerPin(this, m_WriterFeedback             , "feedbackOutput"                  );
    o_LaserScanner.registerPin(this, m_WriterRelevantLaser          , "laserOut"                        );
    o_TSignalValue.registerPin(this, m_WriterRampGuardrail          , "GuardrailPosition"               );    //m_WriterRampGuardrail

    // --------- VIDEO PINS ----------
    //create and set inital input format type
    m_sInputImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    adtf::ucom::object_ptr<IStreamType> pVideoDataType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pVideoDataType, m_sInputImageFormat);

    // set output type
    adtf::ucom::object_ptr<IStreamType> pTypeOutput = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pTypeOutput, m_sOutputImageFormat);

    // Register input pin
    Register(m_ReaderVideo, "inputVideo" , pVideoDataType);
    // Register output pin
    Register(m_WriterVideo     , "outputVideo"        , pTypeOutput);
    Register(m_WriterVideoDebug, "outputVideoImageRoi", pTypeOutput);


    //register callback for type changes
    m_ReaderVideo.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pVideoDataType) -> tResult
    {
        return ChangeType(m_ReaderVideo, *pVideoDataType.Get());
    });

    // -----------------------------------------
    // set property variables
    RegisterPropertyVariable(USACC_CONFIG_DRIVING_FILE              , m_propConfigFileDriving           );
    RegisterPropertyVariable(USACC_CONFIG_MtP_FILE              , m_propConfigFileMtP               );
    RegisterPropertyVariable(USACC_CONFIG_BEND_FILE              , m_propConfigFileBend           );
    RegisterPropertyVariable(USACC_DEBUG_XML_BORDER_WARNING         , m_propBDebugXMLBorderWarning       );
    RegisterPropertyVariable(USACC_SET_ULRASONIC_VALUES_MAX         , m_propBSetUSValuesMax              );
    RegisterPropertyVariable(USACC_DEBUG_PRINT_INITIAL_TABLE        , m_propBDebugPrintInitialTable      );
    RegisterPropertyVariable(USACC_COUNTER_THRESHOLD_NOMOVE         , m_propI32CounterThresholdNoMove      );
    RegisterPropertyVariable(USACC_COUNTER_THRESHOLD_MOVEAGAIN      , m_propI32CounterThresholdMoveAgain   );
    RegisterPropertyVariable(USACC_LOWERBOUND_SPEED                 , m_propF32LowerBoundSpeed             );
    RegisterPropertyVariable(USACC_SENSOR_SCALING_FRONT             , m_propF32SensorScalingFront          );
    RegisterPropertyVariable(USACC_SENSOR_SCALING_REAR              , m_propF32SensorScalingRear           );
    RegisterPropertyVariable(USACC_SENSOR_FRONT_CHECK_LIMIT         , m_propF32SensorFrontCheckLimit       );
    RegisterPropertyVariable(USACC_DEBUG_OUTPUT_TO_CONSOLE          , m_propBDebugOutputToConsole        );
    RegisterPropertyVariable(USACC_EXT_DEBUG_OUTPUT_TO_CONSOLE      , m_propBExtDebugOutputToConsole     );

    RegisterPropertyVariable(USACC_Object_Detection_LIDAR_THRESHOLD_ANGLE     , m_propF32ObjectThresholdAngle     );
    RegisterPropertyVariable(USACC_Object_Detection_LIDAR_THRESHOLD_RADIUS     , m_propF32ObjectThresholdRadius     );
    RegisterPropertyVariable(USACC_Object_Detection_LIDAR_THRESHOLD_DISTANCE    , m_propF32ObjectThresholdDetection     );
    RegisterPropertyVariable(USACC_SCALE_STEERING    , m_propF32ScaleSteering    );
    RegisterPropertyVariable("Debug/ Info for the first start", m_propBDebugFirstStart);
    RegisterPropertyVariable("Movement-related parameters/Angle close to the car", m_propF32CloseObjectThresholdAngle);
    RegisterPropertyVariable("Offset/Steering", m_propF32SensorOffsetSteering);
    RegisterPropertyVariable("Threshold/ Below considered as a straight", m_propF32ThresholdStraight);
    RegisterPropertyVariable("Threshold/ Visual Range in a bend (used)", m_propF32ThresholdVisualRangeBend);//m_propF32ThresholdVisualRangeBend
    RegisterPropertyVariable("Offset/ between the pos of the cam in 2017 and the lidar", m_propF32OffsetCamLidar);
    RegisterPropertyVariable("Debug/ Info for the roi checker", m_propBDebugROI);
    RegisterPropertyVariable("Debug/ for driving straight", m_propBDebugStraight);
    RegisterPropertyVariable("Const/Wheelbase of the car", m_propF32wheelbase);
    RegisterPropertyVariable("Const/Distance of a circular segment", m_propF32circSegmentDistance);
    RegisterPropertyVariable("Const/Distance between front axis and front bumper-end in mm", m_propF32DistanceAxisToBumper);
    RegisterPropertyVariable("Boundery/Free ROI Boundary", m_propUI32FreeROIBoundary); //it will be considerated as free
    RegisterPropertyVariable("Boundery/Occupied ROI Boundary", m_propUI32OccupiedROIBoundary); //it will be considerated as occupied//m_propUI32OccupiedROIBoundary
    RegisterPropertyVariable("Boundery/Temporary Occupied ROI Boundary", m_propUI32TemporaryOccupiedROIBoundary);//m_propUI32TemporaryOccupiedROIBoundary
    RegisterPropertyVariable("Boundery/End of for loop to check roi", m_propUI32EndOfForLoopToCheckROI);
    RegisterPropertyVariable("Debug/ Curve enable", m_propBDebugCurve);//m_propBDebugCurve
    RegisterPropertyVariable("Debug/Action command enable enable", m_propBDebugActionCommand);    //m_propBDebugActionCommand
    RegisterPropertyVariable("Parkour/Distance from the stop to stopline [mm]", m_propF32DistanceToStopLine);
    RegisterPropertyVariable("Threshold/High [y-axis] of a box in the bend [mm]", m_propF32HighBoxBend);    //m_propF32HighBox
    RegisterPropertyVariable("Threshold/for relevant obstacles in a bend [mm]", m_propF32ThresholdRelObstBend);    //m_propF32ThresholdRelObstBend
    RegisterPropertyVariable("Threshold/ for slope too high [mm]", m_propF32ThresholdSlopeTooHigh);
    RegisterPropertyVariable("Threshold/ reduced visual range one side [mm]", m_propF32ThresholdReducedVisualRangeOneSide);//
    RegisterPropertyVariable("Debug/ Info Overtaking ROI", m_propBDebugOvertakingROI);//m_propBDebugOvertakingROI
    RegisterPropertyVariable("Car/Scale Pixel to mm", m_propF32PixToMM);//m_propF32PixToMM
    RegisterPropertyVariable("Car/Offset cam to lidar in y", m_propF32CamOffsety);//m_propF32CamOffsety
    RegisterPropertyVariable("Const/Car width mm", m_propF32CarWidth);
    RegisterPropertyVariable("Debug/True: driving; false ROI", m_propBLaserOutputDrive);
    RegisterPropertyVariable("Offset/ Parking Y", m_propF32OffsetParkingY);
    RegisterPropertyVariable("Debug/ Ramp", m_propBDebugRamp);
    RegisterPropertyVariable("Threshold/ Ramp /Min Number of points", m_propui32MinNumOfPointsRamp);
    RegisterPropertyVariable("Threshold/ Ramp /Differenz Y Not On Line", m_propF32ThresholdDifferenzYNotOnLine);//
    RegisterPropertyVariable("Threshold/ Ramp / Xmin of the ramp", m_propF32XminOfRamp);//;
    RegisterPropertyVariable("Threshold/ Ramp / Xmax of the ramp", m_propF32XmaxOfRamp);//m_propF32XmaxOfRamp;
    RegisterPropertyVariable("Threshold/ Ramp / Min length of the ramp", m_propF32MinLengthOfTheRamp);//m_propF32MinLengthOfTheRamp
    RegisterPropertyVariable("Threshold/ Ramp / Max length of the ramp", m_propF32MaxLengthOfTheRamp);//m_propF32MaxLengthOfTheRamp
    RegisterPropertyVariable("Threshold/ Ramp / Num of samples of lane detetction", m_propui32NumberLaneDetectionInputForRamp);//
    RegisterPropertyVariable("Threshold/ Ramp / Diff y still ramp", m_propF32YDiffRamp);//
    RegisterPropertyVariable("Threshold/ Ramp / NumberNo Guardrail for off ramp", m_propui32NumNoGuardrailForOffRamp);//m_propui32NumNoGuardrailForOffRamp
    RegisterPropertyVariable("Threshold/ Ramp / Driving Y", m_propf32ThresholdRampDrivingY);
    RegisterPropertyVariable("Threshold/ Ramp / Driving X", m_propf32ThresholdRampDrivingX);

    RegisterPropertyVariable("Debug/ LaserOutput", m_propBLaserOutputGeneral);//m_propBLaserOutputGeneral
    RegisterPropertyVariable("Threshold/ Doll /Another object", m_propF32ThresholdDollAnotherObject);//m_propF32ThresholdDollAnotherObject
    RegisterPropertyVariable("Debug/ Doll", m_propBDebugDoll);//m_propBLaserOutputGeneral
    //m_propBDebugDoll
    RegisterPropertyVariable("Distance/ Normal", m_propF32DistanceToStopNormal);
    RegisterPropertyVariable("Distance/ After right", m_propF32DistanceToStopAfterRight);
    RegisterPropertyVariable("Distance/ After left", m_propF32DistanceToStopAfterLeft);

    RegisterPropertyVariable("ROI/Intersection/Oncoming/ x_min", m_propF32IntersectionOncomingXmin);
    RegisterPropertyVariable("ROI/Intersection/Oncoming/ y_min", m_propF32IntersectionOncomingYmin);
    RegisterPropertyVariable("ROI/Intersection/Oncoming/ x_max", m_propF32IntersectionOncomingXmax);
    RegisterPropertyVariable("ROI/Intersection/Oncoming/ y_max", m_propF32IntersectionOncomingYmax);

    RegisterPropertyVariable("ROI/Intersection/Crosstraffic right/ x_min", m_propF32IntersectionCrosstrafficRightXmin);
    RegisterPropertyVariable("ROI/Intersection/Crosstraffic right/ y_min", m_propF32IntersectionCrosstrafficRightYmin);
    RegisterPropertyVariable("ROI/Intersection/Crosstraffic right/ x_max", m_propF32IntersectionCrosstrafficRightXmax);
    RegisterPropertyVariable("ROI/Intersection/Crosstraffic right/ y_max", m_propF32IntersectionCrosstrafficRightYmax);

    RegisterPropertyVariable("ROI/Intersection/Crosstraffic left/ x_min", m_propF32IntersectionCrosstrafficLeftXmin);
    RegisterPropertyVariable("ROI/Intersection/Crosstraffic left/ y_min", m_propF32IntersectionCrosstrafficLeftYmin);
    RegisterPropertyVariable("ROI/Intersection/Crosstraffic left/ x_max", m_propF32IntersectionCrosstrafficLeftXmax);
    RegisterPropertyVariable("ROI/Intersection/Crosstraffic left/ y_max", m_propF32IntersectionCrosstrafficLeftYmax);
    /* Region for oncoming traffic during drive-mode */
    RegisterPropertyVariable("ROI/Overtake/Oncoming/ x_min", m_propF32OvertakeOncomingXmin);
    RegisterPropertyVariable("ROI/Overtake/Oncoming/ y_min", m_propF32OvertakeOncomingYmin);
    RegisterPropertyVariable("ROI/Overtake/Oncoming/ x_max", m_propF32OvertakeOncomingXmax);
    RegisterPropertyVariable("ROI/Overtake/Oncoming/ y_max", m_propF32OvertakeOncomingYmax);
    /* Region for traffic on original lane during overtake, right before going back into original lane */
    RegisterPropertyVariable("ROI/Overtake/Originallane/ x_min", m_propF32OvertakeOriginallaneXmin);
    RegisterPropertyVariable("ROI/Overtake/Originallane/ y_min", m_propF32OvertakeOriginallaneYmin);
    RegisterPropertyVariable("ROI/Overtake/Originallane/ x_max", m_propF32OvertakeOriginallaneXmax);
    RegisterPropertyVariable("ROI/Overtake/Originallane/ y_max", m_propF32OvertakeOriginallaneYmax);
    /* DETECTION OF STATIC OBSTACLE IN FRONT ON STRAIGHT LANE */
    /* Region for traffic ahead on own current lane IN STRAIGHT DRIVING, AC_OD_OVERTAKE_CHECK_OWN_LANE_STRAIGHT*/
    RegisterPropertyVariable("ROI/Overtake/Obstaclestraight/ x_min", m_propF32OvertakeObstacleStraightXmin);
    RegisterPropertyVariable("ROI/Overtake/Obstaclestraight/ y_min", m_propF32OvertakeObstacleStraightYmin);
    RegisterPropertyVariable("ROI/Overtake/Obstaclestraight/ x_max", m_propF32OvertakeObstacleStraightXmax);
    RegisterPropertyVariable("ROI/Overtake/Obstaclestraight/ y_max", m_propF32OvertakeObstacleStraightYmax);
    /* DETECTION OF STATIC OBSTACLE IN FRONT LEFT HALF ON STRAIGHT LANE */
    /* Region for traffic ahead on own current lane IN STRAIGHT DRIVING, AC_OD_OVERTAKE_CHECK_OWN_LANE_STRAIGHT_LEFTHALF*/
    RegisterPropertyVariable("ROI/Overtake/Obstaclestraight_Lefthalf/ x_min", m_propF32OvertakeObstacleStraightLeftHalfXmin);
    RegisterPropertyVariable("ROI/Overtake/Obstaclestraight_Lefthalf/ y_min", m_propF32OvertakeObstacleStraightLeftHalfYmin);
    RegisterPropertyVariable("ROI/Overtake/Obstaclestraight_Lefthalf/ x_max", m_propF32OvertakeObstacleStraightLeftHalfXmax);
    RegisterPropertyVariable("ROI/Overtake/Obstaclestraight_Lefthalf/ y_max", m_propF32OvertakeObstacleStraightLeftHalfYmax);
    /* REGIONS RIGHT BEFORE PARKING OR PULL OUT MANEUVERS */
    /* Region for traffic on oncoming lane that needs to be checked before starting cross-parking maneuver */
    RegisterPropertyVariable("ROI/Parking/Cross_oncomingLane/ x_min", m_propF32ParkingCrossOncomingLaneXmin);
    RegisterPropertyVariable("ROI/Parking/Cross_oncomingLane/ y_min", m_propF32ParkingCrossOncomingLaneYmin);
    RegisterPropertyVariable("ROI/Parking/Cross_oncomingLane/ x_max", m_propF32ParkingCrossOncomingLaneXmax);
    RegisterPropertyVariable("ROI/Parking/Cross_oncomingLane/ y_max", m_propF32ParkingCrossOncomingLaneYmax);
    /* Region for traffic on lane in front that needs to be checked before starting pull out from cross-parking maneuver */
    /*On the first line*/
    RegisterPropertyVariable("ROI/Parking/PullOutCross_Oncoming1/ x_min", m_propF32ParkingPullOutCrossOncoming1Xmin);
    RegisterPropertyVariable("ROI/Parking/PullOutCross_Oncoming1/ y_min", m_propF32ParkingPullOutCrossOncoming1Ymin);
    RegisterPropertyVariable("ROI/Parking/PullOutCross_Oncoming1/ x_max", m_propF32ParkingPullOutCrossOncoming1Xmax);
    RegisterPropertyVariable("ROI/Parking/PullOutCross_Oncoming1/ y_max", m_propF32ParkingPullOutCrossOncoming1Ymax);
    /*On the second line*/
    RegisterPropertyVariable("ROI/Parking/PullOutCross_Oncoming2/ x_min", m_propF32ParkingPullOutCrossOncoming2Xmin);
    RegisterPropertyVariable("ROI/Parking/PullOutCross_Oncoming2/ y_min", m_propF32ParkingPullOutCrossOncoming2Ymin);
    RegisterPropertyVariable("ROI/Parking/PullOutCross_Oncoming2/ x_max", m_propF32ParkingPullOutCrossOncoming2Xmax);
    RegisterPropertyVariable("ROI/Parking/PullOutCross_Oncoming2/ y_max", m_propF32ParkingPullOutCrossOncoming2Ymax);

    /*Crosswalk*/
    RegisterPropertyVariable("ROI/Crosswalk/ x_min", m_propF32CheckCrosswalkXmin);
    RegisterPropertyVariable("ROI/Crosswalk/ x_max", m_propF32CheckCrosswalkXmax);
    RegisterPropertyVariable("ROI/Crosswalk/ y_min", m_propF32CheckCrosswalkYmin);
    RegisterPropertyVariable("ROI/Crosswalk/ y_max", m_propF32CheckCrosswalkYmax);

    /*chech target lane for merge*/
    RegisterPropertyVariable("ROI/Merge/ Target Lane Xmin", m_propF32MergeCheckTargetLaneXmin);//m_propF32MergeCheckTargetLaneXmin
    RegisterPropertyVariable("ROI/Merge/ Target Lane Xmax", m_propF32MergeCheckTargetLaneXmax);
    RegisterPropertyVariable("ROI/Merge/ Target Lane Ymin", m_propF32MergeCheckTargetLaneYmin);
    RegisterPropertyVariable("ROI/Merge/ Target Lane Ymax", m_propF32MergeCheckTargetLaneYmax);

    /*Check for barbie doll*/
    RegisterPropertyVariable("ROI/Doll/ Xmin", m_propF32ROIDollXmin);
    RegisterPropertyVariable("ROI/Doll/ Ymin", m_propF32ROIDollYmin);
    RegisterPropertyVariable("ROI/Doll/ Xmax", m_propF32ROIDollXmax);
    RegisterPropertyVariable("ROI/Doll/ Ymax", m_propF32ROIDollYmax);
    RegisterPropertyVariable("Const/Doll width", m_propF32DollWidth);


    // ---- IMAGE PROCESSING
    RegisterPropertyVariable("ImageProcessing/ROI/Point1/X", m_propROIPoint1X); // upper leftUInt32
    RegisterPropertyVariable("ImageProcessing/ROI/Point1/Y", m_propROIPoint1Y);
    RegisterPropertyVariable("ImageProcessing/ROI/Point2/X", m_propROIPoint2X); // upper right
    RegisterPropertyVariable("ImageProcessing/ROI/Point2/Y", m_propROIPoint2Y);
    RegisterPropertyVariable("ImageProcessing/ROI/Point3/X", m_propROIPoint3X); // lower right
    RegisterPropertyVariable("ImageProcessing/ROI/Point3/Y", m_propROIPoint3Y);
    RegisterPropertyVariable("ImageProcessing/ROI/Point4/X", m_propROIPoint4X); // lower left
    RegisterPropertyVariable("ImageProcessing/ROI/Point4/Y", m_propROIPoint4Y);
    RegisterPropertyVariable("ImageProcessing/Frames", m_propUI32Frames);
    RegisterPropertyVariable("ImageProcessing/TopDownWidth" , m_propTransformedImageWidth);
    RegisterPropertyVariable("ImageProcessing/TopDownHeight", m_propTransformedImageHeight);


    //checks if obstacle is only on the right hand side
    RegisterPropertyVariable("ROI/Overtake/ Only Right hand side/ x_min", m_propF32OvertakeObstacleOnlyRightHandSideXmin);
    RegisterPropertyVariable("ROI/Overtake/ Only Right hand side/ x_max", m_propF32OvertakeObstacleOnlyRightHandSideXmax);
    RegisterPropertyVariable("ROI/Overtake/ Only Right hand side/ y_min", m_propF32OvertakeObstacleOnlyRightHandSideYmin);
    RegisterPropertyVariable("ROI/Overtake/ Only Right hand side/ y_max", m_propF32OvertakeObstacleOnlyRightHandSideYmax);
    //from ConfigureUInt32



    LoadProperties();
    //initialization
    m_I32TimeoutCounter = 0;

    m_intersectionMode = NORMAL_DRIVE;

    m_BRunningState = tFalse;//if runningState NoMovement is active
    m_BRunningStateMoveAgain = tFalse;

    m_UI32ObstacleNoMovementCounter = 0;
    m_UI32ObstacleMovementAgainCounter = 0;
    m_bDebugModeEnabled = tTrue;
    m_bExtendedDebugModeEnabled = tFalse;

    /* initialize to neutral mid-setting, corresponds to anm_propF32YDiffRam angle of zero degree */
    m_dataSteeringAngleTemp.f32Value = 0.0f;//-m_propF32SensorOffsetSteering;//90UInt32
    // 2017:propertyChanged
    m_UI32ScalingFactorLIDAR          = m_propF32SensorScalingFront;
    m_UI32ScalingFactorUSRear         = m_propF32SensorScalingRear;
    m_f32FrontSensorsCheckLimit      = m_propF32SensorFrontCheckLimit;
    m_bDebugModeEnabled         = m_propBDebugOutputToConsole;
    m_BDeactivateACCFilter         = m_propBExtDebugOutputToConsole;
    m_UI32NoMovementThreshold         = m_propI32CounterThresholdNoMove;
    m_UI32MovementAgainThreshold      = m_propI32CounterThresholdMoveAgain;
    m_F32ModifiedSpeedLowerBound     = m_propF32LowerBoundSpeed;
    m_fileDrivingModeACC        = m_propConfigFileDriving;
    m_fileMtPModeACC        = m_propConfigFileMtP;
    m_fileBendModeACC        = m_propConfigFileBend;
    m_bExtendedDebugModeEnabled = m_propBExtDebugOutputToConsole;
    m_bPrintInitialtable        = m_propBDebugPrintInitialTable;
    m_bBorderWarningModeEnabled = m_propBDebugXMLBorderWarning;

    /* static Feedbacks to be sent*/
    m_dataFeedbackAtRest.ui8FilterId = F_ULTRASONIC_ACC; //we do not need it
    m_dataFeedbackAtRest.ui32FeedbackStatus = FB_UA_NO_MOVEMENT;
    m_dataFeedbackMovingAgain.ui8FilterId = F_ULTRASONIC_ACC;//we do not need it
    m_dataFeedbackMovingAgain.ui32FeedbackStatus = FB_UA_MOVING_AGAIN;
    m_dataFeedbackNoObstacel.ui8FilterId = F_ULTRASONIC_ACC;
    m_dataFeedbackNoObstacel.ui32FeedbackStatus = FB_UA_NO_OBSTACLE;
    m_dataFeedbackStaticObstacel.ui8FilterId = F_ULTRASONIC_ACC;
    m_dataFeedbackStaticObstacel.ui32FeedbackStatus = FB_UA_STATIC_OBSTACLE;
    m_dataFeedbackRamp.ui8FilterId = F_ULTRASONIC_ACC;
    m_dataFeedbackRamp.ui32FeedbackStatus = FB_UA_RAMP;
    m_dataFeedbackOffRamp.ui8FilterId = F_ULTRASONIC_ACC;
    m_dataFeedbackOffRamp.ui32FeedbackStatus = FB_UA_OFF_RAMP;

    /*A_I*/
    /* load xml files for linear interpolation, for defined oparational modes */
    //LoadConfigurationData(m_fileDrivingModeACC, m_vecXValuesDriving, m_vecYValuesDriving, DRIVING_MODE_ACTIVE);
    //LoadConfigurationData(m_fileParkingModeACC, m_vecXValuesMtP, m_vecYValuesMtP, PARKING_MODE_ACTIVE);
    //RETURN_IF_FAILED(LoadConfigurationData(m_fileDrivingModeACC, m_vecXValuesDriving, m_vecYValuesDrivingUInt32, DRIVING_MODE_ACTIVE));
    //RETURN_IF_FAILED(LoadConfigurationData(m_fileParkingModeACC, m_vecXValuesMtP, m_vecYValuesMtP, PARKING_MODE_ACTIVE));
    /*
    /! holds the yValues for the supporting points/
    vector<tFloat32> m_vecXValuesDriving = {0, 0.1, 0.3, 0.4, 0,5, 0,51, 5.0};
    //! holds the xValues for the supporting points//
    vector<tFloat32> m_vecYValuesDriving = {0, 0, 7, 10, 16.0, 16.0, 16.0};

    //! holds the yValues for the supporting points/
    vector<tFloat32> m_vecYValuesBend = {0,7,10,12,12,12};
    /! holds the xValues for the supporting points/
    vector<tFloat32> m_vecXValuesBend = {0, 50.0, 150.0, 250.0, 400.0, 5000.0};

    vector<tFloat32> m_vecYValuesMtP = {0, 7, 16, 16, 16, 16};
    vector<tFloat32> m_vecXValuesMtP = {0, 50.0, 150.0, 250.0, 400.0, 5000.0};*/
    /*UInt32
    // holds the yValues for the supporting points
    vector<tFloat32> m_vecXValuesDriving = {0.0, 100.0, 350.0, 550.0, 900.0, 5000.0, 5001.0};
    // holds the xValues for the supporting points
    vector<tFloat32> m_vecYValuesDriving = {0.0, 0.0, 0.0, 7.0, 10.0, 12.0, 12.0};

    // holds the yValues for the supporting points
    vector<tFloat32> m_vecYValuesBend = {0.0,0.0,0.0,7.0,10.0,10.0,12.0};
    // holds the xValues for the supporting points
    vector<tFloat32> m_vecXValuesBend = {0.0, 50.0, 250.0, 450.0, 550.0, 700.0, 5000.0};

    vector<tFloat32> m_vecYValuesMtP = {3.0, 3.0, 3.0, 3.0, 3.0, 3.0};
    vector<tFloat32> m_vecXValuesMtP = {0.0, 50.0, 150.0, 250.0, 400.0, 5000.0};*/
    /*LoadConfigurationData(m_fileDrivingModeACC, m_vecXValuesDriving, m_vecYValuesDriving, DRIVING_MODE_ACTIVE);
    LoadConfigurationData(m_fileMtPModeACC, m_vecXValuesMtP, m_vecYValuesMtP, MtP_MODE_ACTIVE);
    LoadConfigurationData(m_fileBendModeACC, m_vecXValuesBend, m_vecYValuesBend, DRIVING_MODE_CURVE);*/
    /* ACC is per default in driving mode */
    operationalMode = DRIVING_MODE_ACTIVE;
    SetOperationalMode(DRIVING_MODE_ACTIVE);
    if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("UltrasonicACC: Starting in default mode DRIVING_MODE."));
    /*
    if(IS_FAILED(LoadConfigurationData(m_fileDrivingModeACC, m_vecYValuesDriving, m_vecXValuesDriving, DRIVUInt32ING_MODE_ACTIVE)))
    {
        LOG_ERROR("USACC: constructor Error while reading the driving mode!");
    }
    if(IS_FAILED(LoadConfigurationData(m_fileBendModeACC, m_vecXValueBend, m_vecYValueBend, DRIVING_MODE_CURVE)))
    {
        LOG_ERROR("USACC: constructor Error while reading the bend mode!");
    }
    if(IS_FAILED(LoadConfigurationData(m_fileMtPModeACC, m_vecXValuesMtP, m_vecYValuesMtP, MtP_MODE_ACTIVE)))
    {
        LOG_ERROR("USACC: constructor Error while reading the MtP mode!");
    }*/

    /*for (tUIn32 i = 0; i < m_vecXValuesDriving.size(); i++)
    {
        LOG_INFO("I: %d; X: %f, Y: %f", i, m_vecXValuesDriving, m_vecYValuesDriving);
    }*/


    //get clock object
    (_runtime->GetObject(m_pClock));

    //LIDAR may be used before it is read from the stream
    //init with 0
    TLaserScannerData::Data initLaser;
    initLaser.ui32Size =1;
    initLaser.tScanArray[0].f32Radius = 0.0;
    initLaser.tScanArray[0].f32Angle = 0.0;
    SetLIDARInput(initLaser);
    UltrasonicACC::LidarObstacles initObstacles;
    SetLIDARObstaclesInput(initObstacles);

    m_dataDefaultLanes.ui32ArduinoTimestamp = 0;
    m_dataDefaultLanes.i8IsCurve  = 0;
    m_dataDefaultLanes.i32Point1X = 0;
    m_dataDefaultLanes.i32Point1Y = 0;
    m_dataDefaultLanes.i32Point2X = 0;
    m_dataDefaultLanes.i32Point2Y = 0;
    m_dataDefaultLanes.i32Point3X = 0;
    m_dataDefaultLanes.i32Point3Y = 0;
    m_dataDefaultLanes.i32Point4X = 0;
    m_dataDefaultLanes.i32Point4Y = 0;
    SetLastLaneDetectionLine(m_dataDefaultLanes);
    m_dataLastProcessedPoints.f32x1line1 = 0.0;
    m_dataLastProcessedPoints.f32y1line1 = 0.0;
    m_dataLastProcessedPoints.f32x2line1 = 0.0;
    m_dataLastProcessedPoints.f32y2line1 = 0.0;
    m_dataLastProcessedPoints.f32x1line2 = 0.0;
    m_dataLastProcessedPoints.f32y1line2 = 0.0;
    m_dataLastProcessedPoints.f32x2line2 = 0.0;
    m_dataLastProcessedPoints.f32y2line2 = 0.0;



    //-600; -300; 900; 1900
    roiXYIntersectionOncoming.f32x_min = m_propF32IntersectionOncomingXmin;
    roiXYIntersectionOncoming.f32x_max = m_propF32IntersectionOncomingXmax;
    roiXYIntersectionOncoming.f32y_min = m_propF32IntersectionOncomingYmin;
    roiXYIntersectionOncoming.f32y_max = m_propF32IntersectionOncomingYmax;
    /*roiXYIntersectionOncoming = CalcXYMinAndMax((tFloat32(USACC_ROI_INTERSECTION_ONCOMING_BOTTOMLEFT_X)+m_propF32DistanceToStopLine), tFloat32(USACC_ROI_INTERSECTION_ONCOMING_BOTTOMLEFT_Y), tFloat32(USACC_ROI_INTERSECTION_ONCOMING_HEIGHT_X), tFloat32(USACC_ROI_INTERSECTION_ONCOMING_WIDTH_Y));*/
    //200 600 600 950
    roiXYIntersectionCrosstraffic.f32x_min = m_propF32IntersectionCrosstrafficRightXmin;
    roiXYIntersectionCrosstraffic.f32x_max = m_propF32IntersectionCrosstrafficRightXmax;
    roiXYIntersectionCrosstraffic.f32y_min = m_propF32IntersectionCrosstrafficRightYmin;
    roiXYIntersectionCrosstraffic.f32y_max = m_propF32IntersectionCrosstrafficRightYmax;
    //roiXYIntersectionCrosstraffic = CalcXYMinAndMax((tFloat32(USACC_ROI_INTERSECTION_CROSSTRAFFIC_BOTTOMLEFT_X)+m_propF32DistanceToStopLine), tFloat32(USACC_ROI_INTERSECTION_CROSSTRAFFIC_BOTTOMLEFT_Y), tFloat32(USACC_ROI_INTERSECTION_CROSSTRAFFIC_HEIGHT_X), tFloat32(USACC_ROI_INTERSECTION_CROSSTRAFFIC_WIDTH_Y));//right
    //-1100 -700 550 900
    roiXYIntersectionCrosstrafficLeft.f32x_min = m_propF32IntersectionCrosstrafficLeftXmin;
    roiXYIntersectionCrosstrafficLeft.f32y_min = m_propF32IntersectionCrosstrafficLeftYmin;
    roiXYIntersectionCrosstrafficLeft.f32x_max = m_propF32IntersectionCrosstrafficLeftXmax;
    roiXYIntersectionCrosstrafficLeft.f32y_max = m_propF32IntersectionCrosstrafficLeftYmax;
    //roiXYIntersectionCrosstrafficLeft = CalcXYMinAndMax((tFloat32(USACC_ROI_INTERSECTION_CROSSTRAFFIC_LEFT_BOTTOMLEFT_X)+m_propF32DistanceToStopLine), tFloat32(USACC_ROI_INTERSECTION_CROSSTRAFFIC_LEFT_BOTTOMLEFT_Y), tFloat32(USACC_ROI_INTERSECTION_CROSSTRAFFIC_LEFT_HEIGHT_X), tFloat32(USACC_ROI_INTERSECTION_CROSSTRAFFIC_LEFT_WIDTH_Y));
    //-550 -250 300 1200
    roiXYOvertakingOncomingTraffic.f32x_min = m_propF32OvertakeOncomingXmin;
    roiXYOvertakingOncomingTraffic.f32y_min = m_propF32OvertakeOncomingYmin;
    roiXYOvertakingOncomingTraffic.f32x_max = m_propF32OvertakeOncomingXmax;
    roiXYOvertakingOncomingTraffic.f32y_max = m_propF32OvertakeOncomingYmax;
    //roiXYOvertakingOncomingTraffic = CalcXYMinAndMax(tFloat32(USACC_ROI_OVERTAKE_ONCOMING_BOTTOMLEFT_X), tFloat32(USACC_ROI_OVERTAKE_ONCOMING_BOTTOMLEFT_Y), tFloat32(USACC_ROI_OVERTAKE_ONCOMING_HEIGHT_X), tFloat32(USACC_ROI_OVERTAKE_ONCOMING_WIDTH_Y));
    //270 570 300 1200
    //roiXYOvertakingOriginalLane = CalcXYMinAndMax(tFloat32(USACC_ROI_OVERTAKE_ORIGINALLANE_BOTTOMLEFT_X), tFloat32(USACC_ROI_OVERTAKE_ORIGINALLANE_BOTTOMLEFT_Y), tFloat32(USACC_ROI_OVERTAKE_ORIGINALLANE_HEIGHT_X), tFloat32(USACC_ROI_OVERTAKE_ORIGINALLANE_WIDTH_Y));
    roiXYOvertakingOriginalLane.f32x_min = m_propF32OvertakeOriginallaneXmin;
    roiXYOvertakingOriginalLane.f32x_max = m_propF32OvertakeOriginallaneXmax;
    roiXYOvertakingOriginalLane.f32y_min = m_propF32OvertakeOriginallaneYmin;
    roiXYOvertakingOriginalLane.f32y_max = m_propF32OvertakeOriginallaneYmax;
    //-150 150 50 950
    //roiXYOvertakingObstacleStraight = CalcXYMinAndMax(tFloat32(USACC_ROI_OVERTAKE_OBSTACLESTRAIGHT_BOTTOMLEFT_X), tFloat32(USACC_ROI_OVERTAKE_OBSTACLESTRAIGHT_BOTTOMLEFT_Y), tFloat32(USACC_ROI_OVERTAKE_OBSTACLESTRAIGHT_HEIGHT_X), tFloat32(USACC_ROI_OVERTAKE_OBSTACLESTRAIGHT_WIDTH_Y));
    roiXYOvertakingObstacleStraight.f32x_min = m_propF32OvertakeObstacleStraightXmin;
    roiXYOvertakingObstacleStraight.f32y_min = m_propF32OvertakeObstacleStraightYmin;
    roiXYOvertakingObstacleStraight.f32x_max = m_propF32OvertakeObstacleStraightXmax;
    roiXYOvertakingObstacleStraight.f32y_max = m_propF32OvertakeObstacleStraightYmax;
    //-300 -100 50 950
    //roiXYOvertakingObstacleStraightLeftHalf = CalcXYMinAndMax(tFloat32(USACC_ROI_OVERTAKE_OBSTACLESTRAIGHTUInt32_LEFTHALF_BOTTOMLEFT_X), tFloat32(USACC_ROI_OVERTAKE_OBSTACLESTRAIGHT_LEFTHALF_BOTTOMLEFT_Y), tFloat32(USACC_ROI_OVERTAKE_OBSTACLESTRAIGHT_LEFTHALF_HEIGHT_X), tFloat32(USACC_ROI_OVERTAKE_OBSTACLESTRAIGHT_LEFTHALF_WIDTH_Y));
    roiXYOvertakingObstacleStraightLeftHalf.f32x_min = m_propF32OvertakeObstacleStraightLeftHalfXmin;
    roiXYOvertakingObstacleStraightLeftHalf.f32y_min = m_propF32OvertakeObstacleStraightLeftHalfYmin;
    roiXYOvertakingObstacleStraightLeftHalf.f32x_max = m_propF32OvertakeObstacleStraightLeftHalfXmax;
    roiXYOvertakingObstacleStraightLeftHalf.f32y_max = m_propF32OvertakeObstacleStraightLeftHalfYmax;


    //roiXYParkingCrossCrossTrafficRight = CalcXYMinAndMax(tFloat32(USACC_ROI_PARKING_CROSS_CROSSTRAFFIC_RIGHT_BOTTOMLEFT_X), tFloat32(USACC_ROI_PARKING_CROSS_CROSSTRAFFIC_RIGHT_BOTTOMLEFT_Y), tFloat32(USACC_ROI_PARKING_CROSS_CROSSTRAFFIC_RIGHT_HEIGHT_X), tFloat32(USACC_ROI_PARKING_CROSS_CROSSTRAFFIC_RIGHT_WIDTH_Y));
    roiXYParkingOutBox1.f32x_min = m_propF32ParkingPullOutCrossOncoming1Xmin;
    roiXYParkingOutBox1.f32x_max = m_propF32ParkingPullOutCrossOncoming1Xmax;
    roiXYParkingOutBox1.f32y_min = m_propF32ParkingPullOutCrossOncoming1Ymin+m_propF32OffsetParkingY;
    roiXYParkingOutBox1.f32y_max = m_propF32ParkingPullOutCrossOncoming1Ymax+m_propF32OffsetParkingY;

    LOG_INFO("Xmin %f", roiXYParkingOutBox1.f32x_min);
    LOG_INFO("Xmax %f", roiXYParkingOutBox1.f32x_max);
    LOG_INFO("Ymin %f", roiXYParkingOutBox1.f32y_min);
    LOG_INFO("Ymax %f", roiXYParkingOutBox1.f32y_max);

    roiXYParkingOutBox2.f32x_min = m_propF32ParkingPullOutCrossOncoming2Xmin;
    roiXYParkingOutBox2.f32x_max = m_propF32ParkingPullOutCrossOncoming2Xmax;
    roiXYParkingOutBox2.f32y_min = m_propF32ParkingPullOutCrossOncoming2Ymin+m_propF32OffsetParkingY;
    roiXYParkingOutBox2.f32y_max = m_propF32ParkingPullOutCrossOncoming2Ymax+m_propF32OffsetParkingY;
    //-450 -100 100 800#
    roiXYParkingCrossOncomingTrafficRight.f32x_min = m_propF32ParkingCrossOncomingLaneXmin;
    roiXYParkingCrossOncomingTrafficRight.f32y_min = m_propF32ParkingCrossOncomingLaneYmin;
    roiXYParkingCrossOncomingTrafficRight.f32x_max = m_propF32ParkingCrossOncomingLaneXmax;
    roiXYParkingCrossOncomingTrafficRight.f32y_max = m_propF32ParkingCrossOncomingLaneYmax;

    /*Crosswalk*/
    roiXYCrosswalk.f32x_min = m_propF32CheckCrosswalkXmin;
    roiXYCrosswalk.f32x_max = m_propF32CheckCrosswalkXmax;
    roiXYCrosswalk.f32y_min = m_propF32CheckCrosswalkYmin;
    roiXYCrosswalk.f32y_max = m_propF32CheckCrosswalkYmax;

    /*Merge check target lane*/
    roiXYMergeCheckTargetLane.f32x_min = m_propF32MergeCheckTargetLaneXmin;
    roiXYMergeCheckTargetLane.f32x_max = m_propF32MergeCheckTargetLaneXmax;
    roiXYMergeCheckTargetLane.f32y_min = m_propF32MergeCheckTargetLaneYmin;
    roiXYMergeCheckTargetLane.f32y_max = m_propF32MergeCheckTargetLaneYmax;
    //roiXYParkingCrossOncomingTrafficRight = CalcXYMinAndMax(tFloat32(USACC_ROI_PARKING_CROSS_ONCOMINGLANE_BOTTOMLEFT_X), tFloat32(USACC_ROI_PARKING_CROSS_ONCOMINGLANE_BOTTOMLEFT_Y), tFloat32(USACC_ROI_PARKING_CROSS_ONCOMINGLANE_HEIGHT_X), tFloat32(USACC_ROI_PARKING_CROSS_ONCOMINGLANE_WIDTH_Y));
    defaultROI.f32x_max=0.0;
    defaultROI.f32x_min=0.0;
    defaultROI.f32y_max=0.0;
    defaultROI.f32y_min=0.0;



    lastInputAction.ui8FilterId = F_ULTRASONIC_ACC;
    lastInputAction.bEnabled = tTrue;
    lastInputAction.bStarted = tTrue;
    lastInputAction.ui32ArduinoTimestamp = 0;
    lastInputAction.ui32Command = AC_UA_DRIVING_MODE;

    m_UI32MaxCounter=static_cast<tUInt32> (m_propF32ThresholdRelObstBend/m_propF32HighBoxBend);
    if(m_UI32MaxCounter <= 4)LOG_WARNING("Only 4 or less Boxes are considerated for the box");
    if(m_UI32MaxCounter < 1)m_UI32MaxCounter=1;
    //LOG_INFO("roiXYIntersectionCrosstraffic.f32x_min: %f", roiXYIntersectionCrosstraffic.f32x_min);
    /*
    LOG_INFO("roiXYIntersectionOncoming: %f %f %f %f\n", roiXYIntersectionOncoming.f32x_min, roiXYIntersectionOncoming.f32x_max, roiXYIntersectionOncoming.f32y_min, roiXYIntersectionOncoming.f32y_max);
    LOG_INFO("roiXYIntersectionCrosstrafficRight: %f %f %f %f\n", roiXYIntersectionCrosstraffic.f32x_min, roiXYIntersectionCrosstraffic.f32x_max, roiXYIntersectionCrosstraffic.f32y_min, roiXYIntersectionCrosstraffic.f32y_max);
    LOG_INFO("roiXYIntersectionCrosstrafficLeft: %f %f %f %f\n", roiXYIntersectionCrosstrafficLeft.f32x_min, roiXYIntersectionCrosstrafficLeft.f32x_max, roiXYIntersectionCrosstrafficLeft.f32y_min, roiXYIntersectionCrosstrafficLeft.f32y_max);
    LOG_INFO("roiXYOvertakingOncomingTraffic: %f %f %f %f\n", roiXYOvertakingOncomingTraffic.f32x_min, roiXYOvertakingOncomingTraffic.f32x_max, roiXYOvertakingOncomingTraffic.f32y_min, roiXYOvertakingOncomingTraffic.f32y_max);
    LOG_INFO("roiXYOvertakingOriginalLane: %f %f %f %f\n", roiXYOvertakingOriginalLane.f32x_min, roiXYOvertakingOriginalLane.f32x_max, roiXYOvertakingOriginalLane.f32y_min, roiXYOvertakingOriginalLane.f32y_max);
    LOG_INFO("roiXYOvertakingObstacleStraight: %f %f %f %f\n", roiXYOvertakingObstacleStraight.f32x_min, roiXYOvertakingObstacleStraight.f32x_max, roiXYOvertakingObstacleStraight.f32y_min, roiXYOvertakingObstacleStraight.f32y_max);
    LOG_INFO("roiXYOvertakingObstacleStraightLeftHalf: %f %f %f %f\n", roiXYOvertakingObstacleStraightLeftHalf.f32x_min, roiXYOvertakingObstacleStraightLeftHalf.f32x_max, roiXYOvertakingObstacleStraightLeftHalf.f32y_min, roiXYOvertakingObstacleStraightLeftHalf.f32y_max);
    LOG_INFO("roiXYParkingOutBox1: %f %f %f %f\n", roiXYParkingOutBox1.f32x_min, roiXYParkingOutBox1.f32x_max, roiXYParkingOutBox1.f32y_min, roiXYParkingOutBox1.f32y_max);
    LOG_INFO("roiXYParkingOutBox2: %f %f %f %f\n", roiXYParkingOutBox2.f32x_min, roiXYParkingOutBox2.f32x_max, roiXYParkingOutBox2.f32y_min, roiXYParkingOutBox2.f32y_max);
    LOG_INFO("roiXYParkingCrossOncomingTrafficRight: %f %f %f %f\n", roiXYParkingCrossOncomingTrafficRight.f32x_min, roiXYParkingCrossOncomingTrafficRight.f32x_max, roiXYParkingCrossOncomingTrafficRight.f32y_min, roiXYParkingCrossOncomingTrafficRight.f32y_max);
    LOG_INFO(cString::Format("Configuration and constructor finished!"));*/


    for (tUInt32 i = 0; i < m_propui32NumberLaneDetectionInputForRamp; i++)
    {
        vecBvalidLane.push_back(tFalse);
    }
    LOG_INFO("USACC: Constructor ends!!!!");

}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult UltrasonicACC::Configure()
{
    // done
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    //LoadProperties();
    RETURN_NOERROR;
}


// ------------------------------
// ---------- PROCESS -----------
// ------------------------------
// process() is always called each time a trigger occured (new data sample arrive).
// IMPORTANT: Make sure to read ALL samples of ALL readers!
tResult UltrasonicACC::Process(tTimeStamp)
{
    //object_ptr<const ISample> pReadTempSample;
    if(IS_OK(ProcessUS()))
    {
    }
    if (IS_OK(ProcessAction()))
    {
    }
    if (IS_OK(ProcessSteeringAngle()))
    {
    }
    if (IS_OK(ProcessTargetSpeed()))
    {
    }
    if(IS_OK(ProcessLaneDetectionLine()))
    {

    }


    // get the last laserscanner sample
    TLaserScannerData::Data tmp_dataInputLaserData;
    if(IS_OK(o_LaserScanner.readPin(m_ReaderLaserScanner, (void *) &tmp_dataInputLaserData)))
    {
        RETURN_IF_FAILED(ProcessLIDARInput(tmp_dataInputLaserData));
    }



    // video input
    object_ptr<const ISample> pReadVideoSample;
    if (IS_OK(m_ReaderVideo.GetNextSample(pReadVideoSample)))
    {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        //lock read buffer
        if (IS_OK(pReadVideoSample->Lock(pReadBuffer)))
        {

            if(m_ui32FrameCounter == m_propUI32Frames)
            {
                // create a opencv matrix from the media sample buffer
                Mat inputImage (cv::Size(m_sInputImageFormat.m_ui32Width, m_sInputImageFormat.m_ui32Height),
                                CV_8UC3, (uchar*)pReadBuffer->GetPtr());
                Mat blackMat (cv::Size(m_propTransformedImageWidth, m_propTransformedImageHeight),
                              CV_8UC3, Scalar(0,0,0));

                // set our global mat to input image
                m_MatInput  = inputImage.clone();
                m_MatOutputDebug = inputImage.clone();
                m_MatRoi = blackMat.clone();

                // LOG_INFO("process frames");

                rectangle(m_MatRoi, Rect((m_propTransformedImageWidth/2) + m_propF32IntersectionOncomingXmin / 3,
                                         m_propTransformedImageHeight - m_propF32IntersectionOncomingYmax / 3 + 100,
                                         abs(m_propF32IntersectionOncomingXmax - m_propF32IntersectionOncomingXmin) / 3,
                                         abs(m_propF32IntersectionOncomingYmax - m_propF32IntersectionOncomingYmin) / 3),
                          Scalar(0, 255, 0), CV_FILLED);

                rectangle(m_MatRoi, Rect((m_propTransformedImageWidth/2) + m_propF32IntersectionCrosstrafficLeftXmin / 3,
                                         m_propTransformedImageHeight - m_propF32IntersectionCrosstrafficLeftYmax / 3 + 100,
                                         abs(m_propF32IntersectionCrosstrafficLeftXmax - m_propF32IntersectionCrosstrafficLeftXmin) / 3,
                                         abs(m_propF32IntersectionCrosstrafficLeftYmax - m_propF32IntersectionCrosstrafficLeftYmin) / 3),
                          Scalar(0, 255, 0), CV_FILLED);

                rectangle(m_MatRoi, Rect((m_propTransformedImageWidth/2) + m_propF32IntersectionCrosstrafficRightXmin / 3,
                                         m_propTransformedImageHeight - m_propF32IntersectionCrosstrafficRightYmax / 3 + 100,
                                         abs(m_propF32IntersectionCrosstrafficRightXmax - m_propF32IntersectionCrosstrafficRightXmin) / 3,
                                         abs(m_propF32IntersectionCrosstrafficRightYmax - m_propF32IntersectionCrosstrafficRightYmin) / 3),
                          Scalar(0, 255, 0), CV_FILLED);


                // set ROI of our Image
                setRoiOfImage();

                // transform to top down
                transformToBirdsEyeView();

                addWeighted(m_MatOutput, 1, m_MatRoi, 0.4f, 0.0, m_MatOutput);

                m_ui32FrameCounter = 0;
            }

            m_ui32FrameCounter++;

        }
    }

    // output//Write processed Image to Output Pin
    if (!m_MatOutput.empty())
    {
        //update output format if matrix size does not fit to
        if (m_MatOutput.total() * m_MatOutput.elemSize() != m_sOutputImageFormat.m_szMaxByteSize)
        {
            setTypeFromMat(m_WriterVideo, m_MatOutput);
        }
        // write to pin
        writeMatToPin(m_WriterVideo, m_MatOutput, m_pClock->GetStreamTime());
    }
    // output//Write processed Image to Output Pin
    if (!m_MatOutputDebug.empty())
    {
        //update output format if matrix size does not fit to
        if (m_MatOutputDebug.total() * m_MatOutputDebug.elemSize() != m_sOutputImageFormat.m_szMaxByteSize)
        {
            setTypeFromMat(m_WriterVideoDebug, m_MatOutputDebug);
        }
        // write to pin
        writeMatToPin(m_WriterVideoDebug, m_MatOutputDebug, m_pClock->GetStreamTime());
    }

    RETURN_NOERROR;

}

/*****************************************************************/
/* Inputs                                                        */
/*****************************************************************/


// ------------- Functions for VIDEO stuff
// setTypeFromMat
// we need this for video outputs
void UltrasonicACC::setTypeFromMat(adtf::streaming::cSampleWriter& writer, const cv::Mat& outputImage, tBool keepFormat)
{
    adtf::streaming::tStreamImageFormat outputFormat;
    outputFormat.m_ui32Height = outputImage.rows;
    outputFormat.m_ui32Width = outputImage.cols;
    outputFormat.m_szMaxByteSize = outputImage.cols * outputImage.rows * outputImage.channels();

    if (!keepFormat)
    {
        if (outputImage.channels() == 1)
        {
            outputFormat.m_strFormatName = ADTF_IMAGE_FORMAT(GREYSCALE_8);
        }
        if (outputImage.channels() == 3)
        {
            outputFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
        }
    }
    outputFormat.m_ui8DataEndianess = PLATFORM_BYTEORDER;

    adtf::ucom::object_ptr<IStreamType> pTypeOutput = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pTypeOutput, outputFormat);

    writer << pTypeOutput;
}

// writeMatToPin
// this function will output our image
void UltrasonicACC::writeMatToPin(adtf::streaming::cSampleWriter& writer, const cv::Mat& outputImage, tTimeStamp streamTime)
{
    //create write buffer
    object_ptr<ISample> pWriteSample;

    if (IS_OK(alloc_sample(pWriteSample, streamTime)))
    {
        object_ptr_locked<ISampleBuffer> pWriteBuffer;
        if (IS_OK(pWriteSample->WriteLock(pWriteBuffer, outputImage.cols * outputImage.rows * outputImage.channels())))
        {
            pWriteBuffer->Write(adtf_memory_buffer<void, tSize>((void*) outputImage.data,
                                                                outputImage.cols * outputImage.rows * outputImage.channels()));
        }
    }

    writer << pWriteSample << flush << trigger;
}

// ChangeType
// we need this for video inputs
tResult UltrasonicACC::ChangeType(adtf::streaming::cDynamicSampleReader& inputPin,
                                  const adtf::streaming::ant::IStreamType& oType)
{
    if (oType == adtf::streaming::stream_meta_type_image())
    {
        adtf::ucom::object_ptr<const adtf::streaming::IStreamType> pTypeInput;
        // get pType from input reader
        inputPin >> pTypeInput;
        adtf::streaming::get_stream_type_image_format(m_sInputImageFormat, *pTypeInput);

        //set also output format
        adtf::streaming::get_stream_type_image_format(m_sOutputImageFormat, *pTypeInput);
        //we always have a grayscale output image
        m_sOutputImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(BGR_24);
        // and set pType also to samplewriter
        m_WriterVideo << pTypeInput;
        m_WriterVideoDebug << pTypeInput;
    }
    else
    {
        RETURN_ERROR(ERR_INVALID_TYPE);
    }

    RETURN_NOERROR;
}

void UltrasonicACC::setRoiOfImage()
{
    // set roi points
    m_p2fROIVertices[0] = Point(m_propROIPoint1X, m_propROIPoint1Y); // upper left
    m_p2fROIVertices[1] = Point(m_propROIPoint2X, m_propROIPoint2Y); // upper right
    m_p2fROIVertices[2] = Point(m_propROIPoint3X, m_propROIPoint3Y); // lower right
    m_p2fROIVertices[3] = Point(m_propROIPoint4X, m_propROIPoint4Y); // lower left

    // draw it on mat for debugging
    circle(m_MatOutputDebug, m_p2fROIVertices[0], 4, Scalar(0, 0, 255));
    circle(m_MatOutputDebug, m_p2fROIVertices[1], 4, Scalar(0, 0, 255));
    circle(m_MatOutputDebug, m_p2fROIVertices[2], 4, Scalar(0, 0, 255));
    circle(m_MatOutputDebug, m_p2fROIVertices[3], 4, Scalar(0, 0, 255));
    line(m_MatOutputDebug, m_p2fROIVertices[0], m_p2fROIVertices[1], Scalar(0, 100, 255), 2, CV_AA);
    line(m_MatOutputDebug, m_p2fROIVertices[1], m_p2fROIVertices[2], Scalar(0, 100, 255), 2, CV_AA);
    line(m_MatOutputDebug, m_p2fROIVertices[2], m_p2fROIVertices[3], Scalar(0, 100, 255), 2, CV_AA);
    line(m_MatOutputDebug, m_p2fROIVertices[3], m_p2fROIVertices[0], Scalar(0, 100, 255), 2, CV_AA);

}

void UltrasonicACC::transformToBirdsEyeView()
{
    // image width and height
    m_p2fTransformedImageVertices[0] = Point(0, 0);
    m_p2fTransformedImageVertices[1] = Point(m_propTransformedImageWidth, 0);
    m_p2fTransformedImageVertices[2] = Point(m_propTransformedImageWidth, m_propTransformedImageHeight);
    m_p2fTransformedImageVertices[3] = Point(0, m_propTransformedImageHeight);

    Mat dst(cv::Size(m_propTransformedImageWidth, m_propTransformedImageHeight), CV_8UC3);
    // transform
    Mat M = getPerspectiveTransform(m_p2fROIVertices, m_p2fTransformedImageVertices);
    warpPerspective(m_MatInput, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);

    // set to our global mat
    m_MatOutput       = dst.clone();
}

// ---------------------------------------


tResult UltrasonicACC::ProcessUS()
{


    TUltrasonicStruct::Data tmp_dataUltrasonicSignal;
    static tTimeStamp lasttmus = 0;
    tResult res;
    if(IS_FAILED(res = o_UltrasonicStruct.readPin(m_ReaderUltrasonic, (void *) & tmp_dataUltrasonicSignal, lasttmus)))
    {
        RETURN_ERROR(res);
    }
    lasttmus = tmp_dataUltrasonicSignal.tSideLeft.ui32ArduinoTimestamp;



    /* no*/
    /* safety measure if no speed signal is transmitted to ACC, timeout will occur and target speed will be set to zero*/
    if(GetTimeoutCounter() >= 100)
    {
        /*TSignalValue::Data tmp_dataZeroTargetSpeed;
        tmp_dataZeroTargetSpeed.f32Value = 0.0f;
        tmp_dataZeroTargetSpeed.ui32ArduinoTimestamp = m_pClock->GetStreamTime();
        // transmit target speed with value zero to stop immediately
        //TransmitTargetSpeed(tmp_dataZeroTargetSpeed);
        if(m_propBDebugFirstStart) LOG_INFO("USACC: ProcessUS() no speed signal is transmitted to ACC -> speed was set to zero\n");*/
    }
    else
    {
        /* increase timeout counter; if everything works as planned, counter will be reset at receiving input-speed */
        IncreaseTimeoutCounter();
    }

    //tBool state = GetRunningState();
    //if(state == tTrue)LOG_WARNING("Running State is true");
    if(GetRunningState()) //no movement
    {
        /*//no veraendern
        if((GetLastInputTargetSpeed() != 0.0f) && (fabsf(GetLastModifiedTargetSpeed()) <= 10))
        {
            tBool bPossibleRamp = CheckIfObstacleIsRamp();
        }*/
        /*LOG_INFO("In if getrunningstate!");
        tFloat32 targetSpeeed = GetLastInputTargetSpeed();
        LOG_INFO("Target Speed: %f", targetSpeeed);
        tFloat32 modSpeeed = GetLastModifiedTargetSpeed();
        LOG_INFO("Mod Speed: %f", modSpeeed);*/
        if((GetLastInputTargetSpeed() != 0.0f) && (fabsf(GetLastModifiedTargetSpeed()) <= m_F32ModifiedSpeedLowerBound))
        {
            IncreaseObstacleNoMovementCounter();

            if(GetObstacleNoMovementCounter() == (tUInt32(USfrequency)))
            {
                //check if the obstacle is only in the left half of the own lane
                if(GetMergeFlag())
                {
                    //Check if ramp is in front
                    LOG_INFO("Kurz bevor die Rampe gecheckt wird!");
                    //hier checken, was abkommt evtl bei GuardRail ein counter oder last distance speichern und vergleichen
                    tBool bPossibleRampWithLidarpoints = CheckIfObstacleIsRamp();
                    tBool bPossibleRampByGuardRail = CheckForGuardRail();
                    if(m_propBDebugRamp)
                    {
                        if(bPossibleRampWithLidarpoints)LOG_INFO("bPossibleRampWithLidarpoints is true");
                        else LOG_INFO("bPossibleRampWithLidarpoints is false");
                        if(bPossibleRampByGuardRail)LOG_INFO("bPossibleRampByGuardRail is true");
                        else LOG_INFO("bPossibleRampByGuardRail is false");
                    }
                    if(bPossibleRampWithLidarpoints && bPossibleRampByGuardRail)
                    {
                        if(IS_OK(TransmitFeedbackRamp()))
                        {
                            if(m_propBDebugRamp)LOG_INFO("FeedRamp was send!");
                        }
                        ResetObstacleNoMovementCounter();
                        SetOperationalMode(DRIVING_MODE_RAMP);//suchen, wo verwendet;
                        //auch aufheben

                    }
                }

            }
        }
        else
        {
            ResetObstacleNoMovementCounter();
        }
        /* set reset*/
        if(GetObstacleNoMovementCounter() >= (m_UI32NoMovementThreshold*tUInt32(USfrequency))) // 2018: 40; 2017: times 20 since rate of US is 20Hz
        {
            SetRunningState(tFalse);
            ResetObstacleNoMovementCounter();
            RETURN_IF_FAILED(o_FeedbackStruct.writePin(m_WriterFeedback, (void *) &m_dataFeedbackAtRest, m_pClock->GetStreamTime()));
            if(m_propBDebugActionCommand) LOG_INFO("USACC: ProcessUS() reset due to no movement although speed was set!\n");
            if(m_propBDebugFirstStart) LOG_INFO("USACC: ProcessUS() reset due to no movement although speed was set!\n");
            //LOG_WARNING("Sended no movement!");

        }
    }
    /* if runningState MoveAgain is active, speed should be tracked for possible moving */
    if(GetRunningStateMoveAgain())
    {
        if((GetLastInputTargetSpeed() != 0.0f) && (fabsf(GetLastModifiedTargetSpeed()) >= m_F32ModifiedSpeedLowerBound))
        {
            IncreaseObstacleMovementAgainCounter();
        }
        else
        {
            ResetObstacleMovementAgainCounter();
        }
        if(GetObstacleMovementAgainCounter() >= (m_UI32MovementAgainThreshold*tUInt32((USfrequency))))// 2018: 40; 2017: times 20 since rate of US is 20Hz
        {
            SetRunningStateMoveAgain(tFalse);
            ResetObstacleMovementAgainCounter();
            RETURN_IF_FAILED(o_FeedbackStruct.writePin(m_WriterFeedback, (void *) &m_dataFeedbackMovingAgain, m_pClock->GetStreamTime()));
            if(m_propBDebugFirstStart) LOG_INFO("USACC: ProcessUS() Car should move again\n");
        }
    }
    RETURN_IF_FAILED(ProcessUSInput(tmp_dataUltrasonicSignal));
    RETURN_NOERROR;
}

tResult UltrasonicACC::ProcessAction()
{
    TActionStruct::Data inputAction;
    tResult res;
    static tTimeStamp lasttmaction = 0;//to save the last timestamp
    if(IS_FAILED(res = o_ActionStruct.readPin(m_ReaderAction, (void *) &inputAction, lasttmaction)))
    {
        RETURN_ERROR(res);
    }
    lasttmaction = inputAction.ui32ArduinoTimestamp;

    /* Filter gets ui32Command to start the timer */
    if(inputAction.bEnabled && inputAction.bStarted && inputAction.ui8FilterId == F_ULTRASONIC_ACC)
    {
        if(m_propBDebugActionCommand)LOG_INFO("Action Command: %d", inputAction.ui32Command);
        //SetOperationalMode(DRIVING_MODE_ACTIVE);
        if(inputAction.ui32Command == AC_UA_DRIVING_MODE)
        {
            //SetLastActionInput(inputAction);
            if(m_propBDebugActionCommand)LOG_INFO("Goes into debug driving mode!");
            SetOperationalMode(DRIVING_MODE_ACTIVE);
            m_BDeactivateACCFilter = false;
            if(m_bDebugModeEnabled)LOG_WARNING(cString::Format("UltrasonicACC: changed to DRIVING MODE, based on action ui32Command: %d",inputAction.ui32Command));
            RETURN_IF_FAILED(TransmitFeedBackDrivingModeActive());
        }
        else if(inputAction.ui32Command == AC_UA_MTP_MODE)
        {
            //SetLastActionInput(inputAction);
            if(m_propBDebugActionCommand)LOG_INFO("Action Command MtP mode!");
            SetLastLaneDetectionLine(m_dataDefaultLanes);
            SetOperationalMode(MtP_MODE_ACTIVE);
            //m_BDeactivateACCFilter = true;
            if(m_bDebugModeEnabled)LOG_WARNING(cString::Format("UltrasonicACC: changed to MtP MODE, based on action ui32Command: %d",inputAction.ui32Command));
            RETURN_IF_FAILED(TransmitFeedBackMTPMode());
        }
        else if(inputAction.ui32Command == AC_UA_CHECK_NO_MOVEMENT)
        {
            //SetLastActionInput(inputAction);
            //LOG_INFO("Check no movemnet");
            SetMergeFlag(tFalse);
            if(m_propBDebugActionCommand)LOG_INFO("Action Command Check no movement");
            if(!GetRunningState())
            {
                ResetObstacleNoMovementCounter();
                SetRunningState(tTrue);
                /* UA has to track speed and check if targetspeed-input is != zero, but output stays on zero for a longer time! */
                if(m_bDebugModeEnabled)LOG_WARNING(cString::Format("UltrasonicACC: checking no-movement, based on action ui32Command: %d",inputAction.ui32Command));
            }
            else
            {
                if(m_bDebugModeEnabled)LOG_WARNING(cString::Format("UltrasonicACC: ui32Command received to check no-movement, but already running in that mode."));
            }
            SetOperationalMode(DRIVING_MODE_ACTIVE);
        }
        else if(inputAction.ui32Command == AC_UA_CHECK_MOVING_AGAIN)
        {
            //SetLastActionInput(inputAction);
            LOG_INFO("Check moving again!");
            if(m_propBDebugActionCommand)LOG_INFO("AC_UA_CHECK_MOVING_AGAIN");
            if(!GetRunningStateMoveAgain())
            {
                ResetObstacleMovementAgainCounter();
                SetRunningStateMoveAgain(tTrue);
                /* UA has to track speed and check if targetspeed-input is != zero, but output stays on zero for a longer time! */
                if(m_bDebugModeEnabled){LOG_WARNING(cString::Format("UltrasonicACC: checking moveAgain, based on action ui32Command: %d",inputAction.ui32Command));}
            }
            else
            {
                if(m_bDebugModeEnabled){LOG_WARNING(cString::Format("UltrasonicACC: ui32Command received to check moveAgain, but already running in that mode."));}
            }
            SetOperationalMode(DRIVING_MODE_ACTIVE);
        }
        else if(inputAction.ui32Command == AC_UA_MERGE_LEFT)
        {
            if(!GetRunningState())
            {
                ResetObstacleNoMovementCounter();
                SetRunningState(tTrue);
                /* UA has to track speed and check if targetspeed-input is != zero, but output stays on zero for a longer time! */
                if(m_bDebugModeEnabled)LOG_WARNING(cString::Format("UltrasonicACC: checking no-movement, based on action ui32Command: %d",inputAction.ui32Command));
            }
            else
            {
                if(m_bDebugModeEnabled)LOG_WARNING(cString::Format("UltrasonicACC: ui32Command received to check no-movement, but already running in that mode."));
            }
            SetOperationalMode(DRIVING_MODE_ACTIVE);
            if(m_propBDebugActionCommand)LOG_INFO("Action merge!");
            SetMergeFlag(tTrue);

        }
        else if(inputAction.ui32Command == AC_UA_INTERSECTION_CHECK_ALL)
        {
            SetLastActionInput(inputAction);
            RETURN_IF_FAILED(TransmitFeedbackIntersectionROIreceived());
        }
        else if(inputAction.ui32Command == AC_UA_INTERSECTION_CHECK_ONCOMING_TRAFFIC)
        {
            SetLastActionInput(inputAction);
            RETURN_IF_FAILED(TransmitFeedbackIntersectionROIreceived());
        }
        else if(inputAction.ui32Command == AC_UA_INTERSECTION_CHECK_ONCOMING_AND_RIGHT)
        {
            SetLastActionInput(inputAction);
            RETURN_IF_FAILED(TransmitFeedbackIntersectionROIreceived());
        }
        else if(inputAction.ui32Command == AC_UA_INTERSECTION_CHECK_LEFT)
        {
            SetLastActionInput(inputAction);
            RETURN_IF_FAILED(TransmitFeedbackIntersectionROIreceived());
        }
        else if(inputAction.ui32Command == AC_UA_INTERSECTION_NOTHING_TO_CHECK)
        {
            SetLastActionInput(inputAction);
            RETURN_IF_FAILED(TransmitFeedbackIntersectionROIreceived());
        }
        else if((inputAction.ui32Command == AC_UA_INTERSECTION_CHECK_CROSS_TRAFFIC_RIGHT)
                || (inputAction.ui32Command == AC_UA_INTERSECTION_CHECK_RIGHT))
        {
            SetLastActionInput(inputAction);
            RETURN_IF_FAILED(TransmitFeedbackIntersectionROIreceived());
        }
        else if(inputAction.ui32Command == AC_UA_INTERSECTION_CHECK_RIGHT_AND_LEFT)
        {
            SetLastActionInput(inputAction);
            RETURN_IF_FAILED(TransmitFeedbackIntersectionROIreceived());
        }
        else if(inputAction.ui32Command == AC_UA_OVERTAKE_CHECK_ONCOMING_TRAFFIC)
        {
            //SetLastActionInput(inputAction);
            if(m_propBDebugActionCommand)LOG_INFO("AC_UA_OVERTAKE_CHECK_ONCOMING_TRAFFIC");
            ProcessROI(roiXYOvertakingOncomingTraffic);

        }
        else if(inputAction.ui32Command == AC_UA_OVERTAKE_CHECK_OVERTAKE_ORIGINAL_LANE)
        {
            //SetLastActionInput(inputAction);
            ProcessROI(roiXYOvertakingOriginalLane);
        }
        else if (inputAction.ui32Command== AC_UA_OVERTAKE_CHECK_OWN_LANE_STRAIGHT)
        {
            //SetLastActionInput(inputAction);
            if(m_propBDebugActionCommand)LOG_INFO("AC_UA_OVERTAKE_CHECK_OWN_LANE_STRAIGHT");
            // call method to check traffic on actual own lane after overtake
            ProcessROI(roiXYOvertakingObstacleStraight);

        }
        else if (inputAction.ui32Command
                 == AC_UA_OVERTAKE_CHECK_OWN_LANE_STRAIGHT_LEFTHALF) {
            if(m_propBDebugActionCommand)LOG_INFO("AC_UA_OVERTAKE_CHECK_OWN_LANE_STRAIGHT_LEFTHALF");
            /* call method to check traffic on actual own lane after overtake */
            //SetLastActionInput(inputAction);
            ProcessROI(roiXYOvertakingObstacleStraightLeftHalf);

        }
        else if(inputAction.ui32Command
                == AC_UA_PARKING_CROSS_CHECK_ONCOMING_TRAFFIC)
        {
            SetLastActionInput(inputAction);
            if(m_propBDebugActionCommand)LOG_INFO("AC_UA_PARKING_CROSS_CHECK_ONCOMING_TRAFFIC");
            ProcessROI(roiXYParkingCrossOncomingTrafficRight);

        }
        else if(inputAction.ui32Command == AC_UA_PARKING_CROSS_PULLOUT_CHECK_CROSS_TRAFFIC)
        {
            SetLastActionInput(inputAction);
            if(m_propBDebugActionCommand)LOG_INFO("AC_UA_PARKING_CROSS_PULLOUT_CHECK_CROSS_TRAFFIC");
            // call method to check traffic on oncoming lane right before starting cross-parking maneuver
            ProcessROI(roiXYParkingOutBox1, roiXYParkingOutBox2);

        }
        else if(inputAction.ui32Command == AC_UA_CROSSWALK_CHECKFOROBSTACLE)
        {

            //LOG_INFO("Xmin: %f, Xmax: %f", roiXYCrosswalk.f32x_min, roiXYCrosswalk.f32x_max);
             //LOG_INFO("Ymin: %f, Ymax: %f", roiXYCrosswalk.f32y_min, roiXYCrosswalk.f32y_max);
            if(m_propBDebugActionCommand)LOG_INFO("USACC: received command to check the the crosswalk for obstacles");
            RETURN_IF_FAILED(ProcessROI(roiXYCrosswalk));
            /*tResult UltrasonicACC::ProcessActionCrosswalk()
{
    //UltrasonicACC::LidarObstacles lidarObstacle = GetLIDARObstacles();
    ProcessROI(roiXYCrosswalk);

    RETURN_NOERROR;
}*/
        }
        else if(inputAction.ui32Command == AC_UA_NORMAL_DRIVE)
        {
            if(m_propBDebugActionCommand)LOG_INFO("USACC: received command NORMAL_DRIVE");
            RETURN_IF_FAILED(SetIntersectionMode(NORMAL_DRIVE));

        }
        else if(inputAction.ui32Command == AC_UA_AFTER_LEFT_TURN)
        {
            if(m_propBDebugActionCommand)LOG_INFO("USACC: received command AC_UA_AFTER_LEFT_TURN");
            RETURN_IF_FAILED(SetIntersectionMode(AFTER_LEFT));
        }
        else if(inputAction.ui32Command == AC_UA_AFTER_RIGHT_TURN)
        {
            if(m_propBDebugActionCommand)LOG_INFO("USACC: received command AFTER_RIGHT");
            RETURN_IF_FAILED(SetIntersectionMode(AFTER_RIGHT));
        }
        else if(inputAction.ui32Command == AC_UA_CHECK_LANE_FOR_MERGE)
        {
            if(m_propBDebugActionCommand)LOG_INFO("USACC: received command to the lane for merge");
            RETURN_IF_FAILED(ProcessROI(roiXYMergeCheckTargetLane));
        }
        else if(inputAction.ui32Command == AC_UA_CHECK_ROI_INTERSECTION)
        {
            TActionStruct::Data lastAction = GetLastActionInput();
            if(m_propBDebugActionCommand)LOG_INFO("Last command: %d", lastAction.ui32Command);
            if(lastAction.ui32Command == AC_UA_INTERSECTION_CHECK_ALL)
            {
                RETURN_IF_FAILED(ProcessActionIntersectionCheckAll());
            }
            else if(lastAction.ui32Command == AC_UA_INTERSECTION_CHECK_ONCOMING_TRAFFIC)
            {
                RETURN_IF_FAILED(ProcessActionIntersectionOncomingTraffic());
            }
            else if(lastAction.ui32Command == AC_UA_INTERSECTION_CHECK_ONCOMING_AND_RIGHT)
            {
                RETURN_IF_FAILED(ProcessIntersectionCheckOncomingAndRight());
            }
            else if(lastAction.ui32Command == AC_UA_INTERSECTION_CHECK_LEFT)
            {
                RETURN_IF_FAILED(ProcessActionIntersectionCheckLeft());
            }
            else if(lastAction.ui32Command == AC_UA_INTERSECTION_NOTHING_TO_CHECK)
            {
                RETURN_IF_FAILED(ProcessActionIntersectionNothingToCheck());
            }
            else if((lastAction.ui32Command == AC_UA_INTERSECTION_CHECK_RIGHT) || (lastAction.ui32Command == AC_UA_INTERSECTION_CHECK_CROSS_TRAFFIC_RIGHT))
            {
                RETURN_IF_FAILED(ProcessActionIntersectionCrosstrafficRight());
            }
            else if(lastAction.ui32Command == AC_UA_INTERSECTION_CHECK_RIGHT_AND_LEFT)
            {
                RETURN_IF_FAILED(ProcessActionCheckRightAndLeft());
            }
            else
            {
                if(m_propBDebugActionCommand)LOG_INFO("Invalid state");
                RETURN_AND_LOG_ERROR_STR(ERR_INVALID_STATE, cString::Format("UltrasonicACC: ActionInput could not be processed, unknown ui32Command: %d",lastAction.ui32Command));
            }

        }
        else
        {
            if(m_propBDebugActionCommand)LOG_INFO("Invalid state");
            RETURN_AND_LOG_ERROR_STR(ERR_INVALID_STATE, cString::Format("UltrasonicACC: ActionInput could not be processed, unknown ui32Command: %d",inputAction.ui32Command));
        }

        /* in case of disable ui32Command, but still running no-movement check -> stop,disable */
        if(!inputAction.bEnabled && GetRunningState())
        {
            SetRunningState(tFalse);
        }
        /* in case of disable ui32Command, but still running moveAgain check -> stop,disable */
        if(!inputAction.bEnabled && GetRunningStateMoveAgain())
        {
            SetRunningStateMoveAgain(tFalse);
        }
        //RETURN_NOERROR;
    }
    RETURN_NOERROR;
}


tResult UltrasonicACC::ProcessSteeringAngle()
{
    boost::lock_guard<boost::mutex> lock(criticalSectionProcessSteeringAngle);
    TSignalValue::Data tmp_dataInputSteering;
    tResult res;
    static tTimeStamp lasttmsteering = 0;
    if(IS_FAILED(res = o_TSignalValue.readPin(m_ReaderSteeringAngle, (void *) & tmp_dataInputSteering, lasttmsteering)))
    {
        RETURN_ERROR(res);
    }
    lasttmsteering = tmp_dataInputSteering.ui32ArduinoTimestamp;

    /* Save temporary data to member variable */
    RETURN_IF_FAILED(SetSteeringAngle(tmp_dataInputSteering));
    //LOG_INFO("Steering Angle was set to %f", tmp_dataInputSteering.f32Value);
    RETURN_NOERROR;
}


tResult UltrasonicACC::ProcessTargetSpeed()//use US and Lidar
{
    //boost::lock_guard<boost::mutex> lock(criticalSectionLatestRecTargetSpeedAccess);
    //Read target speed

    TSignalValue::Data tmp_dataLastRecTargetSpeed;
    tResult res;
    static tTimeStamp lasttmTargetSpeed;
    if(IS_FAILED(res = o_TSignalValue.readPin(m_ReaderTargetSpeed , (void *) & tmp_dataLastRecTargetSpeed, lasttmTargetSpeed)))
    {
        RETURN_ERROR(res);
    }
    lasttmTargetSpeed = tmp_dataLastRecTargetSpeed.ui32ArduinoTimestamp;

    if(GetOperationalMode() == DRIVING_MODE_RAMP)
    {

        RETURN_IF_FAILED(ProcessDriveOnRamp());
        RETURN_IF_FAILED(SetLastInputTargetSpeed(tmp_dataLastRecTargetSpeed.f32Value));
        RETURN_IF_FAILED(SetLastModifiedTargetSpeed(tmp_dataLastRecTargetSpeed.f32Value));
        RETURN_IF_FAILED(TransmitTargetSpeed(tmp_dataLastRecTargetSpeed));
    }
    /* bool value expressing whether car is driving forwards(true) or backwards(backwards) */
    //LOG_INFO("Target Speed %f", tmp_dataLastRecTargetSpeed.f32Value);
    tBool tmp_DrivingForwards = tTrue;
    if(tmp_dataLastRecTargetSpeed.f32Value >= 0.0f)// car is driving in forwards direction
    {
        tmp_DrivingForwards = tTrue;
    }
    else
    {
        tmp_DrivingForwards = tFalse;// car is driving in backwards direction
        if(m_propBDebugOutputToConsole)LOG_INFO("Car drives backwards");
    }

    /* save input value before modification to global variable */
    SetLastInputTargetSpeed(tmp_dataLastRecTargetSpeed.f32Value);

    /* finding the depending value of the percentage in the data of the activated xml-file ????*/
    currentOperationalMode tmp_operationalMode = GetOperationalMode();

    /* get current steering angle */
    TSignalValue::Data tmp_dataSteeringAngleData = GetSteeringAngleInput();

    /* get weighted speed of all active sensors, depending on driving direction  */

    /* variable to save new target speed limit (absolute) based on sensor values and XML */
    TSignalValue::Data tmp_dataModTargetSpeed;

    ResetTimeoutCounter();
    if(tmp_DrivingForwards)// car moving forwards
    {
        TLaserScannerData::Data tmp_dataLidarStruct = GetLIDARInput();
        //ProcessLIDARInput(tmp_dataLidarStruct);
        UltrasonicACC::LidarObstacles tmp_dataLidarObstacle;
        tmp_dataLidarObstacle = UltrasonicACC::ObstacleDetectionWithLidar(tmp_dataLidarStruct);
        tmp_dataModTargetSpeed = tmp_dataLastRecTargetSpeed;
        //at the moment only one the own lane and to the front
        tFloat32 f32DistanceClosedRelevantObstacel = CalcClosestRelevantObstacle(tmp_dataLidarObstacle);//evaluates if Obstacels are within th roi
        if(m_propBDebugOutputToConsole)LOG_INFO("f32DistanceClosedRelevantObstacel: %f", f32DistanceClosedRelevantObstacel);
        if(m_bDebugModeEnabled)LOG_INFO("f32DistanceClosedRelevantObstacel: %f", f32DistanceClosedRelevantObstacel);
        tmp_operationalMode = GetOperationalMode();
        tFloat32 f32InterpolatedSpeed = GetRedPerscentFront(f32DistanceClosedRelevantObstacel ,tmp_operationalMode);
        //LOG_INFO("Interpolated Speed: %f", f32InterpolatedSpeed);
        if (m_bDebugModeEnabled)LOG_INFO("Interpolated Speed: %f", f32InterpolatedSpeed);
        tmp_dataModTargetSpeed.f32Value = ChooseOutputSpeedFront(f32InterpolatedSpeed, tmp_dataModTargetSpeed);
        if(m_I32DebugLogCounter % 10 == 0 && m_bDebugModeEnabled)
        {
            LOG_INFO("US_ACC [Ext forwards]: modifed speed %f", tmp_dataModTargetSpeed.f32Value);
        }
    }
    else//moving backwards
    {
        TUltrasonicStruct::Data tmp_dataUltrasonicStruct = GetUSInput();
        USweightsRear rearWeights = GetSensorWeightsRear(tmp_dataSteeringAngleData);//takes the steering and the senor into account
        tmp_dataUltrasonicStruct = ComputeWeightedDistanceRear(tmp_dataUltrasonicStruct, rearWeights);//evaluate the importance of the sensor -> not important senors will have a huge value
        RedPercentRear redPercRear= GetRedPerscentRear(tmp_dataUltrasonicStruct, tmp_operationalMode);
        //evaluates the importants related the the current mode
        tmp_dataModTargetSpeed = tmp_dataLastRecTargetSpeed;
        tmp_dataModTargetSpeed = ChooseOutputSpeedRear(redPercRear, tmp_dataModTargetSpeed);//return the speed
        if(m_I32DebugLogCounter % 10 == 0 && m_bDebugModeEnabled) LOG_INFO(cString::Format("UltrasonicACC [Ext Backwards]: Left: %f, Center: %f, Right: %f \n", tmp_dataUltrasonicStruct.tRearLeft.f32Value, tmp_dataUltrasonicStruct.tRearCenter.f32Value, tmp_dataUltrasonicStruct.tRearRight.f32Value));
        if(m_I32DebugLogCounter % 10 == 0 && m_bDebugModeEnabled) LOG_INFO(cString::Format("UltrasonicACC: receivedTargetSpeed: %f, modifiedSpeed: %f",tmp_dataLastRecTargetSpeed.f32Value, tmp_dataModTargetSpeed.f32Value));


    }

    /* limit new output speed to absolute value based on US sensor values and XML */
    if(tmp_DrivingForwards) {
        if(fabsf(tmp_dataLastRecTargetSpeed.f32Value) > fabsf(tmp_dataModTargetSpeed.f32Value))
        {
            tmp_dataLastRecTargetSpeed.f32Value = tmp_dataModTargetSpeed.f32Value;
        }
    }
    else //driving backwards
    {
        if(fabsf(tmp_dataLastRecTargetSpeed.f32Value) > fabsf(tmp_dataModTargetSpeed.f32Value))
        {
            tmp_dataLastRecTargetSpeed.f32Value = -tmp_dataModTargetSpeed.f32Value;
        }
    }

    SetLastModifiedTargetSpeed(tmp_dataLastRecTargetSpeed.f32Value);
    /** transmit target speed **/
    //LOG_INFO("Transmitted speed: %f", tmp_dataLastRecTargetSpeed.f32Value);
    TransmitTargetSpeed(tmp_dataLastRecTargetSpeed);
    //m.unlock();
    m_I32DebugLogCounter++;

    // done
    RETURN_NOERROR;
}

tResult UltrasonicACC::ProcessLaneDetectionLine()
{
    TLaneDetectionLineStruct::Data tmp_laneDetectionLine;
    static tTimeStamp lasttmldl = 0;
    tResult res;
    if(IS_FAILED(res = o_LaneDetectionLineStruct.readPin(m_ReaderLaneDetectionLine, (void *) & tmp_laneDetectionLine, lasttmldl)))
    {
        RETURN_ERROR(res);
    }
    /*if(m_propBDebugCurve)
     LOG_INFO(cString::Format("USACC: send [%d] (%d/%d)(%d/%d)(%d/%d)(%d/%d)",
                                                  tmp_laneDetectionLine.i8IsCurve,
                                                  tmp_laneDetectionLine.i32Point1X,
                                                  tmp_laneDetectionLine.i32Point1Y,
                                                  tmp_laneDetectionLine.i32Point2X,
                                                  tmp_laneDetectionLine.i32Point2Y,
                                                  tmp_laneDetectionLine.i32Point3X,
                                                  tmp_laneDetectionLine.i32Point3Y,
                                                  tmp_laneDetectionLine.i32Point4X,
                                                  tmp_laneDetectionLine.i32Point4Y));*/
    lasttmldl = tmp_laneDetectionLine.ui32ArduinoTimestamp;
    SetLastLaneDetectionLine(tmp_laneDetectionLine);
    //for the ramp
    //stimmt das?
    ProcessValidLane(tmp_laneDetectionLine);
    RETURN_NOERROR;
}

/*********************************************/
/*Increase, reset and get timeoutcounter     */
/*********************************************/
tResult UltrasonicACC::IncreaseTimeoutCounter()
{
    boost::lock_guard<boost::mutex> lock(criticalSectionTimeoutCounter);
    m_I32TimeoutCounter ++;
    RETURN_NOERROR;
}

tResult UltrasonicACC::ResetTimeoutCounter()
{
    boost::lock_guard<boost::mutex> lock(criticalSectionTimeoutCounter);
    m_I32TimeoutCounter = 0;
    RETURN_NOERROR;
}

tUInt32 UltrasonicACC::GetTimeoutCounter()
{
    boost::lock_guard<boost::mutex> lock(criticalSectionTimeoutCounter);
    return m_I32TimeoutCounter;
}

/*********************************************/
/*Process US and Lidar input                 */
/*********************************************/
tResult UltrasonicACC::ProcessUSInput(TUltrasonicStruct::Data tmp_USdata)
{

    /*
         * Maybe in mm
         */

    /* Convert vom cm to mm and discard any values that are smaller than 0.03m (30.0mm)*/
    boost::lock_guard<boost::mutex> lock(criticalSectionUS);
    tmp_USdata.tSideLeft.f32Value *= 10;
    if(tmp_USdata.tSideLeft.f32Value < 30.0) tmp_USdata.tSideLeft.f32Value = 4000.0;

    tmp_USdata.tSideRight.f32Value *= 10;
    if(tmp_USdata.tSideRight.f32Value < 30.0) tmp_USdata.tSideRight.f32Value = 4000.0;

    tmp_USdata.tRearLeft.f32Value *= 10;
    if(tmp_USdata.tRearLeft.f32Value < 30.0) tmp_USdata.tRearLeft.f32Value = 4000.0;

    tmp_USdata.tRearCenter.f32Value *= 10;
    if(tmp_USdata.tRearCenter.f32Value < 30.0) tmp_USdata.tRearCenter.f32Value = 4000.0;

    tmp_USdata.tRearRight.f32Value *= 10;
    if(tmp_USdata.tRearRight.f32Value < 30.0) tmp_USdata.tRearRight.f32Value = 4000.0;

    //set ultrasonic values to 3m (3000mm)
    if(m_BDeactivateACCFilter){

        tmp_USdata.tSideLeft.f32Value = 3000.0;
        tmp_USdata.tSideRight.f32Value = 3000.0;
        tmp_USdata.tRearLeft.f32Value = 3000.0;
        tmp_USdata.tRearCenter.f32Value = 3000.0;
        tmp_USdata.tRearRight.f32Value = 3000.0;

    }

    m_dataUltrasonicTemp = tmp_USdata;

    RETURN_NOERROR;
}

tResult UltrasonicACC::ProcessLIDARInput(TLaserScannerData::Data tmp_LIDARdata)
{
    //set lidar values to 3m
    /*if(m_BDeactivateACCFilter)
    {
        for (uint32_t i = 0; i < (tmp_LIDARdata.ui32Size); i++)
        {
            tmp_LIDARdata.tScanArray[i].f32Radius = 3000;
        }
    }*/


    SetLIDARInput(tmp_LIDARdata);
    UltrasonicACC::LidarObstacles tmp_lIDARObstacles = ObstacleDetectionWithLidar(tmp_LIDARdata);
    SetLIDARObstaclesInput(tmp_lIDARObstacles);
    /*
    if(m_propBDebugOvertakingROI)
    {
        occupancy occ_test;
        //-550 -250 300 1200
        occ_test = CheckROIforObstacles(roiXYOvertakingOncomingTraffic);
        if(occ_test == OCCUPIED_SPACE)LOG_INFO("roiXYOvertakingOncomingTraffic: OCCUPIED_SPACE");
        else if(occ_test == OCCUPIED_STATIC)LOG_INFO("roiXYOvertakingOncomingTraffic: OCCUPIED_STATIC");
    }*/
    //m_dataLidarTmp = tmp_LIDARdata;

    RETURN_NOERROR;
}

/*********************************************/
/*Getter                                     */
/*********************************************/
TUltrasonicStruct::Data UltrasonicACC::GetUSInput()
{

    boost::lock_guard<boost::mutex> lock(criticalSectionUS);

    return m_dataUltrasonicTemp;
}

TLaserScannerData::Data UltrasonicACC::GetLIDARInput()
{

    boost::lock_guard<boost::mutex> lock(criticalSectionSetLidarInputAccess);

    return m_dataLidarTmp;
}

UltrasonicACC::LidarObstacles UltrasonicACC::GetLIDARObstacles()
{

    boost::lock_guard<boost::mutex> lock(criticalSectionSetLidarObstacelsInputAccess);

    return m_dataLidarObstacelesTmp;
}

TSignalValue::Data UltrasonicACC::GetSteeringAngleInput()
{

    boost::lock_guard<boost::mutex> lock(criticalSectionSetSteeringAngle);

    return m_dataSteeringAngleTemp;
}



//checks if the obstacle that caused is the ramp
tBool UltrasonicACC::CheckIfObstacleIsRamp()
{
    /*does nothing if the obstacle is not the ramp
     * transmit feedback if the obstacle is the ramp*/
    UltrasonicACC::LidarObstacles tmp_obstacles = GetLIDARObstacles();
    /*
     * Idee: checken, ob in diesem meter kein Punkt aus der Geraden rausfällt, mit einem gewissen threshold
     * punkt direkt vor dem Auto finden
     *
     * */
    //find point in front of the car
    tFloat32 f32smallestAbsAngle = 90.0;
    tUInt32 ui32IndexOfSmallestAngle = 0;
    tUInt32 ui32ValidValuesCounter = 0;
    for (tUInt32 i = 0; i < tmp_obstacles.ui32Size; i++)
    {
        if(tmp_obstacles.tScanArrayEval[i].ui32ObstacleCounter != 0)
        {
            ui32ValidValuesCounter++;
            if(fabs(tmp_obstacles.tScanArrayEval[i].f32Angle) < f32smallestAbsAngle)
            {
                f32smallestAbsAngle = tmp_obstacles.tScanArrayEval[i].f32Angle;
                ui32IndexOfSmallestAngle = i;
            }
        }
    }

    //LidarPoint pointBeforeSmallestAngle;
    //tBool bPointBeforeSmallestAngleExists = tFalse;
    LidarPoint pointOfSmallestAngle;
    pointOfSmallestAngle.ui32Counter = ui32IndexOfSmallestAngle;
    pointOfSmallestAngle.f32Angle = f32smallestAbsAngle;
    pointOfSmallestAngle.f32Radius = tmp_obstacles.tScanArrayEval[ui32IndexOfSmallestAngle].f32Radius;
    pointOfSmallestAngle.f32y = tmp_obstacles.tScanArrayEval[ui32IndexOfSmallestAngle].f32Radius *sinf((90-tmp_obstacles.tScanArrayEval[ui32IndexOfSmallestAngle].f32Angle)*tFloat32(PI)/180.0f);
    pointOfSmallestAngle.f32x = tmp_obstacles.tScanArrayEval[ui32IndexOfSmallestAngle].f32Radius * cosf((90-tmp_obstacles.tScanArrayEval[ui32IndexOfSmallestAngle].f32Angle)*tFloat32(PI)/180.0f);

    //find point before smallest angle
    //for (tUInt32 i = ui32IndexOfSmallestAngle-1; i > 0; i--)
    /*for (tUInt32 i = ui32IndexOfSmallestAngle+1; i > 0; i++)
    {
        if(tmp_obstacles.tScanArrayEval[i].ui32ObstacleCounter != 0)
        {
            tFloat32 tmp_x = tmp_obstacles.tScanArrayEval[i].f32Radius *sinf((90-tmp_obstacles.tScanArrayEval[i].f32Angle)*tFloat32(PI)/180.0f);
            if((tmp_x > m_propF32XminOfRamp) && (tmp_x < m_propF32XmaxOfRamp))
            {
                pointBeforeSmallestAngle.ui32Counter = i;
                pointBeforeSmallestAngle.f32Angle = tmp_obstacles.tScanArrayEval[i].f32Angle;
                pointBeforeSmallestAngle.f32Radius = tmp_obstacles.tScanArrayEval[i].f32Radius;
                pointBeforeSmallestAngle.f32y = tmp_obstacles.tScanArrayEval[i].f32Radius *sinf((90-tmp_obstacles.tScanArrayEval[i].f32Angle)*tFloat32(PI)/180.0f);
                pointBeforeSmallestAngle.f32x = tmp_obstacles.tScanArrayEval[i].f32Radius * cosf((90-tmp_obstacles.tScanArrayEval[i].f32Angle)*tFloat32(PI)/180.0f);
                bPointBeforeSmallestAngleExists = tTrue;
                break;
            }
            else
            {
                if(m_propBDebugRamp)LOG_INFO("Returned false because the distance to the point before is too long");
                return tFalse;
            }
        }
    }
    if(!bPointBeforeSmallestAngleExists)
    {
        if(m_propBDebugRamp)LOG_INFO("Point before closet Obstacle does not exist");
        return tFalse;
    }
    LOG_INFO("Point before: Counter: %d; Angle: %f, Radius %f", pointBeforeSmallestAngle.ui32Counter, pointBeforeSmallestAngle.f32Angle, pointBeforeSmallestAngle.f32Radius);
    LOG_INFO("X. %f; y: %f", pointBeforeSmallestAngle.f32x, pointBeforeSmallestAngle.f32y);
    //calc slope and check if it is realistic
    //slope of the line of best fit
    //slope = (y2-y1)/(x2-x1)
    tFloat32 f32orignalSlope;
    if(fabs(pointOfSmallestAngle.f32x - pointBeforeSmallestAngle.f32x) > 0.001)
    {
        f32orignalSlope = (pointOfSmallestAngle.f32y - pointBeforeSmallestAngle.f32y)/(pointOfSmallestAngle.f32x - pointBeforeSmallestAngle.f32x);
    }
    else
    {
        if(m_propBDebugRamp)LOG_INFO("Returned because the slope would be inf.");
        LOG_INFO("pointOfSmallestAngle.f32x: %f, pointBeforeSmallestAngle.f32x %f", pointOfSmallestAngle.f32x, pointBeforeSmallestAngle.f32x);
        return tFalse;
    }
    tFloat32 f32minSlope;
    tFloat32 f32maxSlope;
    if(f32orignalSlope > 0)
    {
        if(fabs(m_propF32ThresholdSlopeForRamp) > 0.0001)
        {
            f32minSlope = f32orignalSlope/m_propF32ThresholdSlopeForRamp;
        }
        else
        {
            LOG_WARNING("m_propF32ThresholdSlopeForRamp is %f", m_propF32ThresholdSlopeForRamp);
            f32minSlope =f32orignalSlope;
        }
        f32maxSlope = f32orignalSlope * m_propF32ThresholdSlopeForRamp;
    }
    else
    {
        f32minSlope = f32orignalSlope * m_propF32ThresholdSlopeForRamp;
        if(fabs(m_propF32ThresholdSlopeForRamp) > 0.0001)
        {
            f32maxSlope = f32orignalSlope/m_propF32ThresholdSlopeForRamp;
        }
        else
        {
            LOG_WARNING("m_propF32ThresholdSlopeForRamp is %f", m_propF32ThresholdSlopeForRamp);
            f32maxSlope =f32orignalSlope;
        }
    }
    if(m_propF32ThresholdSlopeForRamp < 0)
    {
        LOG_WARNING("m_propF32ThresholdSlopeForRamp is smaller than 0: %f", m_propF32ThresholdSlopeForRamp);
        tFloat32 tmp = f32maxSlope;
        f32maxSlope = f32minSlope;
        f32minSlope = tmp;
    }
    if(f32maxSlope < f32minSlope)
    {
        LOG_WARNING("MaxSlope is smaller than minslope! The object won't be considerate as a possible ramp!");
        tFloat32 tmp = f32maxSlope;
        f32maxSlope = f32minSlope;
        f32minSlope = tmp;
    }
    LOG_INFO("f32orignalSlope: %f, f32minSlope: %f, f32maxSlope: %f", f32orignalSlope, f32minSlope, f32maxSlope);

    //pointBeforeSmallestAngle.ui32Counter =
    //tFloat32 f32rcos = LidarObstacles.tScanArrayEval[i].f32Radius * cosf((90-LidarObstacles.tScanArrayEval[i].f32Angle) * tFloat(PI) / 180.0f);
    //tFloat32 f32rsin = LidarObstacles.tScanArrayEval[i].f32Radius * sinf((90-LidarObstacles.tScanArrayEval[i].f32Angle) * tFloat(PI) / 180.0f);
    //CalcDistanceWithCos
    //Idee es nur so zu machen vom kleinsten winkel zum nachbarn eine steigung bestimmen und zu schauen, ob die steigung insgesamt sich nicht viel aendert
    //Idee es nur so zu machen vom kleinsten winkel zum nachbarn eine steigung bestimmen und zu schauen, ob die steigung insgesamt sich nicht viel aendert
    */

    TLaserScannerData::Data xyValues;
    xyValues.ui32Size = ui32ValidValuesCounter;
    vector <LidarPoint> vecScanArrayXY;
    //vector <LidarPoint> tmp_vecScanArrayXY;
    //LidarPoint lpLastValidPoint;
    //tBool bFirstPoint = tTrue;
    //tFloat32 f32currentSlope;
    for (tUInt32 i = 0; i < tmp_obstacles.ui32Size; i++)
    {
        if(tmp_obstacles.tScanArrayEval[i].ui32ObstacleCounter != 0)
        {
            LidarPoint scanArray;
            scanArray.ui32Counter = i;
            scanArray.f32Radius = tmp_obstacles.tScanArrayEval[i].f32Radius;
            scanArray.f32Angle = tmp_obstacles.tScanArrayEval[i].f32Angle;
            //xyValues.tScanArray[i].f32Radius = tmp_obstacles.tScanArrayEval[i].f32Radius * cosf((90-tmp_obstacles.tScanArrayEval[i].f32Angle)*tFloat32(PI)/180.0f);//y
            //xyValues.tScanArray[i].f32Angle = tmp_obstacles.tScanArrayEval[i].f32Radius * sinf((90-tmp_obstacles.tScanArrayEval[i].f32Angle)*tFloat32(PI)/180.0f);//x
            scanArray.f32y = tmp_obstacles.tScanArrayEval[i].f32Radius * sinf((90-tmp_obstacles.tScanArrayEval[i].f32Angle)*tFloat32(PI)/180.0f);//y
            //scanArray.f32x = tmp_obstacles.tScanArrayEval[i].f32Radius * sinf((-90+tmp_obstacles.tScanArrayEval[i].f32Angle)*tFloat32(PI)/180.0f);//x
            //scanArray.f32y = tmp_obstacles.tScanArrayEval[i].f32Radius * cosf((90-tmp_obstacles.tScanArrayEval[i].f32Angle)*tFloat32(PI)/180.0f);//y
            scanArray.f32x = tmp_obstacles.tScanArrayEval[i].f32Radius * cosf((90-tmp_obstacles.tScanArrayEval[i].f32Angle)*tFloat32(PI)/180.0f);//x
            if(m_propBDebugRamp)LOG_INFO("Radius %f, Angel: %f; (%f/ %f)", scanArray.f32Radius, scanArray.f32Angle, scanArray.f32x, scanArray.f32y);
            if((scanArray.f32x < m_propF32XmaxOfRamp) && (scanArray.f32x > m_propF32XminOfRamp))
            {

                LOG_INFO("Within the limit!");
                if((scanArray.f32y > (pointOfSmallestAngle.f32y-m_propF32YDiffRamp)) && (scanArray.f32y < (pointOfSmallestAngle.f32y+m_propF32YDiffRamp)))
                {
                    vecScanArrayXY.push_back(scanArray);
                    if(m_propBDebugRamp)LOG_INFO("Punkt (%f, %f) mit r: %f und angle: %f hinzugefuegt!", scanArray.f32x, scanArray.f32y, scanArray.f32Radius, scanArray.f32Angle);
                }
                else
                {


                    if(vecScanArrayXY.size() >= m_propui32MinNumOfPointsRamp)
                    {
                        tFloat32 f32FirstElementRadius = vecScanArrayXY[0].f32Radius;
                        tFloat32 f32FirstElementAngle = vecScanArrayXY[0].f32Angle;
                        tFloat32 f32LastElementRadius = vecScanArrayXY[(vecScanArrayXY.size()-1)].f32Radius;
                        tFloat32 f32LastElementAngle = vecScanArrayXY[(vecScanArrayXY.size()-1)].f32Angle;
                        tFloat32 f32LengthOfTheRamp = CalcDistanceWithCos(f32FirstElementRadius, f32LastElementRadius, (f32LastElementAngle-f32FirstElementAngle));
                        if((f32LengthOfTheRamp > m_propF32MinLengthOfTheRamp)&&(f32LengthOfTheRamp < m_propF32MaxLengthOfTheRamp))
                        {
                            /*for (tUInt32 j = 0; j < tmp_vecScanArrayXY.size()-1; j++)
                            {
                                vecScanArrayXY[j] = tmp_vecScanArrayXY[j];
                            }*/
                            vecScanArrayXY.clear();
                            vecScanArrayXY.shrink_to_fit();
                            break;
                        }
                        if(m_propBDebugRamp)LOG_INFO("The number of points was enough but the distance not: %f", f32LengthOfTheRamp);
                    }
                    else
                    {
                        vecScanArrayXY.clear();
                        vecScanArrayXY.shrink_to_fit();
                    }
                }
                //m_propF32YDiffRamp
                /*if(!bFirstPoint && ((scanArray.f32x-lpLastValidPoint.f32x) > 0.001))
                {
                f32currentSlope = ((scanArray.f32y-lpLastValidPoint.f32y)/(scanArray.f32x-lpLastValidPoint.f32x));
                }
                else
                {
                    f32currentSlope = 99999.9;
                }
                if((f32currentSlope<f32maxSlope) && (f32currentSlope>f32minSlope))
                {
                    if(bFirstPoint)
                    {
                        bFirstPoint = tFalse;
                        vecScanArrayXY.push_back(lpLastValidPoint);
                    }
                    vecScanArrayXY.push_back(scanArray);
                }
                else
                {
                    if(m_propBDebugRamp)LOG_ONFO("(%f,%f) was slope out of boundery: with slope: %f; bounderies: %f, %f", scanArray.f32x, scanArray.f32y, f32currentSlope, f32maxSlope, f32minSlope);
                }
                lpLastValidPoint = scanArray;*/
                //calc slope to the smalles angle point
                /*if(i == pointOfSmallestAngle.ui32Counter)
                {
                    LOG_INFO("Point of smallest Angle");
                    vecScanArrayXY.push_back(scanArray);
                    continue;
                }
                else if(fabs(scanArray.f32x-pointOfSmallestAngle.f32x)>0.001)
                {
                    f32currentSlope = (scanArray.f32y-pointOfSmallestAngle.f32y)/(scanArray.f32x-pointOfSmallestAngle.f32x);
                    LOG_INFO("currentSlope: %f", f32currentSlope);
                }
                else
                {
                    if(m_propBDebugRamp)LOG_WARNING("The x Value of the Point: %f is very close to the x value of the closest point: %f", scanArray.f32x, pointOfSmallestAngle.f32x);
                    f32currentSlope = 99999.9;
                }
                if((f32currentSlope<f32maxSlope) && (f32currentSlope>f32minSlope))
                {
                    LOG_INFO("Slope is within the range");
                    vecScanArrayXY.push_back(scanArray);
                }
                else
                {
                    LOG_INFO("Slope is out of range!");
                }*/
            }


        }
    }

    //checks if vector is empty
    if(vecScanArrayXY.empty())
    {
        //no ramp
        if(m_propBDebugRamp)LOG_WARNING("No obstacle was detected although the car had stopped");
        return tFalse;
    }
    if(vecScanArrayXY.size() < m_propui32MinNumOfPointsRamp)
    {
        if(m_propBDebugRamp)LOG_INFO("No Ramp because there are not enough points: %d", vecScanArrayXY.size());
        return tFalse;
    }


    //checks if the length is too short
    tFloat32 f32LengthOfTheRamp;
    tFloat32 f32FirstElementRadius = vecScanArrayXY[0].f32Radius;
    tFloat32 f32FirstElementAngle = vecScanArrayXY[0].f32Angle;
    tFloat32 f32LastElementRadius = vecScanArrayXY[(vecScanArrayXY.size()-1)].f32Radius;
    tFloat32 f32LastElementAngle = vecScanArrayXY[(vecScanArrayXY.size()-1)].f32Angle;

    f32LengthOfTheRamp = CalcDistanceWithCos(f32FirstElementRadius, f32LastElementRadius, (f32LastElementAngle-f32FirstElementAngle));
    if(m_propBDebugRamp)LOG_INFO("f32LengthOfTheRamp: %f", f32LengthOfTheRamp);


    if(f32LengthOfTheRamp < m_propF32MinLengthOfTheRamp)
    {
        if(m_propBDebugRamp)LOG_INFO("The detcted Obstacle is too short: %f", f32LengthOfTheRamp);
        if(m_propBDebugRamp)LOG_INFO("First: r: %f, angle: %f", f32FirstElementRadius, f32FirstElementAngle);
        if(m_propBDebugRamp)LOG_INFO("Last: r: %f, angle: %f", f32LastElementRadius, f32LastElementAngle);
        return tFalse;
    }

    if(f32LengthOfTheRamp > m_propF32MaxLengthOfTheRamp)
    {
        if(m_propBDebugRamp)LOG_INFO("The detcted Obstacle is too long: %f", f32LengthOfTheRamp);
        return tFalse;
    }

    /*tFloat32 slope = 0.0f;
    tFloat32 yInt = 0.0f;
    tBool bWorked = fitPoints(vecScanArrayXY, slope, yInt);
    if(m_propBDebugRamp)LOG_INFO("Slope: %f", slope);
    if(m_propBDebugRamp)LOG_INFO("yInt: %f", yInt);
    if(bWorked == false)
    {
        return tFalse;
    }
    if(fabs(slope) > m_propF32ThresholdSlopeForRamp)
    {
        //no ramp
        if(m_propBDebugRamp)LOG_INFO("The slope of the object is too great. No ramp!");
        return tFalse;
    }

    for (tUInt32 i = 0; i < vecScanArrayXY.size(); i++)
    {
        //y = slope * x + yInt
        if(fabs((slope*vecScanArrayXY[i].f32Angle+yInt)-vecScanArrayXY[i].f32Radius)>m_propF32ThresholdDifferenzYNotOnLine)
        {
            //evtl erst nach ein paar Punkten
            tFloat32 f32Diff = (slope*vecScanArrayXY[i].f32Angle+yInt)-vecScanArrayXY[i].f32Radius;
            if(m_propBDebugRamp)LOG_INFO("The difference between the line of best fit and the point is to big:");
            if(m_propBDebugRamp)LOG_INFO("Difference: %f; X-Point: %f; Y-point: %f", f32Diff, vecScanArrayXY[i].f32Angle, vecScanArrayXY[i].f32Radius);
            return tFalse;
        }
    }*/
    /*shape of the object is a possible ramp*/
    /*chech if there is a lane:*/
    if(!GetValidLane())
    {
        if(m_propBDebugRamp)LOG_INFO("No valid lane was found!");
        return tFalse;
    }
    if(m_propBDebugRamp)LOG_INFO("Valid lane was found!");
    return tTrue;




}

tBool UltrasonicACC::CheckForGuardRail()
{
    UltrasonicACC::LidarObstacles tmp_obstacles = GetLIDARObstacles();
    tFloat32 f32RadiusPos = 0;
    tFloat32 f32RadiusNeg = 0;
    tFloat32 f32AnglePos = 99;
    tFloat32 f32AngleNeg = -99;

    for(tUInt32 i = 0; i < tmp_obstacles.ui32Size; i++)
    {
        if(tmp_obstacles.tScanArrayEval[i].ui32ObstacleCounter != 0)
        {
            if((tmp_obstacles.tScanArrayEval[i].f32Angle < 0.0f) && (tmp_obstacles.tScanArrayEval[i].f32Angle > f32AngleNeg))
            {
                f32AngleNeg = tmp_obstacles.tScanArrayEval[i].f32Angle;
                f32RadiusNeg = tmp_obstacles.tScanArrayEval[i].f32Radius;
            }
            else if((tmp_obstacles.tScanArrayEval[i].f32Angle > 0.0f) && (tmp_obstacles.tScanArrayEval[i].f32Angle < f32AnglePos))
            {
                f32AnglePos = tmp_obstacles.tScanArrayEval[i].f32Angle;
                f32RadiusPos = tmp_obstacles.tScanArrayEval[i].f32Radius;
            }
        }
    }
    if((fabs(f32AnglePos -99)< 0.01) || (fabs(f32AngleNeg +99)< 0.01))
    {
        return tFalse;
    }
    tFloat32 f32Distance = sqrt(pow(f32RadiusPos, 2)+pow(f32RadiusNeg, 2)-2*f32RadiusNeg*f32AnglePos*cosf(f32AnglePos-f32RadiusNeg));
    if((f32Distance<m_propF32MaxLengthOfTheRamp )&&(f32Distance>m_propF32MinLengthOfTheRamp))
    {
        return tTrue;
    }

    return tFalse;

}

//vector <LidarPoint> vecScanArrayXY
//find line of best fit
//ch&pts
tBool UltrasonicACC::fitPoints(const std::vector<LidarPoint> &pts, tFloat32 &slope, tFloat32 &yInt)
{
    int nPoints = pts.size();
    if(nPoints < 2)
    {
        // Fail: infinitely many lines passing through this single point
        return false;
    }
    tFloat32 sumX=0, sumY=0, sumXY=0, sumX2=0;
    for (int i=0; i<nPoints;i++)
    {
        sumX += pts[i].f32Angle;
        sumY += pts[i].f32Radius;
        sumXY += pts[i].f32Angle * pts[i].f32Radius;
        sumX2 += pts[i].f32Angle * pts[i].f32Angle;
    }
    tFloat32 xMean = sumX / nPoints;
    tFloat32 yMean = sumY / nPoints;
    tFloat32 denominator = sumX2 - sumX * xMean;
    // You can tune the eps (1e-7) below for your specific task
    if( std::fabs(denominator) < 1e-7 )
    {
        // Fail: it seems a vertical line
        return false;
    }
    slope = (sumXY - sumX * yMean) / denominator;
    yInt = yMean - slope * xMean;
    return true;
}

/* calculates temporary sensor weight value */
tFloat32 UltrasonicACC::CalcTmpWeight(tInt32 sensorAngleSetting, tFloat32 tmp_steeringAngle, tUInt32 scalingFactor)
{

    tFloat32 tmpWeight = -1.0f;
    tmpWeight = 1.0f - (fabsf(tmp_steeringAngle - static_cast<tFloat32>(sensorAngleSetting)) * (static_cast<tFloat32>(scalingFactor)) / 1000.0);
    /* if sensor is too far away, tmp_weight will get negative */
    if(tmpWeight < 0.0f){
        tmpWeight = 0.0f;
    }
    else if(tmpWeight > 1.0f)
    {
        tmpWeight = 1.0;
    }

    return tmpWeight;
}

UltrasonicACC::pointOnTwoLanes UltrasonicACC::GetLastProcessedPoints()
{

    boost::lock_guard<boost::mutex> lock(criticalSectionLastProcessedLines);

    return m_dataLastProcessedPoints;
}

tResult UltrasonicACC::SetLastProcessPoints(UltrasonicACC::pointOnTwoLanes processedLanes)
{

    boost::lock_guard<boost::mutex> lock(criticalSectionLastProcessedLines);

    m_dataLastProcessedPoints = processedLanes;
    RETURN_NOERROR;
}

tFloat32 UltrasonicACC::CalcClosestRelevantObstacle(UltrasonicACC::LidarObstacles tmp_dataLidarObstacle)
{//ATTENTION: if the obstacle is too close the lidar does return 0 -> we cannot detect the obstacle

    tFloat32 f32ClosestObstacle = 9999.0;
    UltrasonicACC::LidarObstacles tmp_Obstacle = tmp_dataLidarObstacle;

    TLaneDetectionLineStruct::Data lineStruct = GetLastLaneDetectionLine();

    if(lineStruct.i8IsCurve == 0)
    {//does not considersate the steering angle
        if(GetOperationalMode() != MtP_MODE_ACTIVE)
        {
            SetOperationalMode(DRIVING_MODE_ACTIVE);
        }
        //LOG_INFO("Driving Forward!");

        TLaserScannerData::Data laserOutput;
        laserOutput.ui32Size = 0;
        laserOutput.tScanArray[0].f32Radius = 0.0;
        laserOutput.tScanArray[0].f32Angle = 0.0;

        for (uint32_t i=0; i<tmp_Obstacle.ui32Size; i++)
        {
            if (tmp_Obstacle.tScanArrayEval[i].ui32ObstacleCounter != 0)
            {
                if(tmp_Obstacle.tScanArrayEval[i].f32Radius > (m_propF32CarWidth/2.0))
                {
                    tFloat32 f32AngleThresholdForDetection = CalcAngleFromDistance(tmp_Obstacle.tScanArrayEval[i].f32Radius);
                    tFloat32 f32AngleMin = (-f32AngleThresholdForDetection);//+(steeringAngleData.f32Value*m_propF32ScaleSteering);
                    tFloat32 f32AngleMax = (f32AngleThresholdForDetection);//+(steeringAngleData.f32Value*m_propF32ScaleSteering);
                    if((tmp_dataLidarObstacle.tScanArrayEval[i].f32Angle <= (f32AngleMin)) ||
                            (tmp_Obstacle.tScanArrayEval[i].f32Angle >= f32AngleMax) )
                    {
                        tmp_Obstacle.tScanArrayEval[i].ui32ObstacleCounter=0;
                    }
                    else if((tmp_Obstacle.tScanArrayEval[i].ui32ObstacleCounter != 0))
                    {
                        if(m_propBLaserOutputDrive && m_propBLaserOutputGeneral)
                        {
                            laserOutput.tScanArray[laserOutput.ui32Size].f32Radius = tmp_Obstacle.tScanArrayEval[i].f32Radius;
                            laserOutput.tScanArray[laserOutput.ui32Size].f32Angle = AngleBackCompensation(tmp_Obstacle.tScanArrayEval[i].f32Angle);
                            laserOutput.ui32Size++;
                        }
                        if(f32ClosestObstacle>tmp_Obstacle.tScanArrayEval[i].f32Radius)
                        {
                            f32ClosestObstacle = tmp_Obstacle.tScanArrayEval[i].f32Radius;
                            if(m_propBDebugFirstStart) LOG_INFO("Closed Obstacle at: %f", tmp_Obstacle.tScanArrayEval[i].f32Radius);
                        }
                    }
                }
                else
                {
                    if ((tmp_Obstacle.tScanArrayEval[i].f32Radius < (m_propF32CarWidth/2.0)-20.0))
                    {
                        if(m_propBLaserOutputDrive && m_propBLaserOutputGeneral)
                        {
                            laserOutput.tScanArray[laserOutput.ui32Size].f32Radius = tmp_Obstacle.tScanArrayEval[i].f32Radius;
                            laserOutput.tScanArray[laserOutput.ui32Size].f32Angle = AngleBackCompensation(tmp_Obstacle.tScanArrayEval[i].f32Angle);
                            laserOutput.ui32Size++;
                        }

                        if(f32ClosestObstacle>tmp_Obstacle.tScanArrayEval[i].f32Radius)
                        {
                            f32ClosestObstacle = tmp_Obstacle.tScanArrayEval[i].f32Radius;
                            if(m_propBDebugStraight)LOG_INFO("Closed Obstacle at: %f; Catched with execption", tmp_Obstacle.tScanArrayEval[i].f32Radius);
                        }
                    }

                }
            }

        }
        //if(m_propBLaserOutputDrive && m_propBLaserOutputGeneral){TransmitRelevantLaser(laserOutput);}
        if(m_propBDebugStraight)LOG_INFO("Closed Obstacle at: %f", f32ClosestObstacle);
        //LOG_INFO("uSACC: straight: Closed Obstacle at: %f", f32ClosestObstacle);
    }
    else //taking a bend
    {
        if(GetOperationalMode() != MtP_MODE_ACTIVE)
        {
            SetOperationalMode(DRIVING_MODE_CURVE);
        }


        pointOnTwoLanes lanes = ProcessLaneDetectionInput(lineStruct);
        if(m_propBDebugCurve)LOG_INFO("USACC: In my system: (%f,%f);(%f,%f);(%f,%f);(%f,%f)", lanes.f32x1line1, lanes.f32y1line1,
                                      lanes.f32x2line1, lanes.f32y2line1, lanes.f32x1line2, lanes.f32y1line2, lanes.f32x2line2, lanes.f32y2line2);
        if(m_propBDebugCurve)LOG_INFO("USACC: LD transmitted: [%d] (%d/%d)(%d/%d)(%d/%d)(%d/%d)",
                                      lineStruct.i8IsCurve,
                                      lineStruct.i32Point1X,
                                      lineStruct.i32Point1Y,
                                      lineStruct.i32Point2X,
                                      lineStruct.i32Point2Y,
                                      lineStruct.i32Point3X,
                                      lineStruct.i32Point3Y,
                                      lineStruct.i32Point4X,
                                      lineStruct.i32Point4Y);
        if(m_propBDebugCurve)LOG_INFO("In else taking a bend");
        //func for processing the input value from lane detection
        //tBool BflagForObstacleRightBeforeTheCar = tFalse;
        //if(m_propBDebugCurve)LOG_INFO("Right hand bend!!!!");
        roiXY currentROIstart;
        currentROIstart.f32y_min = 0.0;
        currentROIstart.f32y_max = m_propF32ThresholdVisualRangeBend;
        currentROIstart.f32x_min = -(m_propF32CarWidth/2)+m_propF32ThresholdReducedVisualRangeOneSide;
        currentROIstart.f32x_max = (m_propF32CarWidth/2)-m_propF32ThresholdReducedVisualRangeOneSide;
        tFloat32 f32tmp_Obstacle = IsObjectInBox(currentROIstart, tmp_Obstacle);

        if(f32tmp_Obstacle <(m_propF32ThresholdRelObstBend+m_propF32HighBoxBend))
        {
            f32ClosestObstacle = f32tmp_Obstacle;
            if(m_propBDebugCurve)LOG_INFO("USACC: [CalcClosestRelevantObstacle]: In Bend right before the car: Closest Obstacle:, %f", f32ClosestObstacle);
            return f32ClosestObstacle;

        }
        /*
        if(lineStruct.i8IsCurve == 1)//right hand bend
        {
            //tUInt32 Counter = static_cast<tUInt32>(300.0/m_propF32HighBoxBend);
            for (tUInt32 Counter = 3; Counter < m_UI32MaxCounter; Counter++)
            {
                //if(m_propBDebugCurve)LOG_INFO("Right Hand Bend: MaxCounter: %u; Counter %u", m_UI32MaxCounter, Counter);
                roiXY currentROI;
                currentROI.f32y_min = (static_cast<tFloat32> (Counter))*m_propF32HighBoxBend;
                currentROI.f32y_max = currentROI.f32y_min + m_propF32HighBoxBend;

                currentROI.f32x_min =  GetLinearInterpolatedValueBend(currentROI.f32y_max,lanes.f32x1line1,lanes.f32y1line1,lanes.f32x2line1,lanes.f32y2line1)+m_propF32CamOffsety;//left hand side lane
                currentROI.f32x_max =  GetLinearInterpolatedValueBend(currentROI.f32y_min,lanes.f32x1line2,lanes.f32y1line2,lanes.f32x2line2,lanes.f32y2line2)+m_propF32CamOffsety;//right hand siode lane
                if(currentROI.f32y_min>currentROI.f32y_max)LOG_WARNING("USACC: right hand Bend: x_min (%f)is grater than x_max(%f)", currentROI.f32x_min, currentROI.f32x_max);
                if(currentROI.f32x_min>currentROI.f32x_max)LOG_WARNING("USACC: right hand Bend: y_min (%f)is grater than y_max(%f)", currentROI.f32y_min, currentROI.f32y_max);


                if(currentROI.f32x_min < (-(m_propF32CarWidth/2.0)))
                {
                    if(m_propBDebugCurve)LOG_WARNING("USACC: right hand bend: ROI would be on the left hand side of the car");
                    currentROI.f32x_min = (-(m_propF32CarWidth/2.0));
                }
                if(currentROI.f32x_max < (m_propF32CarWidth/2.0))
                {
                    if(m_propBDebugCurve)LOG_WARNING("USACC: right hand bend: ROI would not cover all the width of the car");
                    currentROI.f32x_max = ((m_propF32CarWidth/2.0));
                }
                if(m_propBDebugCurve)
                {
                    LOG_INFO("USACC: right hand bend Counter: %d", Counter);
                    LOG_INFO("USACC: x_min: %f; x_max: %f; y_min %f; y_max %f",currentROI.f32x_min,currentROI.f32x_max, currentROI.f32y_min, currentROI.f32y_max);
                }
                tFloat32 f32tmp_Obstacle = IsObjectInBox(currentROI, tmp_Obstacle);
                //LOG_INFO("[%d]: obstacle: %f", Counter, f32tmp_Obstacle);
                if(f32tmp_Obstacle <(m_propF32ThresholdRelObstBend+m_propF32HighBoxBend))
                {
                    f32ClosestObstacle = f32tmp_Obstacle;
                    if(m_propBDebugCurve)LOG_INFO("USACC: [CalcClosestRelevantObstacle]: In Bend: Closest Obstacle:, %f", f32ClosestObstacle);
                    break;
                }
                if(m_propBDebugCurve)LOG_INFO("Bend right: at the end: closest obstacle: %f", f32ClosestObstacle);


                //rectangle(m_MatOutput, Rect(430 + currentROI.f32x_min, 480 - currentROI.f32y_max/3,
                //                            abs(currentROI.f32x_max/3 - currentROI.f32x_min/3),
                //                            abs(currentROI.f32y_max/3 - currentROI.f32y_min/3)), cv::Scalar(255, 100, 30), 4);
            }

        }
        else if (lineStruct.i8IsCurve == -1)//left hand bend
        {
            {
                for (tUInt32 Counter = 3; Counter < m_UI32MaxCounter; Counter++)
                {
                    if(m_propBDebugCurve)LOG_INFO("Left Hand Bend MaxCounter: %u; Counter %u", m_UI32MaxCounter, Counter);
                    roiXY currentROI;
                    currentROI.f32y_min = (static_cast<tFloat32> (Counter))*m_propF32HighBoxBend;
                    currentROI.f32y_max = currentROI.f32y_min + m_propF32HighBoxBend;

                    currentROI.f32x_min =  GetLinearInterpolatedValueBend(currentROI.f32y_min,lanes.f32x1line2,lanes.f32y1line2, lanes.f32x2line2,lanes.f32y2line2);//left hand lane
                    currentROI.f32x_max =  GetLinearInterpolatedValueBend(currentROI.f32y_max,lanes.f32x1line1,lanes.f32y1line1,lanes.f32x2line1,lanes.f32y2line1);//right hand lane
                    if(currentROI.f32y_min>currentROI.f32y_max)LOG_WARNING("USACC: Bend: x_min (%f)is grater than x_max(%f)", currentROI.f32x_min, currentROI.f32x_max);
                    if(currentROI.f32x_min>currentROI.f32x_max)LOG_WARNING("USACC: Bend: y_min (%f)is grater than y_max(%f)", currentROI.f32y_min, currentROI.f32y_max);

                    if(currentROI.f32x_max > (m_propF32CarWidth/2.0))
                    {
                        if(m_propBDebugCurve)LOG_WARNING("USACC: left hand bend: ROI would be on the right hand side of the car: x_max%f", currentROI.f32x_max);
                        currentROI.f32x_max = (m_propF32CarWidth/2.0);
                    }
                    if(currentROI.f32x_min > (-(m_propF32CarWidth/2.0)))
                    {
                        if(m_propBDebugCurve)LOG_WARNING("USACC: left hand bend: ROI would not cover all the width of the car: x_min: %f", currentROI.f32x_min);
                        currentROI.f32x_min = (-(m_propF32CarWidth/2.0));
                    }
                    if(m_propBDebugCurve)
                    {
                        LOG_INFO("USACC: left hand bend Counter: %d", Counter);
                        LOG_INFO("USACC: x_min: %f; x_max: %f; y_min %f; y_max %f",currentROI.f32x_min,currentROI.f32x_max, currentROI.f32y_min, currentROI.f32y_max);
                    }
                    tFloat32 f32tmp_Obstacle=IsObjectInBox(currentROI, tmp_Obstacle);
                    if(f32tmp_Obstacle<(m_propF32ThresholdRelObstBend+m_propF32HighBoxBend))
                    {
                        f32ClosestObstacle = f32tmp_Obstacle;
                        break;
                    }
                    if(m_propBDebugCurve)LOG_INFO("Bend left: closest obstacle: %f", f32ClosestObstacle);
                }

            }
        }*/

        //if(m_propBDebugCurve) LOG_INFO("Taking a bend!");

    }



    if(f32ClosestObstacle <= m_propF32ObjectThresholdDetection)
    {
        if(m_bDebugModeEnabled) LOG_INFO("Objekt erkannt!!!!! Bei %f", f32ClosestObstacle);
    }
    if(m_propBDebugCurve)LOG_INFO("Objekt erkannt!!!!! Bei %f", f32ClosestObstacle);
    return f32ClosestObstacle;
}


//checks the ROI for Obstacels calls IsObjectInROI
UltrasonicACC::occupancy UltrasonicACC::CheckROIforObstacles(UltrasonicACC::roiXY roi_XY)
{

    boost::lock_guard<boost::mutex> lock(criticalSectionCheckROIforObstacles);

    /* necessary variables for region-checking */
    /* counters indicating occupied pixel */
    UltrasonicACC::LidarObstacles tmp_lidarObsticels = GetLIDARObstacles();
    tUInt32 ui32OccupiedCoordinateCounter = IsObjectInROI(roi_XY, tmp_lidarObsticels);
    ResetSizeOfOutputLaser();
    if(ui32OccupiedCoordinateCounter != 0)
    {
        m_UI32FreeROICounter = 0;
        m_UI32OccupiedROICounter++;
        if(m_UI32OccupiedROICounter >= m_propUI32OccupiedROIBoundary)
        {
            m_UI32OccupiedROICounter = 0;

            return OCCUPIED_STATIC;
        }

        return OCCUPIED_SPACE;//at least on occupied region -> it continues checking
    }
    else
    {
        m_UI32FreeROICounter++;
        m_UI32OccupiedROICounter = 0;
        if (m_UI32FreeROICounter >= m_propUI32FreeROIBoundary)
        {
            m_UI32FreeROICounter = 0;

            return FREE_SPACE;
        }

        return STATE_PENDING;
    }

    return ERROR;

}

tResult UltrasonicACC::ProcessROI(UltrasonicACC::roiXY roi_XY1)
{

    /* call method to check traffic on actual own original lane after overtake */
    occupancy occupancy_status = INITIALILZED;
    //occupancy_status = CheckROIforObstacles(roiXYOvertakingOriginalLane);
    //tInt64 i64tmp_GlobalCounter = 0;
    for (uint i =0; i < m_propUI32EndOfForLoopToCheckROI; i++)
    {
        //i64tmp_GlobalCounter = GetGlobalCounter();
        occupancy_status = CheckROIforObstacles(roi_XY1);
        if((occupancy_status == FREE_SPACE)||(occupancy_status == ERROR
                                              || occupancy_status == INITIALILZED) || (occupancy_status == OCCUPIED_STATIC))
        {
            break;
        }
        /*LOG_INFO("Vor der while!");
        while(GetGlobalCounter() == i64tmp_GlobalCounter)
        {

            continue;
        }
        LOG_INFO("Proessed Check ROI for obstacles!");*/
    }
    if ((occupancy_status == FREE_SPACE) || (occupancy_status == STATE_PENDING)){
        RETURN_IF_FAILED(TransmitFeedbackNoObstacle());
        SetRunningState(tFalse);
        if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 1) {
            LOG_WARNING(
                        cString::Format(
                            "ObstacleDetec: No oncoming obstacle detected (in driving mode, overtake), free space!"));
            m_ui32DebugLogEventToggle = 1;
        }
    } else if (occupancy_status == ERROR
               || occupancy_status == INITIALILZED) {
        LOG_ERROR(
                    cString::Format(
                        "ObstacleDetec: Error occurred during region-check 'DRIVING - ONCOMING'. Return-status: %d",
                        occupancy_status));
    } else if (occupancy_status == OCCUPIED_SPACE) {
        if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 2) {
            LOG_WARNING(
                        cString::Format(
                            "ObstacleDetec: Oncoming obstacle detected (in driving mode, overtake)!"));
            m_ui32DebugLogEventToggle = 2;
        }
    } else if (occupancy_status == OCCUPIED_STATIC) {
        RETURN_IF_FAILED(TransmitFeedbackStaticObstacle());
        SetRunningState(tFalse);
        if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 3) {
            LOG_WARNING(
                        cString::Format(
                            "ObstacleDetec: Static obstacle detected! (in cross-parking mode, cross pullout)"));
            m_ui32DebugLogEventToggle = 3;
        }
    }

    RETURN_NOERROR;
}
UltrasonicACC::occupancy UltrasonicACC::CheckROIforObstacles(UltrasonicACC::roiXY roi_XY1, UltrasonicACC::roiXY roi_XY2)
{

    boost::lock_guard<boost::mutex> lock(criticalSectionCheckROIforObstacles);

    UltrasonicACC::LidarObstacles tmp_lidarObsticels = GetLIDARObstacles();
    tUInt32 ui32OccupiedCoordinateCounter1 = IsObjectInROI(roi_XY1, tmp_lidarObsticels);
    tUInt32 ui32OccupiedCoordinateCounter2 = IsObjectInROI(roi_XY2, tmp_lidarObsticels);

    ResetSizeOfOutputLaser();
    if(ui32OccupiedCoordinateCounter1 || ui32OccupiedCoordinateCounter2)
    {
        m_UI32FreeROICounter = 0;
        m_UI32OccupiedROICounter++;
        if(m_UI32OccupiedROICounter >= m_propUI32OccupiedROIBoundary)
        {
            m_UI32OccupiedROICounter = 0;


            return OCCUPIED_STATIC;
        }

        return OCCUPIED_SPACE;//at least on occupied region -> it continues checking
    }
    else
    {
        m_UI32FreeROICounter++;
        m_UI32OccupiedROICounter = 0;
        if (m_UI32FreeROICounter >= m_propUI32FreeROIBoundary)
        {
            m_UI32FreeROICounter = 0;

            return FREE_SPACE;
        }

        return STATE_PENDING;
    }

    return ERROR;
}

tResult UltrasonicACC::ProcessROI(UltrasonicACC::roiXY roi_XY1, UltrasonicACC::roiXY roi_XY2)
{
    /* call method to check traffic on actual own original lane after overtake */
    occupancy occupancy_status = INITIALILZED;
    //occupancy_status = CheckROIforObstacles(roiXYOvertakingOriginalLane);
    for (uint i =0; i < m_propUI32EndOfForLoopToCheckROI; i++)
    {
        occupancy_status = CheckROIforObstacles(roi_XY1, roi_XY2);
        if((occupancy_status == FREE_SPACE)||(occupancy_status == ERROR
                                              || occupancy_status == INITIALILZED) || (occupancy_status == OCCUPIED_STATIC))
        {
            break;
        }
    }
    if ((occupancy_status == FREE_SPACE) || (occupancy_status == STATE_PENDING)){
        RETURN_IF_FAILED(TransmitFeedbackNoObstacle());
        SetRunningState(tFalse);
        if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 1) {
            LOG_WARNING(
                        cString::Format(
                            "ObstacleDetec: No oncoming obstacle detected (in driving mode, overtake), free space!"));
            m_ui32DebugLogEventToggle = 1;
        }
    } else if (occupancy_status == ERROR
               || occupancy_status == INITIALILZED) {
        LOG_ERROR(
                    cString::Format(
                        "ObstacleDetec: Error occurred during region-check 'DRIVING - ONCOMING'. Return-status: %d",
                        occupancy_status));
    } else if (occupancy_status == OCCUPIED_SPACE) {
        if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 2) {
            LOG_WARNING(
                        cString::Format(
                            "ObstacleDetec: Oncoming obstacle detected (in driving mode, overtake)!"));
            m_ui32DebugLogEventToggle = 2;
        }
    } else if (occupancy_status == OCCUPIED_STATIC) {
        RETURN_IF_FAILED(TransmitFeedbackStaticObstacle());
        SetRunningState(tFalse);
        if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 3) {
            LOG_WARNING(
                        cString::Format(
                            "ObstacleDetec: Static obstacle detected! (in cross-parking mode, cross pullout)"));
            m_ui32DebugLogEventToggle = 3;
        }
    }

    RETURN_NOERROR;
}

UltrasonicACC::occupancy UltrasonicACC::CheckROIforObstacles(UltrasonicACC::roiXY roi_XY1, UltrasonicACC::roiXY roi_XY2, UltrasonicACC::roiXY roi_XY3)
{

    boost::lock_guard<boost::mutex> lock(criticalSectionCheckROIforObstacles);

    UltrasonicACC::LidarObstacles tmp_lidarObsticels = GetLIDARObstacles();
    tUInt32 ui32OccupiedCoordinateCounter1 = IsObjectInROI(roi_XY1, tmp_lidarObsticels);
    tUInt32 ui32OccupiedCoordinateCounter2 = IsObjectInROI(roi_XY2, tmp_lidarObsticels);
    tUInt32 ui32OccupiedCoordinateCounter3 = IsObjectInROI(roi_XY3, tmp_lidarObsticels);

    ResetSizeOfOutputLaser();

    if(ui32OccupiedCoordinateCounter1 || ui32OccupiedCoordinateCounter2 || ui32OccupiedCoordinateCounter3)
    {
        if(m_propBDebugROI)
        {
            if(ui32OccupiedCoordinateCounter1)LOG_INFO("Oncoming traffic!");
            else if(ui32OccupiedCoordinateCounter2)LOG_INFO("Traffic Right!");
            else if(ui32OccupiedCoordinateCounter3)LOG_INFO("Traffic Left!");
            else LOG_WARNING("USACC: [CheckROIforObstacles]: Something went terribly wrong!");
        }
        m_UI32FreeROICounter = 0;
        m_UI32OccupiedROICounter++;
        if(m_UI32OccupiedROICounter >= m_propUI32OccupiedROIBoundary)
        {
            m_UI32OccupiedROICounter = 0;

            return OCCUPIED_STATIC;
        }

        return OCCUPIED_SPACE;//at least on occupied region -> it continues checking
    }
    else
    {
        m_UI32FreeROICounter++;
        m_UI32OccupiedROICounter = 0;
        if (m_UI32FreeROICounter >= m_propUI32FreeROIBoundary)
        {
            m_UI32FreeROICounter = 0;

            return FREE_SPACE;
        }

        return STATE_PENDING;
    }

    return ERROR;
}

tResult UltrasonicACC::ProcessROI(UltrasonicACC::roiXY roi_XY1, UltrasonicACC::roiXY roi_XY2, UltrasonicACC::roiXY roi_XY3)
{

    /* call method to check traffic on actual own original lane after overtake */
    occupancy occupancy_status = INITIALILZED;
    //occupancy_status = CheckROIforObstacles(roiXYOvertakingOriginalLane);
    for (uint i =0; i < m_propUI32EndOfForLoopToCheckROI; i++)
    {
        occupancy_status = CheckROIforObstacles(roi_XY1, roi_XY2, roi_XY3);
        if(m_propBDebugActionCommand)LOG_INFO("USACC:  [ProcessROI]: status: %d", occupancy_status);
        if((occupancy_status == FREE_SPACE)||(occupancy_status == ERROR
                                              || occupancy_status == INITIALILZED) || (occupancy_status == OCCUPIED_STATIC))
        {
            break;
        }
    }
    if ((occupancy_status == FREE_SPACE) || (occupancy_status == STATE_PENDING)){
        RETURN_IF_FAILED(TransmitFeedbackNoObstacle());
        SetRunningState(tFalse);
        if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 1) {
            LOG_WARNING(
                        cString::Format(
                            "ObstacleDetec: No oncoming obstacle detected (in driving mode, overtake), free space!"));
            m_ui32DebugLogEventToggle = 1;
        }
    } else if (occupancy_status == ERROR
               || occupancy_status == INITIALILZED) {
        LOG_ERROR(
                    cString::Format(
                        "ObstacleDetec: Error occurred during region-check 'DRIVING - ONCOMING'. Return-status: %d",
                        occupancy_status));
    } else if (occupancy_status == OCCUPIED_SPACE) {
        if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 2) {
            LOG_WARNING(
                        cString::Format(
                            "ObstacleDetec: Oncoming obstacle detected (in driving mode, overtake)!"));
            m_ui32DebugLogEventToggle = 2;
        }
    } else if (occupancy_status == OCCUPIED_STATIC) {
        RETURN_IF_FAILED(TransmitFeedbackStaticObstacle());
        SetRunningState(tFalse);
        if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 3) {
            LOG_WARNING(
                        cString::Format(
                            "ObstacleDetec: Static obstacle detected! (in cross-parking mode, cross pullout)"));
            m_ui32DebugLogEventToggle = 3;
        }
    }

    RETURN_NOERROR;
}

UltrasonicACC::USweightsRear UltrasonicACC::GetSensorWeightsRear(TSignalValue::Data steeringAngleData)
{

    /* car is driving backwards */
    USweightsRear rearWeightsStruct;
    rearWeightsStruct.us_rearLeft = CalcTmpWeight((USACC_SENSOR_REAR_LEFT),steeringAngleData.f32Value, m_UI32ScalingFactorUSRear);
    rearWeightsStruct.us_rearCenter = CalcTmpWeight(USACC_SENSOR_REAR_CENTER,steeringAngleData.f32Value, m_UI32ScalingFactorUSRear);
    rearWeightsStruct.us_rearRight = CalcTmpWeight((USACC_SENSOR_REAR_RIGHT),steeringAngleData.f32Value, m_UI32ScalingFactorUSRear);

    if(m_I32DebugLogCounter % 10 == 0 && m_bDebugModeEnabled) LOG_WARNING(cString::Format("cUltrasonicACC:: usRearLeft %f, usRearCenter %f, usRearRight %f",
                                                                                          rearWeightsStruct.us_rearLeft, rearWeightsStruct.us_rearCenter, rearWeightsStruct.us_rearRight));
    return rearWeightsStruct;
}

/*********************************************/
/*Getter and Setter                          */
/*********************************************/
tResult UltrasonicACC::SetOperationalMode(currentOperationalMode new_mode)
{

    boost::lock_guard<boost::mutex> lock(criticalSectionOperationalMode);
    operationalMode = new_mode;

    RETURN_NOERROR;
}
tResult UltrasonicACC::SetSteeringAngle(TSignalValue::Data steering)
{

    boost::lock_guard<boost::mutex> lock(criticalSectionSetSteeringAngle);
    m_dataSteeringAngleTemp.f32Value = steering.f32Value+m_propF32SensorOffsetSteering;
    m_dataSteeringAngleTemp.ui32ArduinoTimestamp = steering.ui32ArduinoTimestamp;

    RETURN_NOERROR;
}


TLaneDetectionLineStruct::Data UltrasonicACC::GetLastLaneDetectionLine()
{

    boost::lock_guard<boost::mutex> lock(criticalSectionLaneDetectionLine);

    return m_dataLastLane;
}

UltrasonicACC::currentOperationalMode UltrasonicACC::GetOperationalMode()
{

    boost::lock_guard<boost::mutex> lock(criticalSectionOperationalMode);

    return operationalMode;
}

tBool UltrasonicACC::GetRunningState()
{

    boost::lock_guard<boost::mutex> lock(criticalSectionSetRunningStateAccess);
    return m_BRunningState;
}

tResult UltrasonicACC::SetRunningState(tBool new_state)
{

    boost::lock_guard<boost::mutex> lock(criticalSectionSetRunningStateAccess);
    m_BRunningState = new_state;

    RETURN_NOERROR;
}

tBool UltrasonicACC::GetRunningStateMoveAgain()
{

    boost::lock_guard<boost::mutex> lock(criticalSectionSetRunningStateMoveAgainAccess);

    return m_BRunningStateMoveAgain;
}

tResult UltrasonicACC::SetRunningStateMoveAgain(tBool new_state)
{

    boost::lock_guard<boost::mutex> lock(criticalSectionSetRunningStateMoveAgainAccess);
    m_BRunningStateMoveAgain = new_state;

    RETURN_NOERROR;
}

tResult UltrasonicACC::SetLIDARInput(TLaserScannerData::Data &LIDARinput)
{

    boost::lock_guard<boost::mutex> lock(criticalSectionSetLidarInputAccess);
    m_dataLidarTmp = LIDARinput;

    RETURN_NOERROR;
}

tResult UltrasonicACC::SetLIDARObstaclesInput(UltrasonicACC::LidarObstacles &LidarObstaclesInput)
{

    boost::lock_guard<boost::mutex> lock(criticalSectionSetLidarObstacelsInputAccess);
    m_dataLidarObstacelesTmp = LidarObstaclesInput;

    RETURN_NOERROR;
}

tResult UltrasonicACC::SetLastLaneDetectionLine(TLaneDetectionLineStruct::Data &lastLane)
{

    boost::lock_guard<boost::mutex> lock(criticalSectionLaneDetectionLine);
    m_dataLastLane = lastLane;

    RETURN_NOERROR;
}

tResult UltrasonicACC::SetLastDistanceForRampChecking(tFloat32 last_distance)
{
    boost::lock_guard<boost::mutex> lock(criticalSectionDistanceForRampChecking);
    m_f32LastDistanceForRampChecking = last_distance;
    RETURN_NOERROR;
}

tFloat32 UltrasonicACC::SetLastDistanceForRampChecking()
{
    boost::lock_guard<boost::mutex> lock(criticalSectionDistanceForRampChecking);
    return m_f32LastDistanceForRampChecking;
}

tResult UltrasonicACC::IncreaseCounterGuardRailsDetected()
{
    boost::lock_guard<boost::mutex> lock(criticalSectionCounterGuardrailsDetected);
    m_ui32CounterGuardRailsDetected++;
    RETURN_NOERROR;
}

tResult UltrasonicACC::ResetCounterGuardRailsDetected()
{
    boost::lock_guard<boost::mutex> lock(criticalSectionCounterGuardrailsDetected);
    m_ui32CounterGuardRailsDetected = 0;
    RETURN_NOERROR;
}
/**
        *@brief weight all the usStruct distances with the weights of how much u trust the value of each sensor if u dont trust it at all set it to 1000
        *
        */


TUltrasonicStruct::Data UltrasonicACC::ComputeWeightedDistanceRear(TUltrasonicStruct::Data USStruct, USweightsRear rearWeights)
{

    if(m_I32DebugLogCounter % 10 == 0 && m_bExtendedDebugModeEnabled) LOG_WARNING(cString::Format("cUltrasonicACC::ComputedWeightedDistanceFront before USStuct.RearLeft %f, USStuct.RearCenter %f, USStuct.RearRight %f",
                                                                                                  USStruct.tRearLeft.f32Value, USStruct.tRearCenter.f32Value, USStruct.tRearRight.f32Value));

    if(m_I32DebugLogCounter % 10 == 0 && m_bExtendedDebugModeEnabled) LOG_WARNING(cString::Format("cUltrasonicACC:: usRearLeft %f, usRearCenter %f, usRearRight %f",
                                                                                                  rearWeights.us_rearLeft, rearWeights.us_rearCenter, rearWeights.us_rearRight));


    if(rearWeights.us_rearLeft != 0.0)
    {
        USStruct.tRearLeft.f32Value /=  rearWeights.us_rearLeft;
    }else
    {
        USStruct.tRearLeft.f32Value = 1000;
    }
    if(rearWeights.us_rearCenter != 0.0)
    {
        USStruct.tRearCenter.f32Value /=  rearWeights.us_rearCenter;
    }else
    {
        USStruct.tRearCenter.f32Value = 1000;
    }
    if(rearWeights.us_rearRight != 0.0)
    {
        USStruct.tRearRight.f32Value /=  rearWeights.us_rearRight;
    }else
    {
        USStruct.tRearRight.f32Value = 1000;
    }
    if(m_I32DebugLogCounter % 10 == 0 && m_bExtendedDebugModeEnabled) LOG_WARNING(cString::Format("cUltrasonicACC::ComputedWeightedDistanceFront after USStuct.RearLeft %f, USStuct.RearCenter %f, USStuct.RearRight %f",
                                                                                                  USStruct.tRearLeft.f32Value, USStruct.tRearCenter.f32Value, USStruct.tRearRight.f32Value));

    return USStruct;
}



TSignalValue::Data UltrasonicACC::ChooseOutputSpeedRear(RedPercentRear redPercent, TSignalValue::Data targetSpeed)
{

    if(- redPercent.RedPercRearLeft > targetSpeed.f32Value) targetSpeed.f32Value = - redPercent.RedPercRearLeft;
    if(- redPercent.RedPercRearCenter > targetSpeed.f32Value) targetSpeed.f32Value = - redPercent.RedPercRearCenter;
    if(- redPercent.RedPercRearRight > targetSpeed.f32Value) targetSpeed.f32Value = - redPercent.RedPercRearRight;

    return targetSpeed;
}

tFloat32 UltrasonicACC::ChooseOutputSpeedFront(tFloat32 interpolatedSpeed, TSignalValue::Data targetSpeed)
{

    if (interpolatedSpeed < targetSpeed.f32Value)
    {
        //LOG_INFO("ChooseOutputSpeedFront: Interpolated Speed was returned: %f", interpolatedSpeed);

        return interpolatedSpeed;
    }
    //LOG_INFO("ChooseOutputSpeedFront: Target Speed  was returned: %f", targetSpeed.f32Value);

    return targetSpeed.f32Value;
}

tResult UltrasonicACC::TransmitFeedbackRamp()
{

    boost::lock_guard<boost::mutex> lock(criticalSection_TransmitFeedback);
    TFeedbackStruct::Data feedbackRamp = m_dataFeedbackRamp;
    //feedbackNoObstacel.ui32ArduinoTimestamp =
    RETURN_IF_FAILED(o_FeedbackStruct.writePin(m_WriterFeedback, (void *) &feedbackRamp, m_pClock->GetStreamTime()));
    if(m_propBDebugActionCommand)LOG_INFO("TransmitFeedbackRamp()");

    RETURN_NOERROR;
}

tResult UltrasonicACC::TransmitFeedbackNoObstacle()
{

    boost::lock_guard<boost::mutex> lock(criticalSection_TransmitFeedback);
    TFeedbackStruct::Data feedbackNoObstacel = m_dataFeedbackNoObstacel;
    RETURN_IF_FAILED(o_FeedbackStruct.writePin(m_WriterFeedback, (void *) &feedbackNoObstacel, m_pClock->GetStreamTime()));
    if(m_propBDebugActionCommand)LOG_INFO("TransmitFeedbackNoObstacle()");

    RETURN_NOERROR;
}


tResult UltrasonicACC::TransmitFeedbackStaticObstacle()
{

    boost::lock_guard<boost::mutex> lock(criticalSection_TransmitFeedback);
    TFeedbackStruct::Data feedbackStaticObstacel = m_dataFeedbackStaticObstacel;
    RETURN_IF_FAILED(o_FeedbackStruct.writePin(m_WriterFeedback, (void *) &feedbackStaticObstacel, m_pClock->GetStreamTime()));
    if(m_propBDebugActionCommand)LOG_INFO("TransmitFeedbackStaticObstacle()");
    RETURN_NOERROR;
}


UltrasonicACC::RedPercentRear UltrasonicACC::GetRedPerscentRear(TUltrasonicStruct::Data USStuct, currentOperationalMode tmp_operationalMode)
{

    RedPercentRear redPercRear;
    redPercRear.RedPercRearLeft = GetReductionPercentage(USStuct.tRearLeft.f32Value,tmp_operationalMode);
    redPercRear.RedPercRearCenter= GetReductionPercentage(USStuct.tRearCenter.f32Value,tmp_operationalMode);
    redPercRear.RedPercRearRight = GetReductionPercentage(USStuct.tRearRight.f32Value,tmp_operationalMode);

    if(m_I32DebugLogCounter % 10 == 0 && m_bExtendedDebugModeEnabled) LOG_WARNING(cString::Format("cUltrasonicACC:: RedPercRearLeft %f,  RedPercRearCenter %f,  RedPercRearRight %f",
                                                                                                  redPercRear.RedPercRearLeft, redPercRear.RedPercRearCenter, redPercRear.RedPercRearRight));

    return redPercRear;
}

tFloat32 UltrasonicACC::GetRedPerscentFront(tFloat32 distance, currentOperationalMode tmp_operationalMode)
{

    tFloat32 interpolatedSpeed = 0.0;

    switch(tmp_operationalMode)
    {
    case DRIVING_MODE_ACTIVE:
        interpolatedSpeed = GetLinearInterpolatedValue(distance, m_vecXValuesDriving, m_vecYValuesDriving);
        break;
    case MtP_MODE_ACTIVE:
        interpolatedSpeed = GetLinearInterpolatedValue(distance, m_vecXValuesMtP,m_vecYValuesMtP);
        break;
    case DRIVING_MODE_CURVE:
        interpolatedSpeed=GetLinearInterpolatedValue(distance, m_vecXValuesBend, m_vecYValuesBend);
        break;
    default:
        LOG_ERROR("UltrasonicACC: (GetRedPerscentFront) invalid state due to undefined operational mode: %d. Only '1', '2' and '3' are currently defined.",tmp_operationalMode);
    }

    return interpolatedSpeed;
}

tFloat32 UltrasonicACC::GetReductionPercentage(tFloat32 distance, currentOperationalMode tmp_operationalMode)
{

    tFloat32 reduction_percentage = 0.0f;
    switch(tmp_operationalMode){
    case DRIVING_MODE_ACTIVE:
        reduction_percentage = GetLinearInterpolatedValue(distance,m_vecXValuesDriving,m_vecYValuesDriving);
        break;
    case MtP_MODE_ACTIVE:
        reduction_percentage = GetLinearInterpolatedValue(distance,m_vecXValuesMtP,m_vecYValuesMtP);
        break;
    case DRIVING_MODE_CURVE:
        reduction_percentage=GetLinearInterpolatedValue(distance, m_vecXValuesBend, m_vecYValuesBend);
    default:
        LOG_ERROR(cString::Format("UltrasonicACC: invalid state due to undefined operational mode: %d. Only '1' and '2' are currently defined.",operationalMode));
        //RETURN_AND_LOG_ERROR_STR(ERR_INVALID_STATE,cString::Format("UltrasonicACC: invalid state due to undefined operational mode: %d. Only '1' and '2' are currently defined.",operationalMode));
    }

    return reduction_percentage;
}

tResult UltrasonicACC::ProcessValidLane(TLaneDetectionLineStruct::Data lastLane)
{
    TLaneDetectionLineStruct::Data tmp_Lane = lastLane;
    //invalid lane
    if((tmp_Lane.i32Point1X == 0) && (tmp_Lane.i32Point1Y == 0) && (tmp_Lane.i32Point2X == 0) && (tmp_Lane.i32Point2Y == 0)&& (tmp_Lane.i32Point3X == 0) && (tmp_Lane.i32Point3Y == 0) && (tmp_Lane.i32Point4X == 0) && (tmp_Lane.i32Point4Y == 0))
    {
        RETURN_IF_FAILED(SetLastElementValidLane(tFalse));
    }
    else
    {
        RETURN_IF_FAILED(SetLastElementValidLane(tTrue));
    }

    RETURN_NOERROR;
}

tFloat32 UltrasonicACC::CalcAngleFromDistance(tFloat32 distance)
{

    return (asinf(((m_propF32CarWidth/2.0)-m_propF32ThresholdReducedVisualRangeOneSide)/tFloat32(distance))*(tFloat32(180.0)/tFloat32(PI)));
}

tResult UltrasonicACC::IncreaseObstacleMovementAgainCounter()
{

    boost::lock_guard<boost::mutex> lock(criticalSectionObstacleMovementAgainCounter);
    m_UI32ObstacleMovementAgainCounter++;

    RETURN_NOERROR;
}
tResult UltrasonicACC::ResetObstacleMovementAgainCounter()
{

    boost::lock_guard<boost::mutex> lock(criticalSectionObstacleMovementAgainCounter);
    m_UI32ObstacleMovementAgainCounter = 0;

    RETURN_NOERROR;
}
tUInt32 UltrasonicACC::GetObstacleMovementAgainCounter()
{

    boost::lock_guard<boost::mutex> lock(criticalSectionObstacleMovementAgainCounter);

    return m_UI32ObstacleMovementAgainCounter;
}
tResult UltrasonicACC::IncreaseObstacleNoMovementCounter()
{

    boost::lock_guard<boost::mutex> lock(criticalSectionObstacleNoMovementAgainCounter);
    m_UI32ObstacleNoMovementCounter++;

    RETURN_NOERROR;
}
tResult UltrasonicACC::ResetObstacleNoMovementCounter()
{

    boost::lock_guard<boost::mutex> lock(criticalSectionObstacleNoMovementAgainCounter);
    m_UI32ObstacleNoMovementCounter = 0;

    RETURN_NOERROR;
}
tUInt32 UltrasonicACC::GetObstacleNoMovementCounter()
{

    boost::lock_guard<boost::mutex> lock(criticalSectionObstacleNoMovementAgainCounter);

    return m_UI32ObstacleNoMovementCounter;
}

tResult UltrasonicACC::TransmitTargetSpeed(TSignalValue::Data mod_targetSpeed)
{
    boost::lock_guard<boost::mutex> lock(criticalSectionTransmitTragetSpeed);
    RETURN_IF_FAILED(o_TSignalValue.writePin(m_WriterTargetSpeed, (void *) &mod_targetSpeed, m_pClock->GetStreamTime()));

    RETURN_NOERROR;
}

tResult UltrasonicACC::SetLastInputTargetSpeed(tFloat32 inputTargetSpeed)
{

    boost::lock_guard<boost::mutex> lock(criticalSectionLastTargetSpeed);
    m_F32LastInputTargetSpeed = inputTargetSpeed;

    RETURN_NOERROR;
}

tFloat32 UltrasonicACC::GetLastInputTargetSpeed()
{

    boost::lock_guard<boost::mutex> lock(criticalSectionLastTargetSpeed);

    return m_F32LastInputTargetSpeed;
}


tResult UltrasonicACC::SetLastModifiedTargetSpeed(tFloat32 modifiedTargetSpeed)
{

    boost::lock_guard<boost::mutex> lock(criticalSectionModifiedTargetSpeed);
    m_F32LastModTargetSpeed = modifiedTargetSpeed;

    RETURN_NOERROR;
}

tFloat32 UltrasonicACC::GetLastModifiedTargetSpeed()
{

    boost::lock_guard<boost::mutex> lock(criticalSectionModifiedTargetSpeed);

    return m_F32LastModTargetSpeed;
}

TActionStruct::Data UltrasonicACC::GetLastActionInput()
{

    boost::lock_guard<boost::mutex> lock(criticalSectionLastInputAction);

    return lastInputAction;
}

tResult UltrasonicACC::SetLastActionInput(TActionStruct::Data lastInput)
{

    boost::lock_guard<boost::mutex> lock(criticalSectionLastInputAction);
    lastInputAction = lastInput;

    RETURN_NOERROR;
}

tResult UltrasonicACC::IncrementGlobalCounter()
{
    boost::lock_guard<boost::mutex> lock(criticalSectionGlobalCounter);
    m_i64GlobalCounter++;
    RETURN_NOERROR;
}

tInt64 UltrasonicACC::GetGlobalCounter()
{
    boost::lock_guard<boost::mutex> lock(criticalSectionGlobalCounter);
    return m_i64GlobalCounter;
}

tResult UltrasonicACC::SetIntersectionMode(UltrasonicACC::currentIntersectionMode currentMode)
{
    boost::lock_guard<boost::mutex> lock(criticalSectionIntersectionMode);
    m_intersectionMode = currentMode;
    RETURN_NOERROR;
}

UltrasonicACC::currentIntersectionMode UltrasonicACC::GetIntersectionMode()
{
    boost::lock_guard<boost::mutex> lock(criticalSectionIntersectionMode);
    return m_intersectionMode;
}


/*
tResult UltrasonicACC::ProcessCheckDoll()
{
    UltrasonicACC::LidarObstacles tmp_obstacle= GetLIDARObstacles();
    vector <LidarPoint> lidarPointPossiblePointDoll;
    for(tUInt32 i = 0; i < tmp_obstacle.ui32Size; i++)
    {

       if(tmp_obstacle.tScanArrayEval[i].ui32ObstacleCounter != 0)
       {
           LidarPoint scanArray;
           scanArray.ui32Counter = i;
           scanArray.f32Radius = tmp_obstacle.tScanArrayEval[i].f32Radius;
           scanArray.f32Angle = tmp_obstacle.tScanArrayEval[i].f32Angle;
           scanArray.f32y = tmp_obstacle.tScanArrayEval[i].f32Radius * sinf((90-tmp_obstacle.tScanArrayEval[i].f32Angle)*tFloat32(PI)/180.0f);
           scanArray.f32x = tmp_obstacle.tScanArrayEval[i].f32Radius * cosf((90-tmp_obstacle.tScanArrayEval[i].f32Angle)*tFloat32(PI)/180.0f);
           if((scanArray.f32x < m_propF32ROIDollXmax) && (scanArray.f32x > m_propF32ROIDollXmin) && (scanArray.f32y < m_propF32ROIDollYmax) && (scanArray.f32y > m_propF32ROIDollYmin))
           {
               lidarPointPossiblePointDoll.push_back(scanArray);
           }
       }
    }
    if(lidarPointPossiblePointDoll.size() < 2)
    {
        if(m_propBDebugDoll)LOG_INFO("Not enough points for the doll");
        RETURN_NOERROR;
    }
    vector <LidarPoint> coorDollFinal;
    tUInt32 ui32LastPoint = 999;
    for (tUInt32 i = 0; i < lidarPointPossiblePointDoll.size(); i++)
    {
        tFloat32 distance = sqrt((pow((lidarPointPossiblePointDoll[i].f32x-lidarPointPossiblePointDoll[0].f32x), 2))+pow((lidarPointPossiblePointDoll[i].f32y-lidarPointPossiblePointDoll[0].f32y), 2));
        if(distance < m_propF32DollWidth)
        {
            coorDollFinal.push_back(lidarPointPossiblePointDoll[i]);
            ui32LastPoint = i;
        }
        else
        {
            if(coorDollFinal.size() < 1)
            {
                RETURN_NOERROR;
            }
            tFloat32 distanceToLast = sqrt((pow((lidarPointPossiblePointDoll[i].f32x-lidarPointPossiblePointDoll[ui32LastPoint].f32x), 2))+pow((lidarPointPossiblePointDoll[i].f32y-lidarPointPossiblePointDoll[ui32LastPoint].f32y), 2));
            //aktuell hier
            if(distanceToLast > m_propF32ThresholdDollAnotherObject)
            {
                tFloat32 distanceBeginToEnd = sqrt((pow((lidarPointPossiblePointDoll[0].f32x-lidarPointPossiblePointDoll[ui32LastPoint].f32x), 2))+pow((lidarPointPossiblePointDoll[0].f32y-lidarPointPossiblePointDoll[ui32LastPoint].f32y), 2));
                if(distanceBeginToEnd > m_propF32DollWidth)
                {
                    RETURN_NOERROR;
                }
                else
                {
                    RETURN_IF_FAILED(TransmitFeedbackDoll());
                }
            }
        }
    }



}
*/
tResult UltrasonicACC::ProcessActionIntersectionCheckAll()
{

    //2017: roiXYIntersectionOncoming, roiXYIntersectionCrosstraffic //2018: + roiXYIntersectionCrosstrafficLeft
    if(m_propBDebugActionCommand)LOG_INFO("AC_UA_INTERSECTION_CHECK_ALL");
    ProcessROI(roiXYIntersectionOncoming, roiXYIntersectionCrosstraffic, roiXYIntersectionCrosstrafficLeft);
    /*occupancy occupancy_status = INITIALILZED;
            occupancy_status = CheckROIforObstacles(roiXYIntersectionOncoming, roiXYIntersectionCrosstraffic, roiXYIntersectionCrosstrafficLeft);
            for (uint i =0; i < m_propUI32EndOfForLoopToCheckROI; i++)
            {
                ///
                ///!!!!!
                ///
                occupancy_status = CheckROIforObstacles(roiXYOvertakingObstacleStraight);//not right!!!!!!
                if((occupancy_status == FREE_SPACE)||(occupancy_status == ERROR
                                                      || occupancy_status == INITIALILZED) || (occupancy_status == OCCUPIED_STATIC))
                {
                    break;
                }
            }
            if (occupancy_status == FREE_SPACE) {
                RETURN_IF_FAILED(TransmitFeedbackNoObstacle());
                SetRunningState(tFalse);
                if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 1) {
                    LOG_WARNING(cString::Format("ObstacleDetec: No obstacle detected at all (in intersection mode), free space!"));
                    m_ui32DebugLogEventToggle = 1;
                }
            } else if (occupancy_status == ERROR
                       || occupancy_status == INITIALILZED) {
                LOG_ERROR(cString::Format("ObstacleDetec: Error occurred during region-check 'INTERSECTION - CHECK_ALL'. Return-status: %d",occupancy_status));
            } else if (occupancy_status == OCCUPIED_SPACE) {
                if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 2) {
                    LOG_WARNING(cString::Format("ObstacleDetec: Oncoming or crossing obstacle detected (in intersection mode)!"));
                    m_ui32DebugLogEventToggle = 2;
                }
            } else if (occupancy_status == OCCUPIED_STATIC) {
                RETURN_IF_FAILED(TransmitFeedbackStaticObstacle());
                SetRunningState(tFalse);
                if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 3) {
                    LOG_WARNING(
                                cString::Format(
                                    "ObstacleDetec: Static obstacle detected! (in intersection check-all mode)"));
                    m_ui32DebugLogEventToggle = 3;
                }*/

    //}

    RETURN_NOERROR;
}

tResult UltrasonicACC::ProcessActionIntersectionOncomingTraffic()
{

    if(m_propBDebugActionCommand)LOG_INFO("AC_UA_INTERSECTION_CHECK_ONCOMING_TRAFFIC");
    //if(m_propBDebugActionCommand){(o_FeedbackStruct.writePin(m_WriterFeedback, (void *) &m_dataFeedbackMovingAgain, m_pClock->GetStreamTime()));}
    ProcessROI(roiXYIntersectionOncoming);
    /*occupancy occupancy_status = INITIALILZED;
            //roiXYIntersectionOncoming
            occupancy_status = CheckROIforObstacles(roiXYIntersectionOncoming);
            for (uint i =0; i < m_propUI32EndOfForLoopToCheckROI; i++)
            {
                ///
                ///!!!!!
                ///
                occupancy_status = CheckROIforObstacles(roiXYOvertakingObstacleStraight);//not right!!!!!!
                if((occupancy_status == FREE_SPACE)||(occupancy_status == ERROR
                                                      || occupancy_status == INITIALILZED) || (occupancy_status == OCCUPIED_STATIC))
                {
                    break;
                }
            }
            if (occupancy_status == FREE_SPACE) {
                RETURN_IF_FAILED(TransmitFeedbackNoObstacle());
                SetRunningState(tFalse);
                if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 1) {
                    LOG_WARNING(
                                cString::Format(
                                    "ObstacleDetec: No oncoming obstacle detected (in intersection mode), free space!"));
                    m_ui32DebugLogEventToggle = 1;
                }
            } else if (occupancy_status == ERROR
                       || occupancy_status == INITIALILZED) {
                LOG_ERROR(
                            cString::Format(
                                "ObstacleDetec: Error occurred during region-check 'INTERSECTION - ONCOMING'. Return-status: %d",
                                occupancy_status));
            } else if (occupancy_status == OCCUPIED_SPACE) {
                if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 2) {
                    LOG_WARNING(
                                cString::Format(
                                    "ObstacleDetec: Oncoming obstacle detected!"));
                    m_ui32DebugLogEventToggle = 2;
                }
            } else if (occupancy_status == OCCUPIED_STATIC) {
                RETURN_IF_FAILED(TransmitFeedbackStaticObstacle());
                SetRunningState(tFalse);
                if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 3) {
                    LOG_WARNING(
                                cString::Format(
                                    "ObstacleDetec: Static obstacle detected! (in cross-parking mode, cross pullout)"));
                    m_ui32DebugLogEventToggle = 3;
                }
            }*/

    RETURN_NOERROR;
}

tResult UltrasonicACC::ProcessActionIntersectionCrosstrafficRight()
{

    if(m_propBDebugActionCommand)LOG_INFO("AC_UA_INTERSECTION_CHECK_CROSS_TRAFFIC_RIGHT");
    /* call method to check traffic from right-hand-side only */
    ProcessROI(roiXYIntersectionCrosstraffic);
    /*occupancy occupancy_status = INITIALILZED;
            occupancy_status = CheckROIforObstacles(roiXYIntersectionCrosstraffic);
            for (uint i =0; i < m_propUI32EndOfForLoopToCheckROI; i++)
            {
                ///
                ///!!!!!
                ///
                occupancy_status = CheckROIforObstacles(roiXYOvertakingObstacleStraight);//not right!!!!!!
                if((occupancy_status == FREE_SPACE)||(occupancy_status == ERROR
                                                      || occupancy_status == INITIALILZED) || (occupancy_status == OCCUPIED_STATIC))
                {
                    break;
                }
            }
            if (occupancy_status == FREE_SPACE) {
                RETURN_IF_FAILED(TransmitFeedbackNoObstacle());
                SetRunningState(tFalse);
                if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 1) {
                    LOG_WARNING(
                                cString::Format(
                                    "ObstacleDetec: No crossing obstacle detected (in intersection mode), free space!"));
                    m_ui32DebugLogEventToggle = 1;
                }
            } else if (occupancy_status == ERROR
                       || occupancy_status == INITIALILZED) {
                LOG_ERROR(
                            cString::Format(
                                "ObstacleDetec: Error occurred during region-check 'INTERSECTION - CROSSING RIGHT'. Return-status: %d",
                                occupancy_status));
            } else if (occupancy_status == OCCUPIED_SPACE) {
                if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 2) {
                    LOG_WARNING(
                                cString::Format(
                                    "ObstacleDetec: Crossing obstacle detected!"));
                    m_ui32DebugLogEventToggle = 2;
                }
            } else if (occupancy_status == OCCUPIED_STATIC) {
                RETURN_IF_FAILED(TransmitFeedbackStaticObstacle());
                SetRunningState(tFalse);
                if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 3) {
                    LOG_WARNING(
                                cString::Format(
                                    "ObstacleDetec: Static obstacle detected! (in cross-parking mode, cross pullout)"));
                    m_ui32DebugLogEventToggle = 3;
                }

            }*/

    RETURN_NOERROR;
}


tResult UltrasonicACC::ProcessIntersectionCheckOncomingAndRight()
{

    if(m_propBDebugActionCommand)LOG_INFO("AC_UA_INTERSECTION_CHECK_ONCOMING_AND_RIGHT");
    /* call method to check traffic from right-hand-side only */
    ProcessROI(roiXYIntersectionCrosstraffic, roiXYIntersectionOncoming);
    /*occupancy occupancy_status = INITIALILZED;
        //  occupancy_status = CheckROIforObstacles(roiXYIntersectionCrosstraffic, roiXYIntersectionOncoming);
            for (uint i =0; i < m_propUI32EndOfForLoopToCheckROI; i++)
            {
                ///
                ///!!!!!
                ///
                occupancy_status = CheckROIforObstacles(roiXYIntersectionCrosstraffic, roiXYIntersectionOncoming);//not right!!!!!!
                if((occupancy_status == FREE_SPACE)||(occupancy_status == ERROR
                                                      || occupancy_status == INITIALILZED) || (occupancy_status == OCCUPIED_STATIC))
                {
                    break;
                }
            }
            if (occupancy_status == FREE_SPACE) {
                RETURN_IF_FAILED(TransmitFeedbackNoObstacle());
                SetRunningState(tFalse);
                if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 1) {
                    LOG_WARNING(
                                cString::Format(
                                    "ObstacleDetec: No crossing obstacle detected (in intersection mode), free space!"));
                    m_ui32DebugLogEventToggle = 1;
                }
            } else if (occupancy_status == ERROR
                       || occupancy_status == INITIALILZED) {
                LOG_ERROR(
                            cString::Format(
                                "ObstacleDetec: Error occurred during region-check 'INTERSECTION - CROSSING RIGHT'. Return-status: %d",
                                occupancy_status));
            } else if (occupancy_status == OCCUPIED_SPACE) {
                if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 2) {
                    LOG_WARNING(
                                cString::Format(
                                    "ObstacleDetec: Crossing obstacle detected!"));
                    m_ui32DebugLogEventToggle = 2;
                }
            } else if (occupancy_status == OCCUPIED_STATIC) {
                RETURN_IF_FAILED(TransmitFeedbackStaticObstacle());
                SetRunningState(tFalse);
                if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 3) {
                    LOG_WARNING(
                                cString::Format(
                                    "ObstacleDetec: Static obstacle detected! (in cross-parking mode, cross pullout)"));
                    m_ui32DebugLogEventToggle = 3;
                }

            }*/

    RETURN_NOERROR;
}

tResult UltrasonicACC::ProcessActionIntersectionCheckLeft()
{

    if(m_propBDebugActionCommand)LOG_INFO("AC_UA_INTERSECTION_CHECK_LEFT");
    /* call method to check traffic from right-hand-side only */
    ProcessROI(roiXYIntersectionCrosstrafficLeft);
    /*occupancy occupancy_status = INITIALILZED;
            occupancy_status = CheckROIforObstacles(roiXYIntersectionCrosstrafficLeft);
            for (uint i =0; i < m_propUI32EndOfForLoopToCheckROI; i++)
            {
                ///
                ///!!!!!
                ///
                occupancy_status = CheckROIforObstacles(roiXYOvertakingObstacleStraight);//not right!!!!!!
                if((occupancy_status == FREE_SPACE)||(occupancy_status == ERROR
                                                      || occupancy_status == INITIALILZED) || (occupancy_status == OCCUPIED_STATIC))
                {
                    break;
                }
            }
            if (occupancy_status == FREE_SPACE) {
                RETURN_IF_FAILED(TransmitFeedbackNoObstacle());
                SetRunningState(tFalse);
                if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 1) {
                    LOG_WARNING(
                                cString::Format(
                                    "ObstacleDetec: No crossing obstacle detected (in intersection mode), free space!"));
                    m_ui32DebugLogEventToggle = 1;
                }
            } else if (occupancy_status == ERROR
                       || occupancy_status == INITIALILZED) {
                LOG_ERROR(
                            cString::Format(
                                "ObstacleDetec: Error occurred during region-check 'INTERSECTION - CROSSING RIGHT'. Return-status: %d",
                                occupancy_status));
            } else if (occupancy_status == OCCUPIED_SPACE) {
                if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 2) {
                    LOG_WARNING(
                                cString::Format(
                                    "ObstacleDetec: Crossing obstacle detected!"));
                    m_ui32DebugLogEventToggle = 2;
                }
            } else if (occupancy_status == OCCUPIED_STATIC) {
                RETURN_IF_FAILED(TransmitFeedbackStaticObstacle());
                SetRunningState(tFalse);
                if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 3) {
                    LOG_WARNING(
                                cString::Format(
                                    "ObstacleDetec: Static obstacle detected! (in cross-parking mode, cross pullout)"));
                    m_ui32DebugLogEventToggle = 3;
                }

            }*/

    RETURN_NOERROR;
}


tResult UltrasonicACC::ProcessActionIntersectionNothingToCheck()
{

    if(m_propBDebugActionCommand)LOG_INFO("AC_UA_INTERSECTION_NOTHING_TO_CHECK");
    RETURN_IF_FAILED(TransmitFeedbackNoObstacle());

    RETURN_NOERROR;
}

tResult UltrasonicACC::ProcessActionCheckRightAndLeft()
{

    if(m_propBDebugActionCommand)LOG_INFO("AC_UA_INTERSECTION_CHECK_RIGHT_AND_LEFT");
    /* call method to check traffic from right-hand-side only */
    RETURN_IF_FAILED(ProcessROI(roiXYIntersectionCrosstrafficLeft, roiXYIntersectionCrosstraffic));
    /*occupancy occupancy_status = INITIALILZED;
            occupancy_status = CheckROIforObstacles(roiXYIntersectionCrosstrafficLeft, roiXYIntersectionCrosstraffic);
            for (uint i =0; i < m_propUI32EndOfForLoopToCheckROI; i++)
            {
                ///
                ///!!!!!
                ///
                occupancy_status = CheckROIforObstacles(roiXYOvertakingObstacleStraight);//not right!!!!!!
                if((occupancy_status == FREE_SPACE)||(occupancy_status == ERROR
                                                      || occupancy_status == INITIALILZED) || (occupancy_status == OCCUPIED_STATIC))
                {
                    break;
                }
            }
            if (occupancy_status == FREE_SPACE) {
                RETURN_IF_FAILED(TransmitFeedbackNoObstacle());
                SetRunningState(tFalse);
                if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 1) {
                    LOG_WARNING(
                                cString::Format(
                                    "ObstacleDetec: No crossing obstacle detected (in intersection mode), free space!"));
                    m_ui32DebugLogEventToggle = 1;
                }
            } else if (occupancy_status == ERROR
                       || occupancy_status == INITIALILZED) {
                LOG_ERROR(
                            cString::Format(
                                "ObstacleDetec: Error occurred during region-check 'INTERSECTION - CROSSING RIGHT'. Return-status: %d",
                                occupancy_status));
            } else if (occupancy_status == OCCUPIED_SPACE) {
                if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 2) {
                    LOG_WARNING(
                                cString::Format(
                                    "ObstacleDetec: Crossing obstacle detected!"));
                    m_ui32DebugLogEventToggle = 2;
                }
            } else if (occupancy_status == OCCUPIED_STATIC) {
                RETURN_IF_FAILED(TransmitFeedbackStaticObstacle());
                SetRunningState(tFalse);
                if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 3) {
                    LOG_WARNING(
                                cString::Format(
                                    "ObstacleDetec: Static obstacle detected! (in cross-parking mode, cross pullout)"));
                    m_ui32DebugLogEventToggle = 3;
                }

            }*/

    RETURN_NOERROR;
}




tResult UltrasonicACC::TransmitFeedbackIntersectionROIreceived()
{

    boost::lock_guard<boost::mutex> lock(criticalSection_TransmitFeedback);
    TFeedbackStruct::Data feedbackIntersectionROIreceived = m_dataFeedbackNoObstacel;
    feedbackIntersectionROIreceived.ui32FeedbackStatus = FB_UA_RECEIVED_INTERSECTION_COMMAND;
    feedbackIntersectionROIreceived.ui8FilterId = F_ULTRASONIC_ACC;
    RETURN_IF_FAILED(o_FeedbackStruct.writePin(m_WriterFeedback, (void *) &feedbackIntersectionROIreceived, m_pClock->GetStreamTime()));
    if(m_propBDebugActionCommand)LOG_INFO("TransmitFeedbackIntersectionROIreceived()");
    RETURN_NOERROR;
}

tResult UltrasonicACC::TransmitFeedBackDrivingModeActive()
{

    boost::lock_guard<boost::mutex> lock(criticalSection_TransmitFeedback);
    TFeedbackStruct::Data feedbackDrivingModeActive;
    feedbackDrivingModeActive.ui32FeedbackStatus = FB_UA_DRIVING_MODE;
    feedbackDrivingModeActive.ui8FilterId = F_ULTRASONIC_ACC;
    RETURN_IF_FAILED(o_FeedbackStruct.writePin(m_WriterFeedback, (void *) &feedbackDrivingModeActive, m_pClock->GetStreamTime()));
    if(m_propBDebugActionCommand)LOG_INFO("TransmitFeedBackDrivingModeActive()");
    RETURN_NOERROR;
}

tResult UltrasonicACC::TransmitFeedBackMTPMode()
{

    boost::lock_guard<boost::mutex> lock(criticalSection_TransmitFeedback);
    TFeedbackStruct::Data feedbackMTPMode;
    feedbackMTPMode.ui32FeedbackStatus = FB_UA_MTP_MODE;
    feedbackMTPMode.ui8FilterId = F_ULTRASONIC_ACC;
    RETURN_IF_FAILED(o_FeedbackStruct.writePin(m_WriterFeedback, (void *) &feedbackMTPMode, m_pClock->GetStreamTime()));
    if(m_propBDebugActionCommand)LOG_INFO("TransmitFeedBackMTPMode()");
    RETURN_NOERROR;
}

tResult UltrasonicACC::TransmitFeedBackOffRamp()
{

    boost::lock_guard<boost::mutex> lock(criticalSection_TransmitFeedback);
    TFeedbackStruct::Data feedbackOffRamp;
    feedbackOffRamp.ui32FeedbackStatus = FB_UA_OFF_RAMP;
    feedbackOffRamp.ui8FilterId = F_ULTRASONIC_ACC;
    RETURN_IF_FAILED(o_FeedbackStruct.writePin(m_WriterFeedback, (void *) &feedbackOffRamp, m_pClock->GetStreamTime()));
    if(m_propBDebugActionCommand)LOG_INFO("TransmitfeedbackOffRamp()");
    RETURN_NOERROR;
}
/*
tResult UltrasonicACC::TransmitFeedbackDoll()
{

    boost::lock_guard<boost::mutex> lock(criticalSection_TransmitFeedback);
    TFeedbackStruct::Data feedbackDoll;
    feedbackDoll.ui32FeedbackStatus = FB_UA_DOLL;
    feedbackDoll.ui8FilterId = F_ULTRASONIC_ACC;
    RETURN_IF_FAILED(o_FeedbackStruct.writePin(m_WriterFeedback, (void *) &feedbackDoll, m_pClock->GetStreamTime()));
    if(m_propBDebugActionCommand)LOG_INFO("TransmitFeedDoll()");
    RETURN_NOERROR;
}*/
/*
        //unfortunaly, there was an error due to adtf3 and I had no time to fix it. I copied the values from the xml file to a vector -> I did not have to use the function
        tResult UltrasonicACC::LoadConfigurationData(cFilename& m_fileConfig, vector<tFloat32>& m_xValues, vector<tFloat32>& m_yValues,currentOperationalMode OpMode)
        {


            using namespace tinyxml2;
            // check if file exits
            if (m_fileConfig.IsEmpty()) {
                LOG_ERROR("UltrasonicACC: Configuration file not found");
                RETURN_ERROR(ERR_INVALID_FILE);
            }

            // create absolute path
            //    ADTF_GET_CONFIG_FILENAME(m_fileConfig);
            //    m_fileConfig = m_fileConfig.CreateAbsolutePath(".");
            cFilename fileConfig = m_fileConfig;
            adtf::services::ant::adtf_resolve_macros(fileConfig);
            //Load file, parse configuration, print the data

            if (cFileSystem::Exists(m_fileConfig))
            {
                cDOM oDOM;//xml parser
                oDOM.Load(m_fileConfig);
                // changelog: only linear interpolation is allowed, so cubic interpolation is removed
                //load supporting points
                cDOMElementRefList oElems;
                if (IS_OK(oDOM.FindNodes("calibration/supportingPoints/point", oElems))) {
                    for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem) {

                        cDOMElement* pConfigElement;
                        if (IS_OK((*itElem)->FindNode("xValue", pConfigElement))) {
                            m_xValues.push_back(tFloat32(cString(pConfigElement->GetData()).AsFloat64()));
                        }
                        if (IS_OK((*itElem)->FindNode("yValue", pConfigElement))) {
                            m_yValues.push_back(tFloat32(cString(pConfigElement->GetData()).AsFloat64()));
                        }
                    }
                }
                if (oElems.size() > 0)
                {
                    if (m_bPrintInitialtable)
                    {
                        if(OpMode==DRIVING_MODE_ACTIVE) LOG_WARNING(cString::Format("UltrasonicACC: printing xml data for driving mode file: "));
                        if(OpMode==MtP_MODE_ACTIVE) LOG_WARNING(cString::Format("UltrasonicACC: printing xml data for MtP mode file: "));
                        for (tUInt i = 0; i < m_xValues.size(); i++){
                            if (i > m_yValues.size())
                                break;
                            LOG_WARNING(cString::Format("UltrasonicACC: supportingPoint #%d: (%lf/%lf)", i, m_xValues[i], m_yValues[i]));
                        }
                    }
                } else
                {
                    RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,"UltrasonicACC: no supporting points in given file found!");
                }
                //checks if data are valid
                RETURN_IF_FAILED(CheckConfigurationData(m_fileConfig,m_xValues));

            }
            {
                RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,"UltrasonicACC: Configured configuration file not found");
            }

            RETURN_NOERROR;
        }

        */
tResult UltrasonicACC::CheckConfigurationData(cFilename m_fileConfig, vector<tFloat32> m_xValues)
{

    //checks if the xValues of the calibration table are increasing
    for (vector<tFloat32>::iterator it = m_xValues.begin(); it != m_xValues.end(); it++)
    {
        vector<tFloat32>::iterator it2 = it;
        it2++;
        if (it2 != m_xValues.end()) {
            // next values is smaller than current value
            if ((tFloat32(*it) > tFloat32(*it2))) {
                RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,cString::Format("UltrasonicACC: The xValues in the file %s are not in increasing order. Please reorder the points!", m_fileConfig.GetPtr()));
            }
        }
    }

    RETURN_NOERROR;
}

//for calc the line of the bend
tFloat32 UltrasonicACC::GetLinearInterpolatedValueBend(tFloat32 f32InputValue, tFloat32 x1, tFloat32 y1, tFloat32 x2, tFloat32 y2)
{//valid if the values are correct

    //input value is y value
    //y = slope*x + yintercept
    //values are correct
    if((fabs(x2-x1))<m_propF32ThresholdSlopeTooHigh)//slope would be very high
    {
        return x2;
    }
    tFloat32 slope = (y2-y1)/(x2-x1);
    tFloat32 yintercept = y2 - slope * x2;

    //I should check if the return value is logical
    return ((f32InputValue - yintercept)/slope);//x = (y-yintercept)/slope
}

tFloat32 UltrasonicACC::GetLinearInterpolatedValue(tFloat32 fl32InputValue, vector<tFloat32> m_xValues, vector<tFloat32> m_yValues)
{

    // requested value is smaller than smallest value in table
    if(m_xValues.empty())
    {
        LOG_ERROR("xVec is empty");
    }
    if(m_yValues.empty())
    {
        LOG_ERROR("yVec is empty");
    }
    if(fl32InputValue < m_xValues.front())
    {
        if(m_bBorderWarningModeEnabled)
        {
            LOG_WARNING(cString::Format("UltrasonicACC: requested x-value %f is lower than smallest x-value in calibration table", fl32InputValue));
        }
        return m_yValues.front();
    }
    // requested value is bigger than biggest value in table
    else if(fl32InputValue > m_xValues.back())
    {
        if(m_bBorderWarningModeEnabled)
        {
            LOG_WARNING(cString::Format("UltrasonicACC: requested x-value %f is higher than highes x-value in calibration table", fl32InputValue));
        }
        return m_yValues.back();
    }
    // search in vector for corresponding index (smallest neighbor)
    tUInt iIndex;
    if(m_xValues.size() > 2)
    {
        for(iIndex = 0; iIndex < m_xValues.size(); iIndex++)
        {
            if(m_xValues[iIndex] >= fl32InputValue)
            {
                break;
            }
        }
        // get smaller neighbor
        if(iIndex != 0)
        {
            iIndex = iIndex - 1;
        }
    }
    else
    {
        iIndex = 0;
    }
    if(iIndex == (m_xValues.size()-1))
    {
        LOG_WARNING("Potentieller seg fault!");
    }
    if((m_xValues[iIndex + 1] - m_xValues[iIndex]) != 0)
    {
        // doing the linear interpolation
        tFloat32 f32Value = (fl32InputValue - m_xValues[iIndex]) * (m_yValues[iIndex + 1] - m_yValues[iIndex])
                / (m_xValues[iIndex + 1] - m_xValues[iIndex]) + m_yValues[iIndex];

        //tFloat32 security check to send only minimum or maximum value of table
        if(f32Value > *max_element(m_yValues.begin(), m_yValues.end()))
        {
            return *max_element(m_yValues.begin(), m_yValues.end());
        }
        else if(f32Value < *min_element(m_yValues.begin(), m_yValues.end()))
        {
            return *min_element(m_yValues.begin(), m_yValues.end());
        }
        else
        {
            return f32Value;
        }
    }
    else
    {
        LOG_ERROR("UltrasonicACC: invalid table in xml, multiple x_data_points with same value! Please check.");
        return 0;
    }
}

tBool UltrasonicACC::GetValidLane()
{
    boost::lock_guard<boost::mutex> lock(criticalSectionValidLane);
    if(vecBvalidLane.empty())
    {
        LOG_ERROR("USACC: vecBvalidLane is empty");
        return tFalse;
    }
    for (tUInt32 i = 0; i < vecBvalidLane.size(); i++)
    {
        if(!vecBvalidLane[i])
        {
            return tFalse;
        }
    }
    return tTrue;
}

tResult UltrasonicACC::SetLastElementValidLane(tBool validLane)
{
    boost::lock_guard<boost::mutex> lock(criticalSectionValidLane);
    //vecBvalidLane
    if(vecBvalidLane.empty())
    {
        LOG_ERROR("USACC: vecBvalidLane is empty");
        vecBvalidLane.push_back(validLane);
    }
    else
    {
        for (tUInt32 i = vecBvalidLane.size(); i > 0; i--)
        {
            if(i == 1)
            {
                //LOG_INFO("In i ==1 !");
                vecBvalidLane[0] = validLane;
            }
            else
            {
                if((i-2) < 0)LOG_INFO("i-2 ist kleiner null!!!! i: %d", i);
                vecBvalidLane[i-1] = vecBvalidLane[i-2];
            }

            /*
            LOG_INFO("i: %d", i);
            if(i == 0)
            {
                LOG_INFO("In i == 0!");
                vecBvalidLane[0] = validLane;
            }
            else
            {
                vecBvalidLane[i] = vecBvalidLane[i-1];
            }*/
        }

    }
    RETURN_NOERROR;


}

UltrasonicACC::LidarObstacles UltrasonicACC::ObstacleDetectionWithLidar(TLaserScannerData::Data lasersample) //calls angleCompensation
{

    /* returns LAserScannerData with the angle and a related counter(every Object has on) at the moment the radius is
             * not returned -> we have to compare it with the input after the function */
    UltrasonicACC::LidarObstacles lidarObstaclesResult;

    tFloat32 f32counter = 0.0f;
    for (uint32_t i=0; (i<lasersample.ui32Size); i++)
    {
        lasersample.tScanArray[i].f32Angle = AngleCompensation(lasersample.tScanArray[i].f32Angle);
    }
    //sort the input data
    qsort((void *) lasersample.tScanArray, lasersample.ui32Size, sizeof(TPolarCoordiante::Data), compareAngle);//check if it works
    //check if it works
    TLaserScannerData::Data objectCounterArray = lasersample; //the radius contains the information about the counter
    //set the all elements of the radius to 0.0
    for (uint32_t i=0; (i<lasersample.ui32Size); i++)
    {
        objectCounterArray.tScanArray[i].f32Radius = 0.0f;

    }
    for (uint32_t i=0; (i<lasersample.ui32Size); i++)
    {
        if(i == 0)//intercept first value
        {
            if ((lasersample.tScanArray[i].f32Radius < m_propF32ObjectThresholdDetection) && (lasersample.tScanArray[i].f32Radius != 0.0))
            {
                if(objectCounterArray.tScanArray[i].f32Radius != 0.0f)
                {
                    LOG_WARNING("There was already a value in the counter array. Nevertheless, continue the calculation! Angle: %f", objectCounterArray.tScanArray[i].f32Angle);
                }
                f32counter++; //it has to be a new object because it the first value
                objectCounterArray.tScanArray[i].f32Radius = f32counter;
            }
        }

        else if ((lasersample.tScanArray[i].f32Radius != 0.0f) && (lasersample.tScanArray[i].f32Radius < m_propF32ObjectThresholdDetection)) //zero is an invalid value
        {
            if(lasersample.tScanArray[i-1].f32Radius == 0.0f)//last value was 0
            {
                if(objectCounterArray.tScanArray[i].f32Radius != 0.0f)
                {
                    LOG_WARNING("There was already a value in the counter array. Nevertheless, continue the calculation! Angle: %f", objectCounterArray.tScanArray[i].f32Angle);
                }
                f32counter++;
                objectCounterArray.tScanArray[i].f32Radius = f32counter;
            }
            else if(lasersample.tScanArray[i-1].f32Radius >= m_propF32ObjectThresholdDetection)//last sample was higher than the treshold
            {
                if(objectCounterArray.tScanArray[i].f32Radius != 0.0f)
                {
                    LOG_WARNING("There was already a value in the counter array. Nevertheless, continue the calculation! Angle: %f", objectCounterArray.tScanArray[i].f32Angle);
                }
                f32counter++;
                objectCounterArray.tScanArray[i].f32Radius = f32counter;
            }
            else if(lasersample.tScanArray[i-1].f32Radius < m_propF32ObjectThresholdDetection)//the last sample was within the threhold
            {
                if(m_propF32ObjectThresholdRadius <= fabsf(lasersample.tScanArray[i-1].f32Radius-lasersample.tScanArray[i].f32Radius))//considered as a different object
                {
                    if(objectCounterArray.tScanArray[i].f32Radius != 0.0f)
                    {
                        LOG_WARNING("There was already a value in the counter array. Nevertheless, continue the calculation! Angle: %f", objectCounterArray.tScanArray[i].f32Angle);
                    }
                    f32counter++;
                    objectCounterArray.tScanArray[i].f32Radius = f32counter;
                }
                else if (m_propF32ObjectThresholdRadius > fabsf(lasersample.tScanArray[i-1].f32Radius-lasersample.tScanArray[i].f32Radius))//considered as the same object
                {
                    if(objectCounterArray.tScanArray[i].f32Radius != 0.0f)
                    {
                        LOG_WARNING("There was already a value in the counter array. Nevertheless, continue the calculation! Angle: %f", objectCounterArray.tScanArray[i].f32Angle);
                    }
                    objectCounterArray.tScanArray[i].f32Radius = f32counter;
                }
            }
        }
    }
    lidarObstaclesResult.ui32Size = lasersample.ui32Size;
    for (uint32_t i=0; (i<lidarObstaclesResult.ui32Size); i++)
    {
        lidarObstaclesResult.tScanArrayEval[i].f32Angle = objectCounterArray.tScanArray[i].f32Angle;
        lidarObstaclesResult.tScanArrayEval[i].f32Radius = lasersample.tScanArray[i].f32Radius;
        lidarObstaclesResult.tScanArrayEval[i].ui32ObstacleCounter = static_cast<tUInt32>(objectCounterArray.tScanArray[i].f32Radius); //cast
    }

    return lidarObstaclesResult;//return objectCounterArray;
}

tResult UltrasonicACC::TransmitRelevantLaser(TLaserScannerData::Data laserToTransmit)
{
    boost::lock_guard<boost::mutex> lock(criticalSectionTransmitRelevantLaser);
    RETURN_IF_FAILED(o_LaserScanner.writePin(m_WriterRelevantLaser, (void *) &laserToTransmit, m_pClock->GetStreamTime()));
    RETURN_NOERROR;
}

//to compensate the inconsistante angle range-> output car right 90° to car left -90°
tFloat32 UltrasonicACC::AngleCompensation(tFloat32 angle)
{

    if (angle >= 0.0 && angle <= 90)
    {
        return angle;
    }
    else if(angle >= 270 && angle <= 360)
    {
        return (angle -360);
    }
    else
    {
        LOG_ERROR("Angle out of range!");
        return 100000.0f;//Error value but without error handeling!
    }
}

tFloat32 UltrasonicACC::CalcDistanceWithCos(tFloat32 a, tFloat32 b, tFloat32 gammaInDeg)
{
    //c²=a²+b²-2abcosgamma
    return (sqrt(pow(a,2)+pow(b,2)-2*a*b*cosf(gammaInDeg * tFloat32(PI) /180.0f)));
}

tFloat32 UltrasonicACC::AngleBackCompensation(tFloat32 angle)
{

    if (angle >= 0.0 && angle <= 90)
    {
        return angle;
    }
    else if(angle >= -90 && angle <= 0)
    {
        return (angle +360);
    }
    else
    {
        LOG_ERROR("Angle out of range!");
        return 100000.0f;//Error value but without error handeling!
    }
}


//checks if an obstacle is in the box and returns the radius of the closest obstacle in the box
tFloat32 UltrasonicACC::IsObjectInBox(UltrasonicACC::roiXY roi_XY, UltrasonicACC::LidarObstacles LidarObstacles)
{
    tFloat32 f32ClosestObstacle = 9999.9f;
    TLaserScannerData::Data laserToTransmit;
    laserToTransmit.ui32Size = 0;
    tUInt32 ui32SizeOf = 0;


    for (uint32_t i = 0; i < (LidarObstacles.ui32Size); i++)
    {
        if(LidarObstacles.tScanArrayEval[i].ui32ObstacleCounter != 0)
        {
            //calc the x and the y coordinate of the object detected by the laser
            tFloat32 f32rcos = LidarObstacles.tScanArrayEval[i].f32Radius * cosf((90-LidarObstacles.tScanArrayEval[i].f32Angle) * tFloat(PI) / 180.0f);
            tFloat32 f32rsin = LidarObstacles.tScanArrayEval[i].f32Radius * sinf((90-LidarObstacles.tScanArrayEval[i].f32Angle) * tFloat(PI) / 180.0f);
            if((roi_XY.f32x_min <= f32rcos) && (f32rcos <= roi_XY.f32x_max) && (roi_XY.f32y_min <= f32rsin) && (f32rsin <= roi_XY.f32y_max))
            {
                if(m_propBLaserOutputDrive && m_propBLaserOutputGeneral)
                {
                    laserToTransmit.tScanArray[ui32SizeOf].f32Angle = AngleBackCompensation(LidarObstacles.tScanArrayEval[i].f32Angle);
                    laserToTransmit.tScanArray[ui32SizeOf].f32Radius = LidarObstacles.tScanArrayEval[i].f32Radius;
                    ui32SizeOf++;
                }

                if(LidarObstacles.tScanArrayEval[i].f32Radius < f32ClosestObstacle)
                {
                    f32ClosestObstacle = LidarObstacles.tScanArrayEval[i].f32Radius;
                }
            }
        }
    }
    if(ui32SizeOf != 0 && m_propBLaserOutputDrive && m_propBLaserOutputGeneral)
    {
        //TransmitRelevantLaser(laserToTransmit);
    }

    return f32ClosestObstacle;
}



tUInt32 UltrasonicACC::IsObjectInROI(UltrasonicACC::roiXY roi_XY, UltrasonicACC::LidarObstacles LidarObstacles)
{
    //int ui32Size;
    TLaserScannerData::Data laserOutput;
    laserOutput.ui32Size = 0;
    if(!m_propUI32TemporaryOccupiedROIBoundary)
    {
        //tUInt32 ui32CounterOfObject = 0;
        TLaserScannerData::Data laserOutput;
        laserOutput.ui32Size = 0;
    }
    UltrasonicACC::roiXY roi_XYWithOffset = roi_XY;
    tUInt32 ui32ObjectInROICounter = 0;
    currentIntersectionMode lastIntersectMode = GetIntersectionMode();
    if(lastIntersectMode == NORMAL_DRIVE)
    {
        roi_XYWithOffset.f32y_max += m_propF32DistanceToStopNormal;
        roi_XYWithOffset.f32y_min += m_propF32DistanceToStopNormal;
    }
    else if(lastIntersectMode == AFTER_RIGHT)
    {
        roi_XYWithOffset.f32y_max += m_propF32DistanceToStopAfterRight;
        roi_XYWithOffset.f32y_min += m_propF32DistanceToStopAfterRight;
    }
    else if(lastIntersectMode == AFTER_LEFT)
    {
        roi_XYWithOffset.f32y_max += m_propF32DistanceToStopAfterLeft;
        roi_XYWithOffset.f32y_min += m_propF32DistanceToStopAfterLeft;
    }
    else
    {
        LOG_ERROR("Invalid IntersectionMode: %d ;Continue without an offset", lastIntersectMode);
    }

    for (uint32_t i = 0; i < (LidarObstacles.ui32Size); i++)
    {
        if(LidarObstacles.tScanArrayEval[i].ui32ObstacleCounter != 0)
        {
            //calc the x and the y coordinate of the object detected by the laser
            tFloat32 f32rcos = LidarObstacles.tScanArrayEval[i].f32Radius * cosf((90-LidarObstacles.tScanArrayEval[i].f32Angle) * tFloat(PI) / 180.0f);
            tFloat32 f32rsin = LidarObstacles.tScanArrayEval[i].f32Radius * sinf((90-LidarObstacles.tScanArrayEval[i].f32Angle) * tFloat(PI) / 180.0f);
            //LOG_INFO("f32rcos: %f", f32rcos);
            //LOG_INFO("f32rsin: %f", f32rsin);
            if((roi_XYWithOffset.f32x_min <= f32rcos) && (f32rcos <= roi_XYWithOffset.f32x_max) && (roi_XYWithOffset.f32y_min <= f32rsin) && (f32rsin <= roi_XYWithOffset.f32y_max))
            {
                if(!m_propUI32TemporaryOccupiedROIBoundary)
                {
                    laserOutput.ui32Size = GetSizeOfOutputLaser();
                    laserOutput.tScanArray[laserOutput.ui32Size].f32Radius = LidarObstacles.tScanArrayEval[i].f32Radius;
                    laserOutput.tScanArray[laserOutput.ui32Size].f32Angle = AngleBackCompensation(LidarObstacles.tScanArrayEval[i].f32Angle);
                    IncreaseSizeOfOutputLaser();
                }
                ui32ObjectInROICounter++;

                if((fabs(roi_XYWithOffset.f32x_min-m_propF32CheckCrosswalkXmin) < 0.01) && (fabs(roi_XYWithOffset.f32x_max-m_propF32CheckCrosswalkXmax) < 0.01) && (fabs(roi_XYWithOffset.f32y_min-m_propF32CheckCrosswalkYmin) < 0.01) && (fabs(roi_XYWithOffset.f32y_max-m_propF32CheckCrosswalkYmax) < 0.01))
                {
                    //LOG_INFO("While checking the crosswalk");
                    if(!m_propUI32TemporaryOccupiedROIBoundary)
                    {
                        laserOutput.ui32Size = GetSizeOfOutputLaser();
                        //TransmitRelevantLaser(laserOutput);
                    }

                    return ui32ObjectInROICounter;
                }

                if(ui32ObjectInROICounter >= m_propUI32TemporaryOccupiedROIBoundary)//not neightbors yet
                {
                    if(m_propBDebugROI)LOG_INFO("x: %f", f32rcos);
                    if(m_propBDebugROI)LOG_INFO("y: %f", f32rsin);
                    if(!m_propUI32TemporaryOccupiedROIBoundary)
                    {
                        laserOutput.ui32Size = GetSizeOfOutputLaser();
                        //TransmitRelevantLaser(laserOutput);
                    }
                    return ui32ObjectInROICounter;
                    //break;
                }
            }
        }
    }
    /*
            if((m_propBDebugROI) && (ui32CounterOfObject != 0))LOG_INFO("US_ACC: [IsObjectROI] Found Object: %u", ui32CounterOfObject);
            else if((m_propBDebugROI) && (ui32CounterOfObject == 0))LOG_INFO("No Obstacel in the roi found!");
            */

    return 0;
}

//process input from lane detection only needed in a bend
UltrasonicACC::pointOnTwoLanes UltrasonicACC::ProcessLaneDetectionInput(TLaneDetectionLineStruct::Data lineStruct)
{/*
           rechts -> mittelstreifen
           links -> rechte spur
          */

    tInt32 x1,y1,x2,y2,x3,y3,x4,y4;

    x1 = lineStruct.i32Point1X;
    y1 = lineStruct.i32Point1Y;
    x2 = lineStruct.i32Point2X;
    y2 = lineStruct.i32Point2Y;
    x3 = lineStruct.i32Point3X;
    y3 = lineStruct.i32Point3Y;
    x4 = lineStruct.i32Point4X;
    y4 = lineStruct.i32Point4Y;
    if(lineStruct.i32Point1Y > lineStruct.i32Point2Y)
    {
        x1 = lineStruct.i32Point2X;
        y1 = lineStruct.i32Point2Y;
        x2 = lineStruct.i32Point1X;
        y2 = lineStruct.i32Point1Y;
    }
    if(lineStruct.i32Point3Y > lineStruct.i32Point4Y)
    {
        x3 = lineStruct.i32Point4X;
        y3 = lineStruct.i32Point4Y;
        x4 = lineStruct.i32Point3X;
        y4 = lineStruct.i32Point3Y;
    }

    pointOnTwoLanes twoLanes;
    if((x1 == 0) && (x2 == 0) && (x3 == 0) && (x4 == 0) && (y1 == 0) && (y2 == 0) && (y3 == 0) && (y4 == 0))//not valid data
    {
        /*LOG_WARNING("Invalid laneDetectionLineStruct!");
                twoLanes.f32x1line1 = 0.0;
                twoLanes.f32x1line2 = 0.0;
                twoLanes.f32x2line1 = 0.0;
                twoLanes.f32x2line2 = 0.0;
                twoLanes.f32y1line1 = 0.0;
tResult cMoveToPoint::LoadProperties(){


      //TEST CASE
        LOG_SUCCESS("here in load properties");
        mtp_properties properties;
        LoadXMLData("/home/aadc/AADC/utilities/propertyXMLs/mtp_properties.xml", (void*) (&properties));

        mtp_steeringAngle = properties.mtp_steeringAngle;
        mtp_movementSpeed = properties.mtp_movementSpeed;
        mtp_use_absolute_pos = properties.mtp_use_absolute_pos;
        mtp_stop_after_command = properties.mtp_stop_after_command;
        max_speed = properties.max_speed;
        min_speed = properties.min_speed;
        kalpha = properties.kalpha;
        kbeta_t = properties.kbeta_t;
        offset_steering = properties.offset_steering;
        krho_forw = properties.krho_forw;
        krho_back = properties.krho_back;
        reduction_offset = properties.reduction_offset;
        goalDistanceShutOff = properties.goalDistanceShutOff;


        LOG_SUCCESS(cString::Format("mtp_steeringAngle: %f", (float)mtp_steeringAngle));
        LOG_SUCCESS(cString::Format("mtp_movementSpeed: %f", (float)mtp_movementSpeed));
        LOG_SUCCESS(cString::Format("mtp_use_absolute_pos: %f", (float)mtp_use_absolute_pos));
        LOG_SUCCESS(cString::Format("mtp_stop_after_command: %f", (float)mtp_stop_after_command));
        LOG_SUCCESS(cString::Format("max_speed: %f", (float)max_speed));
        LOG_SUCCESS(cString::Format("min_speed: %f", (float)min_speed));
        LOG_SUCCESS(cString::Format("kalpha: %f", (float)kalpha));
        LOG_SUCCESS(cString::Format("kbeta_t: %f", (float)kbeta_t));
        LOG_SUCCESS(cString::Format("offset_steering: %f", (float)offset_steering));
        LOG_SUCCESS(cString::Format("krho_forw: %f", (float)krho_forw));
        LOG_SUCCESS(cString::Format("krho_back: %f", (float)krho_back));
        LOG_SUCCESS(cString::Format("reduction_offset: %f", (float)reduction_offset));
        LOG_SUCCESS(cString::Format("goalDistanceShutOff: %f", (float)goalDistanceShutOff));

    RETURN_NOERROR;


}  twoLanes.f32y1line2 = 0.0;
                twoLanes.f32y2line1 = 0.0;
                twoLanes.f32y2line2 = 0.0;*/
        twoLanes = GetLastProcessedPoints();
        return twoLanes;
    }


    if(lineStruct.i8IsCurve == 1)//right hand bend
    {
        //inner roadway
        twoLanes.f32x1line1 = (static_cast<tFloat32>(x1))*m_propF32PixToMM;
        twoLanes.f32x2line1 = (static_cast<tFloat32>(x2))*m_propF32PixToMM;
        twoLanes.f32y1line1 = (static_cast<tFloat32>(y1))*m_propF32PixToMM;
        twoLanes.f32y2line1 = (static_cast<tFloat32>(y2))*m_propF32PixToMM;

        //roadway on the right hand side
        twoLanes.f32x1line2 = twoLanes.f32x1line1+m_propF32InnerRoadway;
        twoLanes.f32y1line2 = twoLanes.f32y1line1;
        twoLanes.f32x2line2 = twoLanes.f32x2line1+m_propF32InnerRoadway;
        twoLanes.f32y2line2 = twoLanes.f32y2line1;
    }
    else if(lineStruct.i8IsCurve == -1)//left hand bend
    {
        //roadway on the right hand side
        twoLanes.f32x1line1 = ((static_cast<tFloat32>(x1))*m_propF32PixToMM);//-m_propF32InnerRoadway-f32InnerRoadwayLineWidth-m_propF32InnerRoadway;
        twoLanes.f32x2line1 = ((static_cast<tFloat32>(x2))*m_propF32PixToMM);//-m_propF32InnerRoadway-f32InnerRoadwayLineWidth-m_propF32InnerRoadway;
        twoLanes.f32y1line1 = (static_cast<tFloat32>(y1))*m_propF32PixToMM;
        twoLanes.f32y2line1 = (static_cast<tFloat32>(y2))*m_propF32PixToMM;

        //m_propF32InnerRoadway
        twoLanes.f32x1line2 = twoLanes.f32x1line1-m_propF32InnerRoadway;
        twoLanes.f32y1line2 = twoLanes.f32y1line1;
        twoLanes.f32x2line2 = twoLanes.f32x2line1-m_propF32InnerRoadway;
        twoLanes.f32y2line2 = twoLanes.f32y2line1;

    }
    else
    {
        LOG_ERROR("bend has got an invalid value: %i", lineStruct.i8IsCurve);
    }
    if((twoLanes.f32y1line1 > twoLanes.f32y2line1)||(twoLanes.f32y1line2 > twoLanes.f32y2line2))LOG_ERROR("USACC [ProcessLaneDetectionInput] y1 is greater than y2!");
    SetLastProcessPoints(twoLanes);

    return twoLanes;
}

UltrasonicACC::roiXY UltrasonicACC::CalcXYMinAndMax(tFloat32 f32x_bottom_left_corner, tFloat32 f32y_bottom_left_corner, tFloat32 f32x_height, tFloat32 f32y_width)
{

    //transform the roi to the math definition
    UltrasonicACC::roiXY roi_XY;
    //tFloat f32x_max, f32x_min, f32y_min, f32y_max;
    if (f32y_width <= 0.0f)
    {
        roi_XY.f32x_min = f32y_bottom_left_corner*(-1000.0f);//smalles x -value
        roi_XY.f32x_max = (f32y_bottom_left_corner + f32y_width)*(-1000.0f);
    }
    else
    {
        roi_XY.f32x_min = (f32y_bottom_left_corner + f32y_width)*(-1000.0f);
        roi_XY.f32x_max = f32y_bottom_left_corner*(-1000.0f);
    }
    if (f32x_height <= 0.0f)
    {
        roi_XY.f32y_min = (f32x_bottom_left_corner+f32x_height)*1000+m_propF32OffsetCamLidar;
        roi_XY.f32y_max = f32x_bottom_left_corner*1000+m_propF32OffsetCamLidar;
    }
    else
    {
        roi_XY.f32y_min = f32x_bottom_left_corner*1000+m_propF32OffsetCamLidar;
        roi_XY.f32y_max = (f32x_bottom_left_corner+f32x_height)*1000+m_propF32OffsetCamLidar;
    }

    return roi_XY;

}
/*merge getter and setter */
tResult UltrasonicACC::SetMergeFlag(tBool flag)
{
    boost::lock_guard<boost::mutex> lock(criticalSectorMergeFlag);
    m_bMergeIntoLane = flag;
    RETURN_NOERROR;
}

tBool UltrasonicACC::GetMergeFlag()
{
    boost::lock_guard<boost::mutex> lock(criticalSectorMergeFlag);
    return m_bMergeIntoLane;
}

//for laser only for the roi
tResult UltrasonicACC::IncreaseSizeOfOutputLaser()
{
    boost::lock_guard<boost::mutex> lock(criticalSectionSizeOfOutputLaser);
    m_ui32SizeOfOutput++;
    RETURN_NOERROR;
}

tUInt32 UltrasonicACC::GetSizeOfOutputLaser()
{
    boost::lock_guard<boost::mutex> lock(criticalSectionSizeOfOutputLaser);
    return m_ui32SizeOfOutput;
}

tResult UltrasonicACC::ResetSizeOfOutputLaser()
{
    boost::lock_guard<boost::mutex> lock(criticalSectionSizeOfOutputLaser);
    m_ui32SizeOfOutput = 0;
    RETURN_NOERROR;
}

tResult UltrasonicACC::ProcessDriveOnRamp()
{

    UltrasonicACC::LidarObstacles tmp_obstacles = GetLIDARObstacles();

    tFloat32 f32ClosestPointPosAngle = 999.9;
    tFloat32 f32PosAngleOfClosestPoint = 0.0;
    tFloat32 f32ClosestPointNegAngle = 999.9;
    tFloat32 f32NegAngleOfClosestPoint = 0.0;
    for (tUInt32 i = 0; i < tmp_obstacles.ui32Size; i++)
    {
        if(tmp_obstacles.tScanArrayEval[i].ui32ObstacleCounter != 0)
        {
            //find obstacle with pos angle and closest radius and with neg angle
            if(tmp_obstacles.tScanArrayEval[i].f32Angle < 0)
            {
                if(fabs(f32NegAngleOfClosestPoint) < fabs(tmp_obstacles.tScanArrayEval[i].f32Angle))
                {
                    f32ClosestPointNegAngle = tmp_obstacles.tScanArrayEval[i].f32Radius;
                    f32NegAngleOfClosestPoint = tmp_obstacles.tScanArrayEval[i].f32Angle;
                }
                /*if(tmp_obstacles.tScanArrayEval[i].f32Radius < f32ClosestPointNegAngle)
                {
                    f32ClosestPointNegAngle = tmp_obstacles.tScanArrayEval[i].f32Radius;
                    f32NegAngleOfClosestPoint = tmp_obstacles.tScanArrayEval[i].f32Angle;
                }*/
            }
            else if(tmp_obstacles.tScanArrayEval[i].f32Angle > 0)
            {
                if(fabs(f32PosAngleOfClosestPoint) < fabs(tmp_obstacles.tScanArrayEval[i].f32Angle))
                {
                    f32ClosestPointPosAngle = tmp_obstacles.tScanArrayEval[i].f32Radius;
                    f32PosAngleOfClosestPoint = tmp_obstacles.tScanArrayEval[i].f32Angle;
                }
                /*if(tmp_obstacles.tScanArrayEval[i].f32Radius < f32ClosestPointPosAngle)
                {
                    f32ClosestPointPosAngle = tmp_obstacles.tScanArrayEval[i].f32Radius;
                    f32PosAngleOfClosestPoint = tmp_obstacles.tScanArrayEval[i].f32Angle;
                }*/
            }
        }
    }
    if((f32ClosestPointPosAngle == 999.9) || (f32ClosestPointNegAngle == 999.9))
    {
        if(m_propBDebugRamp)LOG_INFO("One closest Point not found: Pos: %f; Neg: %f", f32ClosestPointPosAngle, f32ClosestPointNegAngle);
    }
    //test it!
    //tFloat32 f32DistanceBetweenClosestPoints = CalcDistanceWithCos(f32ClosestPointNegAngle, f32ClosestPointPosAngle, (f32PosAngleOfClosestPoint-f32NegAngleOfClosestPoint));
    tFloat32 f32xToTransmit = f32ClosestPointPosAngle*cosf((90-f32PosAngleOfClosestPoint) * tFloat(PI) / 180.0f);
    tFloat32 f32y = f32ClosestPointPosAngle*sinf((90-f32PosAngleOfClosestPoint) * tFloat(PI) / 180.0f);
    //if(m_propBDebugRamp)LOG_INFO("On the ramp: distance guardrail: %f", f32DistanceBetweenClosestPoints);
    if((m_propf32ThresholdRampDrivingY > f32y) && (m_propf32ThresholdRampDrivingX > f32xToTransmit))//(f32DistanceBetweenClosestPoints > m_propF32MinLengthOfTheRamp) && (f32DistanceBetweenClosestPoints < m_propF32MaxLengthOfTheRamp))
    {
        //tFloat32 f32rcos = LidarObstacles.tScanArrayEval[i].f32Radius * cosf((90-LidarObstacles.tScanArrayEval[i].f32Angle) * tFloat(PI) / 180.0f);
        tFloat32 f32xToTransmit = f32ClosestPointPosAngle*cosf((90-f32PosAngleOfClosestPoint) * tFloat(PI) / 180.0f);
        //tFloat32 f32rsin = LidarObstacles.tScanArrayEval[i].f32Radius * sinf((90-LidarObstacles.tScanArrayEval[i].f32Angle) * tFloat(PI) / 180.0f);
        tFloat32 f32y = f32ClosestPointPosAngle*sinf((90-f32PosAngleOfClosestPoint) * tFloat(PI) / 180.0f);
        f32xToTransmit = f32xToTransmit/1000; //[m]
        TSignalValue::Data transmitValue;
        transmitValue.f32Value = f32xToTransmit;
        if(m_propBDebugRamp)LOG_INFO("Transmit point: %f", f32xToTransmit);
        if(m_propBDebugRamp)LOG_INFO("y of Transmit point: %f", f32y);
        //transmit points to move to point pos value
        o_TSignalValue.writePin(m_WriterRampGuardrail, (void *) &transmitValue, m_pClock->GetStreamTime());
        lastValidXValue = f32xToTransmit;
        m_ui32CounterNoGuardRail = 0;
        //save points
        //lastLidarPointPosValue.f32Radius = f32ClosestPointPosAngle;
        //lastLidarPointPosValue.f32Angle = f32PosAngleOfClosestPoint;
        //lastLidarPointNegValue.f32Radius = f32ClosestPointNegAngle;
        //lastLidarPointNegValue.f32Angle = f32NegAngleOfClosestPoint;

    }
    else if(m_ui32CounterNoGuardRail < (m_propui32NumNoGuardrailForOffRamp))
    {
        if(m_propBDebugRamp)LOG_INFO("No valid lidar data (in terms of value), transmit the last one: %f", 0.28);
        m_ui32CounterNoGuardRail++;
        //tFloat32 f32xToTransmit = f32ClosestPointPosAngle*cosf((90-f32PosAngleOfClosestPoint) * tFloat(PI) / 180.0f);
        //tFloat32 tmp_xTransmit = lastValidXValue;
        TSignalValue::Data transmitValue;
        transmitValue.f32Value = 0.28;
        if(m_propBDebugRamp)LOG_INFO("No valid lidar data (in terms of value), transmit: %f", f32xToTransmit);
        o_TSignalValue.writePin(m_WriterRampGuardrail, (void *) &transmitValue, m_pClock->GetStreamTime());
    }
    else
    {
        if(m_propBDebugRamp)LOG_INFO("Off the ramp!");
        //off the ramp
        m_bIsOnTheRamp = tFalse;
        m_ui32CounterNoGuardRail = 0;
        lastValidXValue = 0.35;
        RETURN_IF_FAILED(TransmitFeedBackOffRamp());
    }
    RETURN_NOERROR;
}

tResult UltrasonicACC::LoadProperties()
{


    //LOG_ERROR("In load proerties");
    //LOG_SUCCESS("here in load properties");
    usacc_properties properties;
    if(!LoadXMLData("/home/aadc/AADC/utilities/propertyXMLs/usacc_properties.xml", (void*) (&properties))){
        LOG_WARNING("load xml properties didnt work");
        RETURN_NOERROR;
    }

    m_propI32CounterThresholdNoMove = properties.m_propI32CounterThresholdNoMove;
    m_propI32CounterThresholdMoveAgain = properties.m_propI32CounterThresholdMoveAgain;

    m_propF32LowerBoundSpeed = properties.m_propF32LowerBoundSpeed;
    m_propF32SensorOffsetSteering = properties.m_propF32SensorOffsetSteering;
    m_propF32SensorFrontCheckLimit = properties.m_propF32SensorFrontCheckLimit;
    m_propF32ObjectThresholdAngle = properties.m_propF32ObjectThresholdAngle;
    m_propF32ObjectThresholdRadius = properties.m_propF32ObjectThresholdRadius;
    m_propF32ObjectThresholdDetection = properties.m_propF32ObjectThresholdDetection;
    m_propF32CloseObjectThresholdAngle = properties.m_propF32CloseObjectThresholdAngle;
    m_propF32wheelbase = properties.m_propF32wheelbase;
    m_propF32ThresholdVisualRangeBend = properties.m_propF32ThresholdVisualRangeBend;
    m_propF32ThresholdReducedVisualRangeOneSide = properties.m_propF32ThresholdReducedVisualRangeOneSide;
    m_propF32OffsetParkingY = properties.m_propF32OffsetParkingY;
    m_propF32DistanceToStopLine = properties.m_propF32DistanceToStopLine;
    m_propF32PixToMM = properties.m_propF32PixToMM;
    m_propF32CamOffsety = properties.m_propF32CamOffsety;
    m_propF32XminOfRamp = properties.m_propF32XminOfRamp;
    m_propF32XmaxOfRamp = properties.m_propF32XmaxOfRamp;
    m_propF32IntersectionOncomingXmin = properties.m_propF32IntersectionOncomingXmin;
    m_propF32IntersectionOncomingYmin = properties.m_propF32IntersectionOncomingYmin;
    m_propF32IntersectionOncomingXmax = properties.m_propF32IntersectionOncomingXmax;
    m_propF32IntersectionOncomingYmax = properties.m_propF32IntersectionOncomingYmax;
    m_propF32IntersectionCrosstrafficRightXmin = properties.m_propF32IntersectionCrosstrafficRightXmin;
    m_propF32IntersectionCrosstrafficRightYmin = properties.m_propF32IntersectionCrosstrafficRightYmin;
    m_propF32IntersectionCrosstrafficRightXmax = properties.m_propF32IntersectionCrosstrafficRightXmax;
    m_propF32IntersectionCrosstrafficRightYmax = properties.m_propF32IntersectionCrosstrafficRightYmax;
    m_propF32IntersectionCrosstrafficLeftXmin = properties.m_propF32IntersectionCrosstrafficLeftXmin;
    m_propF32IntersectionCrosstrafficLeftYmin = properties.m_propF32IntersectionCrosstrafficLeftYmin;
    m_propF32IntersectionCrosstrafficLeftXmax = properties.m_propF32IntersectionCrosstrafficLeftXmax;
    m_propF32IntersectionCrosstrafficLeftYmax = properties.m_propF32IntersectionCrosstrafficLeftYmax;
    m_propF32OvertakeOncomingXmin = properties.m_propF32OvertakeOncomingXmin;
    m_propF32OvertakeOncomingYmin = properties.m_propF32OvertakeOncomingYmin;
    m_propF32OvertakeOncomingXmax = properties.m_propF32OvertakeOncomingXmax;
    m_propF32OvertakeOncomingYmax = properties.m_propF32OvertakeOncomingYmax;
    m_propF32OvertakeOriginallaneXmin = properties.m_propF32OvertakeOriginallaneXmin;
    m_propF32OvertakeOriginallaneYmin = properties.m_propF32OvertakeOriginallaneYmin;
    m_propF32OvertakeOriginallaneXmax = properties.m_propF32OvertakeOriginallaneXmax;
    m_propF32OvertakeOriginallaneYmax = properties.m_propF32OvertakeOriginallaneYmax;
    m_propF32OvertakeObstacleStraightXmin = properties.m_propF32OvertakeObstacleStraightXmin;
    m_propF32OvertakeObstacleStraightYmin = properties.m_propF32OvertakeObstacleStraightYmin;
    m_propF32OvertakeObstacleStraightXmax = properties.m_propF32OvertakeObstacleStraightXmax;
    m_propF32OvertakeObstacleStraightYmax = properties.m_propF32OvertakeObstacleStraightYmax;
    m_propF32OvertakeObstacleStraightLeftHalfXmin = properties.m_propF32OvertakeObstacleStraightLeftHalfXmin;
    m_propF32OvertakeObstacleStraightLeftHalfYmin = properties.m_propF32OvertakeObstacleStraightLeftHalfYmin;
    m_propF32OvertakeObstacleStraightLeftHalfXmax = properties.m_propF32OvertakeObstacleStraightLeftHalfXmax;
    m_propF32OvertakeObstacleStraightLeftHalfYmax = properties.m_propF32OvertakeObstacleStraightLeftHalfYmax;
    m_propF32ParkingCrossOncomingLaneXmin = properties.m_propF32ParkingCrossOncomingLaneXmin;
    m_propF32ParkingCrossOncomingLaneYmin = properties.m_propF32ParkingCrossOncomingLaneYmin;
    m_propF32ParkingCrossOncomingLaneXmax = properties.m_propF32ParkingCrossOncomingLaneXmax;
    m_propF32ParkingCrossOncomingLaneYmax = properties.m_propF32ParkingCrossOncomingLaneYmax;
    m_propF32ParkingPullOutCrossOncoming1Xmin = properties.m_propF32ParkingPullOutCrossOncoming1Xmin;
    m_propF32ParkingPullOutCrossOncoming1Xmax = properties.m_propF32ParkingPullOutCrossOncoming1Xmax;
    m_propF32ParkingPullOutCrossOncoming1Ymin = properties.m_propF32ParkingPullOutCrossOncoming1Ymin;
    m_propF32ParkingPullOutCrossOncoming1Ymax = properties.m_propF32ParkingPullOutCrossOncoming1Ymax;
    m_propF32CheckCrosswalkXmin = properties.m_propF32CheckCrosswalkXmin;
    m_propF32CheckCrosswalkXmax = properties.m_propF32CheckCrosswalkXmax;
    m_propF32CheckCrosswalkYmin = properties.m_propF32CheckCrosswalkYmin;
    m_propF32CheckCrosswalkYmax = properties.m_propF32CheckCrosswalkYmax;
    m_propF32OvertakeObstacleOnlyRightHandSideXmin = properties.m_propF32OvertakeObstacleOnlyRightHandSideXmin;
    m_propF32OvertakeObstacleOnlyRightHandSideYmin = properties.m_propF32OvertakeObstacleOnlyRightHandSideYmin;
    m_propF32OvertakeObstacleOnlyRightHandSideXmax = properties.m_propF32OvertakeObstacleOnlyRightHandSideXmax;
    m_propF32OvertakeObstacleOnlyRightHandSideYmax = properties.m_propF32OvertakeObstacleOnlyRightHandSideYmax;
    m_propF32ThresholdSlopeForRamp = properties.m_propF32ThresholdSlopeForRamp;
    m_propF32ThresholdDifferenzYNotOnLine = properties.m_propF32ThresholdDifferenzYNotOnLine;
    m_propF32MinLengthOfTheRamp = properties.m_propF32MinLengthOfTheRamp;
    m_propF32MaxLengthOfTheRamp = properties.m_propF32MaxLengthOfTheRamp;
    m_propF32YDiffRamp = properties.m_propF32YDiffRamp;
    m_propF32XDiffMax = properties.m_propF32XDiffMax;
    m_propF32MergeCheckTargetLaneXmin = properties.m_propF32MergeCheckTargetLaneXmin;
    m_propF32MergeCheckTargetLaneYmin = properties.m_propF32MergeCheckTargetLaneYmin;
    m_propF32MergeCheckTargetLaneXmax = properties.m_propF32MergeCheckTargetLaneXmax;
    m_propF32MergeCheckTargetLaneYmax = properties.m_propF32MergeCheckTargetLaneYmax;

    m_propBLaserOutputDrive = properties.m_propBLaserOutputDrive;
    m_propBDebugFirstStart = properties.m_propBDebugFirstStart;
    m_propBDebugROI = properties.m_propBDebugROI;
    m_propBDebugCurve = properties.m_propBDebugCurve;
    m_propBDebugActionCommand = properties.m_propBDebugActionCommand;
    m_propBDebugOvertakingROI = properties.m_propBDebugOvertakingROI;
    m_propBDebugStraight = properties.m_propBDebugStraight;
    m_propBDebugRamp = properties.m_propBDebugRamp;
    m_propUI32FreeROIBoundary = properties.m_propUI32FreeROIBoundary;
    m_propUI32OccupiedROIBoundary = properties.m_propUI32OccupiedROIBoundary;
    m_propUI32TemporaryOccupiedROIBoundary = properties.m_propUI32TemporaryOccupiedROIBoundary;
    m_propUI32EndOfForLoopToCheckROI = properties.m_propUI32EndOfForLoopToCheckROI;
    m_propUI32Frames = properties.m_propUI32Frames;
    m_propROIPoint1X = properties.m_propROIPoint1X;
    m_propROIPoint1Y = properties.m_propROIPoint1Y;
    m_propROIPoint2X = properties.m_propROIPoint2X;
    m_propROIPoint2Y = properties.m_propROIPoint2Y;
    m_propROIPoint3X = properties.m_propROIPoint3X;
    m_propROIPoint3Y = properties.m_propROIPoint3Y;
    m_propROIPoint4X = properties.m_propROIPoint4X;
    m_propROIPoint4Y = properties.m_propROIPoint4Y;
    m_propTransformedImageWidth = properties.m_propTransformedImageWidth;
    m_propTransformedImageHeight = properties.m_propTransformedImageHeight;
    m_propui32MinNumOfPointsRamp = properties.m_propui32MinNumOfPointsRamp;
    m_propui32NumberLaneDetectionInputForRamp = properties.m_propui32NumberLaneDetectionInputForRamp;



    /*LOG_SUCCESS(cString::Format("m_propI32CounterThresholdNoMove: %d", (int)(m_propI32CounterThresholdNoMove)));
    LOG_SUCCESS(cString::Format("m_propI32CounterThresholdMoveAgain: %d", (int) m_propI32CounterThresholdMoveAgain));
    LOG_SUCCESS(cString::Format("m_propF32LowerBoundSpeed: %f", (float)m_propF32LowerBoundSpeed));
    LOG_SUCCESS(cString::Format("m_propF32SensorOffsetSteering: %f", (float)m_propF32SensorOffsetSteering));
    LOG_SUCCESS(cString::Format("m_propF32SensorFrontCheckLimit: %f", (float)m_propF32SensorFrontCheckLimit));
    LOG_SUCCESS(cString::Format("m_propF32ObjectThresholdAngle: %f", (float)m_propF32ObjectThresholdAngle));
    LOG_SUCCESS(cString::Format("m_propF32ObjectThresholdRadius: %f", (float)m_propF32ObjectThresholdRadius));
    LOG_SUCCESS(cString::Format("m_propF32ObjectThresholdDetection: %f", m_propF32ObjectThresholdDetection));
    LOG_SUCCESS(cString::Format("m_propF32CloseObjectThresholdAngle: %f", m_propF32CloseObjectThresholdAngle));
    LOG_SUCCESS(cString::Format("m_propF32wheelbase: %f", m_propF32wheelbase));
    LOG_SUCCESS(cString::Format("m_propF32ThresholdVisualRangeBend: %f", m_propF32ThresholdVisualRangeBend));
    LOG_SUCCESS(cString::Format("m_propF32ThresholdReducedVisualRangeOneSide: %f", m_propF32ThresholdReducedVisualRangeOneSide));
    LOG_SUCCESS(cString::Format("m_propF32OffsetParkingY: %f", m_propF32OffsetParkingY));
    LOG_SUCCESS(cString::Format("m_propF32DistanceToStopLine: %f", m_propF32DistanceToStopLine));
    LOG_SUCCESS(cString::Format("m_propF32PixToMM: %f", m_propF32PixToMM));
    LOG_SUCCESS(cString::Format("m_propF32CamOffsety: %f", m_propF32CamOffsety));
    LOG_SUCCESS(cString::Format("m_propF32XminOfRamp: %f", m_propF32XminOfRamp));
    LOG_SUCCESS(cString::Format("m_propF32XmaxOfRamp: %f", m_propF32XmaxOfRamp));
    LOG_SUCCESS(cString::Format("m_propF32IntersectionOncomingXmin: %f", m_propF32IntersectionOncomingXmin));
    LOG_SUCCESS(cString::Format("m_propF32IntersectionOncomingYmin: %f", m_propF32IntersectionOncomingYmin));
    LOG_SUCCESS(cString::Format("m_propF32IntersectionOncomingXmax: %f", m_propF32IntersectionOncomingXmax));
    LOG_SUCCESS(cString::Format("m_propF32IntersectionOncomingYmax : %f", m_propF32IntersectionOncomingYmax));
    LOG_SUCCESS(cString::Format("m_propF32IntersectionCrosstrafficRightXmin: %f", m_propF32IntersectionCrosstrafficRightXmin));
    LOG_SUCCESS(cString::Format("m_propF32IntersectionCrosstrafficRightYmin: %f", m_propF32IntersectionCrosstrafficRightYmin));
    LOG_SUCCESS(cString::Format("m_propF32IntersectionCrosstrafficRightXmax: %f", m_propF32IntersectionCrosstrafficRightXmax));
    LOG_SUCCESS(cString::Format("m_propF32IntersectionCrosstrafficRightYmax: %f", m_propF32IntersectionCrosstrafficRightYmax));
    LOG_SUCCESS(cString::Format("m_propF32IntersectionCrosstrafficLeftXmin: %f", m_propF32IntersectionCrosstrafficLeftXmin));
    LOG_SUCCESS(cString::Format("m_propF32IntersectionCrosstrafficLeftYmin: %f", m_propF32IntersectionCrosstrafficLeftYmin));
    LOG_SUCCESS(cString::Format("m_propF32IntersectionCrosstrafficLeftXmax: %f", m_propF32IntersectionCrosstrafficLeftXmax));
    */
    /*
    LOG_SUCCESS(cString::Format("m_propF32YDiffRamp: %f", (float)m_propF32YDiffRamp));
    LOG_SUCCESS(cString::Format("m_propF32XDiffMax: %f", (float)m_propF32XDiffMax));
    LOG_SUCCESS(cString::Format("m_propF32MergeCheckTargetLaneXmin: %f", (float)m_propF32MergeCheckTargetLaneXmin));
    LOG_SUCCESS(cString::Format("m_propF32MergeCheckTargetLaneXmax: %f", (float)m_propF32MergeCheckTargetLaneXmax));
    LOG_SUCCESS(cString::Format("m_propF32MergeCheckTargetLaneYmin: %f", (float)m_propF32MergeCheckTargetLaneYmin));
    LOG_SUCCESS(cString::Format("m_propF32MergeCheckTargetLaneYmax: %f", (float)m_propF32MergeCheckTargetLaneYmax));*/
    RETURN_NOERROR;


}
