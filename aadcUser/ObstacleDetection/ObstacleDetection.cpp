/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. Team FAUtonomous. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

TODO: write a short description of your filter!!
This filter is our template filter for aadc2018.
Use this filter for creating your own filters.
Keep in mind to change "ObstacleDetection" to your filtername.

**********************************************************************/

#include "ObstacleDetection.h"

//#define DEBUG_MODE_OUTPUT_VIDEO
//#define

//****************************************************************************************************
//******                               DEFINE ROIs                                              ******
//****************************************************************************************************

/* macro for size of CV_16UC1 for validation image*/
#define OD_VI_CV8UC1_MAX_RANGE 255 // (256 -1 )
#define OD_VI_CV16UC1_MAX_RANGE 65535 // (256*256 - 1)

/* IMPORTANT: also check DEBUG_MODE_OUTPUT_VIDEO in header file */

/* Regions of interest */
/* INTERSECTION */
/* Region for oncoming traffic at intersection*/
#define OD_ROI_INTERSECTION_ONCOMING_BOTTOMLEFT_X 1.10f
#define OD_ROI_INTERSECTION_ONCOMING_BOTTOMLEFT_Y 0.6f
#define OD_ROI_INTERSECTION_ONCOMING_HEIGHT_X 0.6f		// magnitude expresses height, sign expresses coordinate direction (positive ->straight ahead)
#define OD_ROI_INTERSECTION_ONCOMING_WIDTH_Y -0.3f		// magnitude expresses width, sign expresses coordinate direction (positive -> to left hand side)

/* Region for cross-traffic from right hand side at intersection */
#define OD_ROI_INTERSECTION_CROSSTRAFFIC_BOTTOMLEFT_X 0.6f
#define OD_ROI_INTERSECTION_CROSSTRAFFIC_BOTTOMLEFT_Y -0.2f
#define OD_ROI_INTERSECTION_CROSSTRAFFIC_HEIGHT_X 0.35f	// magnitude expresses height, sign expresses coordinate direction (positive ->straight ahead)
#define OD_ROI_INTERSECTION_CROSSTRAFFIC_WIDTH_Y -0.4f	// magnitude expresses width, sign expresses coordinate direction (positive -> to left hand side)

/* OVERTAKING */
/* Region for oncoming traffic during drive-mode */
#define OD_ROI_OVERTAKE_ONCOMING_BOTTOMLEFT_X 0.3f
#define OD_ROI_OVERTAKE_ONCOMING_BOTTOMLEFT_Y 0.55f
#define OD_ROI_OVERTAKE_ONCOMING_HEIGHT_X 0.9f	// magnitude expresses height, sign expresses coordinate direction (positive ->straight ahead)
#define OD_ROI_OVERTAKE_ONCOMING_WIDTH_Y -0.3f	// magnitude expresses width, sign expresses coordinate direction (positive -> to left hand side)

/* Region for traffic on original lane during overtake, right before going back into original lane */
#define OD_ROI_OVERTAKE_ORIGINALLANE_BOTTOMLEFT_X 0.3f
#define OD_ROI_OVERTAKE_ORIGINALLANE_BOTTOMLEFT_Y -0.27f
#define OD_ROI_OVERTAKE_ORIGINALLANE_HEIGHT_X 0.9f	// magnitude expresses height, sign expresses coordinate direction (positive ->straight ahead)
#define OD_ROI_OVERTAKE_ORIGINALLANE_WIDTH_Y -0.3f	// magnitude expresses width, sign expresses coordinate direction (positive -> to left hand side)

/* DETECTION OF STATIC OBSTACLE IN FRONT ON STRAIGHT LANE */
/* Region for traffic ahead on own current lane IN STRAIGHT DRIVING, AC_OD_OVERTAKE_CHECK_OWN_LANE_STRAIGHT*/
#define OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_BOTTOMLEFT_X 0.05f
#define OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_BOTTOMLEFT_Y 0.15f
#define OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_HEIGHT_X 0.9f	// magnitude expresses height, sign expresses coordinate direction (positive ->straight ahead)
#define OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_WIDTH_Y -0.3f	// magnitude expresses width, sign expresses coordinate direction (positive -> to left hand side)

/* DETECTION OF STATIC OBSTACLE IN FRONT LEFT HALF ON STRAIGHT LANE */
/* Region for traffic ahead on own current lane IN STRAIGHT DRIVING, AC_OD_OVERTAKE_CHECK_OWN_LANE_STRAIGHT_LEFTHALF*/
#define OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_LEFTHALF_BOTTOMLEFT_X 0.05f
#define OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_LEFTHALF_BOTTOMLEFT_Y 0.3f
#define OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_LEFTHALF_HEIGHT_X 0.9f	// magnitude expresses height, sign expresses coordinate direction (positive ->straight ahead)
#define OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_LEFTHALF_WIDTH_Y -0.2f	// magnitude expresses width, sign expresses coordinate direction (positive -> to left hand side)

/* REGIONS RIGHT BEFORE PARKING OR PULL OUT MANEUVERS */
/* Region for traffic on oncoming lane that needs to be checked before starting cross-parking maneuver */
#define OD_ROI_PARKING_CROSS_ONCOMINGLANE_BOTTOMLEFT_X 0.1f
#define OD_ROI_PARKING_CROSS_ONCOMINGLANE_BOTTOMLEFT_Y 0.45f
#define OD_ROI_PARKING_CROSS_ONCOMINGLANE_HEIGHT_X 0.7f	// magnitude expresses height, sign expresses coordinate direction (positive ->straight ahead)
#define OD_ROI_PARKING_CROSS_ONCOMINGLANE_WIDTH_Y -0.35f	// magnitude expresses width, sign expresses coordinate direction (positive -> to left hand side)

/* Region for traffic on lane in front that needs to be checked before starting pull out from cross-parking maneuver */
#define OD_ROI_PARKING_CROSS_CROSSTRAFFIC_RIGHT_BOTTOMLEFT_X 0.1f
#define OD_ROI_PARKING_CROSS_CROSSTRAFFIC_RIGHT_BOTTOMLEFT_Y 0.7f
#define OD_ROI_PARKING_CROSS_CROSSTRAFFIC_RIGHT_HEIGHT_X 0.65f	// magnitude expresses height, sign expresses coordinate direction (positive ->straight ahead)
#define OD_ROI_PARKING_CROSS_CROSSTRAFFIC_RIGHT_WIDTH_Y -1.4f	// magnitude expresses width, sign expresses coordinate direction (positive -> to left hand side)

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
    CID_OBSTACLE_DETECTION_FILTER,		// references to header file
    "ObstacleDetection",              // label
    ObstacleDetection,                // class
    adtf::filter::pin_trigger({"action_struct", "laserscanner_data", "steering_angle", "target_speed"}));	// set trigger pin
//TODO: trigger correct?



//****************************************************************************************************
//******                                  DEFINE PARAMETERS                                     ******
//****************************************************************************************************

#define OD_PITCH_STORAGE "Camera parameters::Camera m_f32Rotation::pitch angle file"
#define OD_CAMERA_PITCH "Camera parameters::Camera m_f32Rotation::pitch"
#define OD_USE_PITCH_FROM_FILE "Camera parameters::Camera m_f32Rotation::use pitch angle from file"
#define OD_CAMERA_PITCH_CORRECTION "Camera parameters::Camera m_f32Rotation::pitch correction factor"

/* Focal length and center points of camera */
#define OD_CAMERA_FX "Camera parameters::Camera calibration::focal length in x"
#define OD_CAMERA_FY "Camera parameters::Camera calibration::focal length in y"
#define OD_CAMERA_CX "Camera parameters::Camera calibration::center point x"
#define OD_CAMERA_CY "Camera parameters::Camera calibration::center point y"

// Scale factor for depth image in meter
#define OD_DEPTH_IMAGE_SCALE_FACTOR "Camera parameters::Depth Image::scale factor"

#define OD_CAM_POSITION_OFFSET_X "Camera parameters::Camera position::offset in x direction"
#define OD_CAM_POSITION_OFFSET_Y "Camera parameters::Camera position::offset in y direction"
#define OD_CAM_POSITION_OFFSET_Z "Camera parameters::Camera position::offset in z direction"
#define OD_CAM_POSITION_OFFSET_DEPTH_TO_RGB "Camera parameters::Camera position::offset between depth-cam and rgb-cam"

//#define OD_CAR_CARWIDTH "Field of view::width of car"
#define OD_CAR_MAX_FIELD_OF_VIEW_X "Check ROI::Field of view::x:maximal field of view in x-direction (distance)"
#define OD_CAR_MIN_FIELD_OF_VIEW_X "Check ROI::Field of view::x:minimal field of view in x-direction (distance)"
#define OD_CAR_MAX_FIELD_OF_VIEW_Z "Check ROI::Field of view::z:maximal field of view in z-direction (height)"
#define OD_CAR_MIN_FIELD_OF_VIEW_Z "Check ROI::Field of view::z:minimal field of view in z-direction (height)"

#define OD_VALIDMASK_MIN_FIELD_OF_VIEW_Z "Validation Mask::Max 'object height' for lines regarded 'valid'"

#define OD_CHECK_ROI_INTERSECTION_FREEFRAME_LOWERBOUND "Check ROI::Intersection::lower bound of 'free-frames'"
#define OD_CHECK_ROI_INTERSECTION_OCCUPIED_CROSSRIGHT_UPPERBOUND "Check ROI::Intersection::upper bound of accepted hits for 'cross-traffic-right'"
#define OD_CHECK_ROI_INTERSECTION_OCCUPIED_ONCOMING_UPPERBOUND "Check ROI::Intersection::upper bound of accepted hits for 'oncoming traffic'"
#define OD_CHECK_ROI_INTERSECTION_OCCUPIEDCOUNTER_LOWERBOUND "Check ROI::Intersection::lower bound of 'occupied-frames' for 'static-obstacle-status'"

#define OD_CHECK_ROI_OVERTAKE_OBSTACLESTRAIGHT_FREEFRAME_LOWERBOUND "Check ROI::Overtake::straight in front::lower bound of 'free-frames' for static obstacle in front"
#define OD_CHECK_ROI_OVERTAKE_OBSTACLESTRAIGHT_OCCUPIED_UPPERBOUND "Check ROI::Overtake::straight in front::upper bound of accepted hits for static obstacle in front"
#define OD_CHECK_ROI_OVERTAKE_OBSTACLESTRAIGHT_OCCUPIEDCOUNTER_LOWERBOUND "Check ROI::Overtake::straight in front::lower bound for 'occupied frames' for 'static-obstacle-status'"

#define OD_CHECK_ROI_OVERTAKE_FREEFRAME_LOWERBOUND "Check ROI::Overtake::oncoming lane::lower bound of 'free-frames'"
#define OD_CHECK_ROI_OVERTAKE_OCCUPIED_ONCOMING_UPPERBOUND "Check ROI::Overtake::oncoming lane::upper bound of accepted hits for 'oncoming traffic'"
#define OD_CHECK_ROI_OVERTAKE_ONCOMINGLANE_OCCUPIEDCOUNTER_LOWERBOUND "Check ROI::Overtake::oncoming lane::lower bound for 'occupied frames' for 'static-obstacle-status'"

#define OD_CHECK_ROI_PARKING_OCCUPIED_CROSSRIGHT_UPPERBOUND "Check ROI::Cross Parking::pull-out: upper bound of accepted hits for 'cross-traffic'"
#define OD_CHECK_ROI_PARKING_OCCUPIED_ONCOMING_UPPERBOUND "Check ROI::Cross Parking::oncoming: upper bound of accepted hits for 'oncoming-traffic'"
#define OD_CHECK_ROI_PARKING_OCCUPIEDCOUNTER_LOWERBOUND "Check ROI::Cross Parking::lower bound for 'occupied frames' for 'static-obstacle-status'"
#define OD_CHECK_ROI_PARKING_CROSSTRAFFIC_FREEFRAME_LOWERBOUND "Check ROI::Cross Parking::pull-out: lower bound of 'free-frames'"
#define OD_CHECK_ROI_PARKING_ONCOMING_FREEFRAME_LOWERBOUND "Check ROI::Cross Parking::oncoming: lower bound of 'free-frames'"

#define OD_DEBUG_TYPE "Check ROI::Debug::Debug Type"

#define OD_OACC_PAR_WHEELBASE "OpticalACC::parameters::wheelbase"
//#define OD_OACC_PAR_CARWIDTH "OpticalACC::parameters::carwidth"
#define OD_OACC_PAR_AXISTOBUMPER_FRONT "OpticalACC::parameters::distance front axis to front bumper"
#define OD_OACC_PAR_OBLIQUEANGLE_FRONT "OpticalACC::parameters::oblique angle front"
// #define OD_OACC_PAR_OBLIQUEANGLE_REAR "OpticalACC::parameters::oblique angle rear" // not yet in use
#define OD_OACC_CONFIG_XML_FILE "OpticalACC::parameters::xml-configuration-file speed-reduction"

#define OD_OACC_ROI_SEGMENTLINE "OpticalACC::ROI::length of segment-line (check-distance)"
#define OD_OACC_ROI_Y_SEARCHTOLERANCE "OpticalACC::ROI::width/2 of search-corridor in y-direction"
#define OD_OACC_ROI_HEIGHTTHRESHOLD_MIN "OpticalACC::ROI::minimal obstacle height threshold"
#define OD_OACC_ROI_HEIGHTTHRESHOLD_MAX "OpticalACC::ROI::maximal obstacle height threshold"

#define OD_OACC_DEPTTOCOLOR_X "OpticalACC::DeptToColor::X"
#define OD_OACC_DEPTTOCOLOR_Y "OpticalACC::DeptToColor::Y"
#define OD_OACC_DEPTTOCOLOR_Z "OpticalACC::DeptToColor::Z"

#define OD_OACC_OCCUPANCY_GRID_CELLRESOLUTION "OpticalACC::OccupancyGrid::Resolution of gridcell element"
#define OD_OACC_OCCUPANCY_GRID_CELLCOUNTERTHRESHOLD "OpticalACC::OccupancyGrid::Counter-threshold for gridcell to be seen as occupied"
#define OD_OACC_OCCUPANCY_GRID_DEBUG_LOG "OpticalACC::OccupancyGrid::Debug messages"

#define OD_OACC_DEBUG_SHOW_OAAC_BINARYIMAGE_DYNAMIC "OpticalACC::Debug::enable output dynamic driving lane as binary"
#define OD_OACC_DEBUG_INFO "OpticalACC::Debug::log pixel-count to console"
#define OD_OACC_DEBUG_XML_BORDER_WARNING "OpticalACC::Debug::log xml border warning"
#define OD_OACC_DEBUG_XML_PRINT_INITIAL_TABLE "OpticalACC::Debug::print xml table"
#define OD_GCL_DEBUG_INFO "OpticalACC::Debug::GCL Debug output"
#define OD_GCL_DEBUG_INFO_LOG "OpticalACC::Debug::GCL Debug Log"

#define OD_DEBUG_TO_CONSOLE "Debug::Debug Output to Console"
#define OD_EXTENDED_DEBUG_TO_CONSOLE "Debug::Extended Debug Output to Console"


//****************************************************************************************************
//******                                  CONSTRUCTOR                                           ******
//******                     (Set Properties, Register In- & Outputs)                           ******
//****************************************************************************************************

ObstacleDetection::ObstacleDetection()
{
    //***********************
    // Register properties
    //***********************

    RegisterPropertyVariable(OD_PITCH_STORAGE, m_propstrPitchStorage);
    RegisterPropertyVariable(OD_USE_PITCH_FROM_FILE, m_propbUsePitchFromFile);
    RegisterPropertyVariable(OD_CAMERA_PITCH, m_propui32CameraPitch);
    RegisterPropertyVariable(OD_CAMERA_PITCH_CORRECTION, m_propi32CameraPitchCorrection);
    RegisterPropertyVariable(OD_CAM_POSITION_OFFSET_X, m_propf32CamPositionOffsetX);
    RegisterPropertyVariable(OD_CAM_POSITION_OFFSET_Y, m_propf32CamPositionOffsetY);
    RegisterPropertyVariable(OD_CAM_POSITION_OFFSET_Z, m_propf32CamPositionOffsetZ);
//TODO: Needed?
//    RegisterPropertyVariable(OD_CAM_POSITION_OFFSET_DEPTH_TO_RGB, m_propf32CamPositionOffsetDepthToRGB);

    /* Focal length and center points of camera */
    RegisterPropertyVariable(OD_CAMERA_FX, m_propf32CameraFX);
    RegisterPropertyVariable(OD_CAMERA_FY, m_propf32CameraFy);
    RegisterPropertyVariable(OD_CAMERA_CX, m_propf32CameraCX);
    RegisterPropertyVariable(OD_CAMERA_CY, m_propf32CameraCY);

    RegisterPropertyVariable(OD_DEPTH_IMAGE_SCALE_FACTOR, m_propf32DepthImageScaleFactor);

//	RegisterPropertyVariable(OD_CAR_CARWIDTH, 0.40);
//	SetPropertyStr(OD_CAR_CARWIDTH NSSUBPROP_DESCRIPTION, "Width of the car in meter.");

    RegisterPropertyVariable(OD_CAR_MAX_FIELD_OF_VIEW_X, m_propf32CarMaxFieldOfViewX);
    RegisterPropertyVariable(OD_CAR_MIN_FIELD_OF_VIEW_X, m_propf32CarMinFieldOfViewX);
    RegisterPropertyVariable(OD_CAR_MAX_FIELD_OF_VIEW_Z, m_propf32CarMaxFieldOfViewZ);
    RegisterPropertyVariable(OD_CAR_MIN_FIELD_OF_VIEW_Z, m_propf32CarMinFieldOfViewZ);
    RegisterPropertyVariable(OD_VALIDMASK_MIN_FIELD_OF_VIEW_Z, m_propf32ValidMaskMinFieldOfViewZ);

    RegisterPropertyVariable(OD_CHECK_ROI_INTERSECTION_FREEFRAME_LOWERBOUND, m_propi32RoiIntersectionFreeframeLowerbound);
    RegisterPropertyVariable(OD_CHECK_ROI_INTERSECTION_OCCUPIED_ONCOMING_UPPERBOUND, m_propi32RoiOccupiedOncomingUpperbound);
    RegisterPropertyVariable(OD_CHECK_ROI_INTERSECTION_OCCUPIED_CROSSRIGHT_UPPERBOUND, m_propi32RoiOccupiedCrossrightUpperbound);
    RegisterPropertyVariable(OD_CHECK_ROI_INTERSECTION_OCCUPIEDCOUNTER_LOWERBOUND, m_propi32RoiOccupiedCounterLowerbound);
    RegisterPropertyVariable(OD_CHECK_ROI_OVERTAKE_FREEFRAME_LOWERBOUND, m_propi32RoiOvertakeFreeframeLowerbound);
    RegisterPropertyVariable(OD_CHECK_ROI_OVERTAKE_OCCUPIED_ONCOMING_UPPERBOUND, m_propi32RoiOvertakeOccupiedOncomingUpperbound);
    RegisterPropertyVariable(OD_CHECK_ROI_OVERTAKE_OBSTACLESTRAIGHT_FREEFRAME_LOWERBOUND, m_propi32RoiOvertakeObstaclestraightFreeframeLowerbound);
    RegisterPropertyVariable(OD_CHECK_ROI_OVERTAKE_OBSTACLESTRAIGHT_OCCUPIED_UPPERBOUND, m_propi32RoiOvertakeObstaclestraightOccupiedUpperbound);
    RegisterPropertyVariable(OD_CHECK_ROI_OVERTAKE_OBSTACLESTRAIGHT_OCCUPIEDCOUNTER_LOWERBOUND, m_propi32RoiOvertakeObstaclestraightOccupiedCounterLowerbound);
    RegisterPropertyVariable(OD_CHECK_ROI_OVERTAKE_ONCOMINGLANE_OCCUPIEDCOUNTER_LOWERBOUND, m_propi32RoiOvertakeOncomoinglaneOccupiedcounterLowerbound);

    RegisterPropertyVariable(OD_CHECK_ROI_PARKING_OCCUPIED_ONCOMING_UPPERBOUND, m_propi32RoiParkingOccupiedOncomoingUpperbound);
    RegisterPropertyVariable(OD_CHECK_ROI_PARKING_OCCUPIED_CROSSRIGHT_UPPERBOUND, m_propi32RoiParkingOccupiedCrossrightUpperbound);
    RegisterPropertyVariable(OD_CHECK_ROI_PARKING_OCCUPIEDCOUNTER_LOWERBOUND, m_propi32RoiParkingOccupiedcounterLowerbound);
    RegisterPropertyVariable(OD_CHECK_ROI_PARKING_CROSSTRAFFIC_FREEFRAME_LOWERBOUND, m_propi32RoiParkingCrosstrafficFreeframeLowerbound);
    RegisterPropertyVariable(OD_CHECK_ROI_PARKING_ONCOMING_FREEFRAME_LOWERBOUND, m_propi32RoiParkingOncomingFreeframeLowerbound);

    RegisterPropertyVariable(OD_OACC_PAR_WHEELBASE, m_propf32AccParWheelbase);
//	RegisterPropertyVariable(OD_OACC_PAR_CARWIDTH, m_propf32AccParCarwidth);
    RegisterPropertyVariable(OD_OACC_PAR_AXISTOBUMPER_FRONT, m_propf32AccParAxisToBumperFront);
    RegisterPropertyVariable(OD_OACC_PAR_OBLIQUEANGLE_FRONT, m_propf32AccParObliqueAngleFront);
    //RegisterPropertyVariable(OD_OACC_PAR_OBLIQUEANGLE_REAR, m_propf32AccParObliqueAngleRear);
    SetPropertyStr(OD_OACC_CONFIG_XML_FILE, m_propstringAccConfigXmlFile);

//    SetPropertyBool(OD_OACC_CONFIG_XML_FILE NSSUBPROP_FILENAME, tTrue);
//    SetPropertyStr(
//                    OD_OACC_CONFIG_XML_FILE NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER,
//                    "XML Files (*.xml)");
//    SetPropertyStr(OD_OACC_CONFIG_XML_FILE NSSUBPROP_DESCRIPTION,
//                    "The path of the XML, defining the behaviour of the OpticalACC, has to be set here.");

    RegisterPropertyVariable(OD_OACC_ROI_SEGMENTLINE, m_propf32AccRoiSegmentLine);
    RegisterPropertyVariable(OD_OACC_ROI_Y_SEARCHTOLERANCE, m_propf32AccRoiYSearchtolerance);
    RegisterPropertyVariable(OD_OACC_ROI_HEIGHTTHRESHOLD_MIN, m_propf32AccRoiHeightThresholdMin);
    RegisterPropertyVariable(OD_OACC_ROI_HEIGHTTHRESHOLD_MAX, m_propf32AccRoiHeightThresholdMax);

    RegisterPropertyVariable(OD_OACC_DEPTTOCOLOR_X, m_propf32AccDeptToColorX);
    RegisterPropertyVariable(OD_OACC_DEPTTOCOLOR_Y, m_propf32AccDeptToColorY);
    RegisterPropertyVariable(OD_OACC_DEPTTOCOLOR_Z, m_propf32AccDeptToColorZ);


//    RegisterPropertyVariable(OD_OACC_OCCUPANCY_GRID_CELLRESOLUTION, 5);
//    SetPropertyStr(
//                    OD_OACC_OCCUPANCY_GRID_CELLRESOLUTION NSSUBPROP_VALUELISTNOEDIT,
//                    "1@Cell 2x2 (Image 160x120)|2@Cell 4x4 (Image 80x60)|3@Cell 5x5 (Image 64x48)|4@Cell 8x8 (Image 40x30)|5@Cell 10x10 (Image 32x24)|6@Cell 16x16(Image 20x15)|7@ Cell 20x20 (Image 16x12)");
//    SetPropertyStr(OD_OACC_OCCUPANCY_GRID_CELLRESOLUTION NSSUBPROP_DESCRIPTION,
//                    "Resolution of the one gridcell element (in brackets 'downsampled' resolution of actual input depth image).");

//    //Occupancy-threshold: Minimum number of data-points per gridcell that gridcell is seen as occupied. DEPENDS on Resolution, has to be chosen accordingly
//    RegisterPropertyVariable(OD_OACC_OCCUPANCY_GRID_CELLCOUNTERTHRESHOLD, 30.0);

//    //If enabled additional debug info concerning the OccupancyGrid will be printed to console (Warning: decreases performance
//    SetPropertyBool(OD_OACC_OCCUPANCY_GRID_DEBUG_LOG, tFalse);

    RegisterPropertyVariable(OD_OACC_DEBUG_INFO, m_propbAccDEBUGInfo);
    RegisterPropertyVariable(OD_OACC_DEBUG_XML_PRINT_INITIAL_TABLE, m_propbAccDEBUGXmlPrintInitialTable);
    RegisterPropertyVariable(OD_OACC_DEBUG_XML_BORDER_WARNING, m_propbAccDEBUGXmlBorderWarning);
    RegisterPropertyVariable(OD_DEBUG_TO_CONSOLE, m_propbDEBUGToConsole);
    RegisterPropertyVariable(OD_EXTENDED_DEBUG_TO_CONSOLE, m_propbExtendedDEBUGToConsole);

#ifdef DEBUG_MODE_OUTPUT_VIDEO
    RegisterPropertyVariable(OD_DEBUG_TYPE, m_propi32DEBUGType);
    SetPropertyStr(OD_DEBUG_TYPE NSSUBPROP_VALUELISTNOEDIT, "1@all | 2@oncoming traffic | 3@crosstraffic right|4@no modification|5@oncoming overturn|6@back to original lane after overturn|7@static obstacle straight|8@parking cross oncoming|9@parking cross pull-out-check");
    SetPropertyStr(OD_DEBUG_TYPE NSSUBPROP_DESCRIPTION, "Type that is used for debug output video.");
#endif
    RegisterPropertyVariable(OD_GCL_DEBUG_INFO, m_propbGclDEBUGInfo);
    RegisterPropertyVariable(OD_GCL_DEBUG_INFO_LOG, m_propbGclDEBUGInfoLog);
    RegisterPropertyVariable(OD_OACC_DEBUG_SHOW_OAAC_BINARYIMAGE_DYNAMIC, m_propbAccDEBUGShowACCBinaryImageDynamic);

    m_bDebugACCObstaclepixelcount = tFalse;

    //*************************
    //Register input pins
    //*************************
//TODO: create & register pins for video in&output
    // Register input pin for action struct
    o_actionStruct.registerPin(this, m_ReaderAction, "action_struct");
    // Register input pin for laser scanner data
    o_laserScannerData.registerPin(this, m_ReaderLaserScannerData, "laserscanner_data");
    // Register input pin for steering Angle
    o_signalValue.registerPin(this, m_ReaderSteeringAngle, "steering_angle");
    // Register input pin for target speed
    o_signalValue.registerPin(this, m_ReaderTargetSpeed, "target_speed");
    //TODO: Register input pin for video data

    //*************************
    //Register output pins
    //*************************
    // Register output pin for statemachine
    o_feedbackStruct.registerPin(this, m_WriterFeedback, "feedback");
    // Register output pin for targetSpeed
    o_signalValue.registerPin(this, m_WriterTargetSpeed, "target_speed");
    // Register output pin for video valid output
//TODO: create & register pin
    // Register output pin for Video Debug OptACC output
//TODO: create & register pin

//TODO: create & register Debug output pin
    //#ifdef DEBUG_MODE_OUTPUT_VIDEO
    //	// Register output pin for debug video output
    //      o_videoData.registerPin(this, m_WriterDebugVideo, "debug_video");
    //#endif


}


//****************************************************************************************************
//******                                CONFIGURE                                               ******
//****************************************************************************************************
// implement configure() to read ALL properties
tResult ObstacleDetection::Configure()
{
//    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}

//****************************************************************************************************
//******                                 PROCESS                                                ******
//****************************************************************************************************
// process() is always called each time a trigger occured (new data sample arrive).
// IMPORTANT: Make sure to read ALL samples of ALL readers!
// This is how to read a sample
tResult ObstacleDetection::Process(tTimeStamp tmTimeOfTrigger)
{
    if (IS_OK(ProcessLaserScannerInput()))
    {
        //LOG_SUCCESS("ProcessLaserScannerInput");
    }else if(IS_OK(ProcessActionInput()))
    {
        //LOG_SUCCESS("ProcessActionInput");
    }else if(IS_OK(ProcessSteeringAngleInput()))
    {
        //LOG_SUCCESS("ProcessSteeringAngleInput");
    }else if(IS_OK(ProcessTargetSpeedInput()))
    {
        //LOG_SUCCESS("ProcessTargetSpeedInput");
    }else if(IS_OK(ProcessVideoInput()))
    {
        //LOG_SUCCESS("ProcessVideo");
    }


//TODO:'s LaserScanner Normal & Transformed (to front axis) debug to file
    //#ifdef DEBUG_TO_FILE_POINTCLOUD_TRANSFORMED
    //		// log file for transformed point-cloud based on second frame of depth image
    //		LOG_WARNING("ObstacleDetec: debug mode: 'logging' for transformed point-cloud enabled, writing to /tmp/point_cloud_trans.dat");
    //		file_pointcloud_trans.open("/tmp/point_cloud_trans.dat", ios::out | ios::trunc);
    //		file_pointcloud_trans << "# Transformed Point cloud data x, y, z\n";
    //		//file_pointcloud_trans.close();
    //#endif
    //#ifdef DEBUG_TO_FILE_POINTCLOUD
    //                m_bPointcloud_to_file = tFalse;
    //#endif
    //#ifdef DEBUG_TO_FILE_POINTCLOUD_TRANSFORMED
    //                m_bTransformed_pointcloud_to_file = tFalse;
    //                m_bTransformed_logging_completed = tFalse;
    //#endif


    //*************************
    // assign CAMERA Properties
    //*************************
    m_f32CameraOffsetX = GetPropertyFloat(OD_CAM_POSITION_OFFSET_X);
    m_f32CameraOffsetY = GetPropertyFloat(OD_CAM_POSITION_OFFSET_Y);
    m_f32CameraOffsetZ = GetPropertyFloat(OD_CAM_POSITION_OFFSET_Z);
//TODO: Following properties needed?
//    m_f32Camera_DepthToRgbOffest = GetPropertyFloat(OD_CAM_POSITION_OFFSET_DEPTH_TO_RGB);
//    m_f32DepthImageScaleFactor = GetPropertyFloat(OD_DEPTH_IMAGE_SCALE_FACTOR);

    _depthIntrinsicData.m_f32Fx = GetPropertyFloat(OD_CAMERA_FX);
    _depthIntrinsicData.m_f32Fy = GetPropertyFloat(OD_CAMERA_FY);
    _depthIntrinsicData.m_f32Cx = GetPropertyFloat(OD_CAMERA_CX);
    _depthIntrinsicData.m_f32Cy = GetPropertyFloat(OD_CAMERA_CY);

    _colorIntrinsicData.m_f32Fx = 628.013611/2.;
    _colorIntrinsicData.m_f32Fy = 633.612183/2.;
    _colorIntrinsicData.m_f32Cx = 305.885284/2.;
    _colorIntrinsicData.m_f32Cy = 257.385101/2.;

    _depthToColor.m_f32Rotation[0] = 0.999958;
    _depthToColor.m_f32Rotation[1] = -0.000147;
    _depthToColor.m_f32Rotation[2] = 0.009170;
    _depthToColor.m_f32Rotation[3] = -0.000068;
    _depthToColor.m_f32Rotation[4] = 0.999726;
    _depthToColor.m_f32Rotation[5] = 0.023389;
    _depthToColor.m_f32Rotation[6] = -0.009170;
    _depthToColor.m_f32Rotation[7] = -0.023388;
    _depthToColor.m_f32Rotation[8] = 0.999684;

    //*************************
    // assign CAR Properties
    //*************************
//  width_of_car = GetPropertyFloat(OD_CAR_CARWIDTH);
    m_f32DistanceThresholdMax = GetPropertyFloat(OD_CAR_MAX_FIELD_OF_VIEW_X);
    m_f32DistanceThresholdMin = GetPropertyFloat(OD_CAR_MIN_FIELD_OF_VIEW_X);
    m_f32HeightThresholdMax = GetPropertyFloat(OD_CAR_MAX_FIELD_OF_VIEW_Z);
    m_f32HeightThresholdMin = GetPropertyFloat(OD_CAR_MIN_FIELD_OF_VIEW_Z);

    //************************************************
    // assign Optical ACC Properties
    //************************************************
    OACC_wheelbase = GetPropertyFloat(OD_OACC_PAR_WHEELBASE);
//  OACC_carWidth = GetPropertyFloat(OD_OACC_PAR_CARWIDTH);
    OACC_frontAxisToFrontBumper = GetPropertyFloat(OD_OACC_PAR_AXISTOBUMPER_FRONT);
    OACC_obliqueAngleFront = GetPropertyFloat(OD_OACC_PAR_OBLIQUEANGLE_FRONT);
    OACC_circSegmentDistance = GetPropertyFloat(OD_OACC_ROI_SEGMENTLINE);
    OACC_ySearchTolerance = GetPropertyFloat(OD_OACC_ROI_Y_SEARCHTOLERANCE);
    OACC_obstacleHeightThreshold_min = GetPropertyFloat(OD_OACC_ROI_HEIGHTTHRESHOLD_MIN);
    OACC_obstacleHeightThreshold_max = GetPropertyFloat(OD_OACC_ROI_HEIGHTTHRESHOLD_MAX);
//TODO: GridCell variables still needed??????????
//    OACC_GridCellResolutionIndicator = GetPropertyInt(OD_OACC_OCCUPANCY_GRID_CELLRESOLUTION);
//TODO: following lines  &  porting GetResolutionFromIndicator still needed????
//    OACC_GridCellResolution = GetResolutionFromIndicator(OACC_GridCellResolutionIndicator);
//    if(OACC_GridCellResolution == 0) {
//            RETURN_AND_LOG_ERROR_STR(ERR_INVALID_ARG,cString::Format("ObstacleDetec: Error occurred, gridcell resolution could not be loaded. Please check."));
//    }
//    OACC_GridCellOccupiedCounterThreshold = GetPropertyInt(OD_OACC_OCCUPANCY_GRID_CELLCOUNTERTHRESHOLD);
//    b_debug_occupancyGrid = GetPropertyBool(OD_OACC_OCCUPANCY_GRID_DEBUG_LOG);

    //Get path of configuration file for "parking-mode-ACC"
    m_xml_configFileOpticalACC = GetPropertyStr(OD_OACC_CONFIG_XML_FILE);
    m_xml_BorderWarningModeEnabled = GetPropertyBool(OD_OACC_DEBUG_XML_BORDER_WARNING);
    m_xml_PrintInitialtable = GetPropertyBool(OD_OACC_DEBUG_XML_PRINT_INITIAL_TABLE);

    m_curSteeringAngle.f32Value = 0.0f;
    m_f32CurTargetSpeedLimit = 10.0f;

    //************************************************
    // Initialize instanced for regions of interest
    //************************************************
    /* INTERSECTION MODE*/
    /* OnComing Traffic at intersection */
    roi_intersection_onComingTraffic.bottomleft_corner.x = OD_ROI_INTERSECTION_ONCOMING_BOTTOMLEFT_X;
    roi_intersection_onComingTraffic.bottomleft_corner.y = OD_ROI_INTERSECTION_ONCOMING_BOTTOMLEFT_Y;
    roi_intersection_onComingTraffic.m_f32Roi_y_width = OD_ROI_INTERSECTION_ONCOMING_WIDTH_Y;
    roi_intersection_onComingTraffic.m_f32Roi_x_height = OD_ROI_INTERSECTION_ONCOMING_HEIGHT_X;
    /* CrossTraffic from right hand side at intersection */
    roi_intersection_crossTrafficRight.bottomleft_corner.x = OD_ROI_INTERSECTION_CROSSTRAFFIC_BOTTOMLEFT_X;
    roi_intersection_crossTrafficRight.bottomleft_corner.y = OD_ROI_INTERSECTION_CROSSTRAFFIC_BOTTOMLEFT_Y;
    roi_intersection_crossTrafficRight.m_f32Roi_y_width = OD_ROI_INTERSECTION_CROSSTRAFFIC_WIDTH_Y;
    roi_intersection_crossTrafficRight.m_f32Roi_x_height = OD_ROI_INTERSECTION_CROSSTRAFFIC_HEIGHT_X;

    /* OVERTURN MODE */
    /* OnComing Traffic during 'on-the-road-mode' */
    roi_overtake_onComingTraffic.bottomleft_corner.x = OD_ROI_OVERTAKE_ONCOMING_BOTTOMLEFT_X;
    roi_overtake_onComingTraffic.bottomleft_corner.y = OD_ROI_OVERTAKE_ONCOMING_BOTTOMLEFT_Y;
    roi_overtake_onComingTraffic.m_f32Roi_y_width = OD_ROI_OVERTAKE_ONCOMING_WIDTH_Y;
    roi_overtake_onComingTraffic.m_f32Roi_x_height = OD_ROI_OVERTAKE_ONCOMING_HEIGHT_X;

    /* overturn: currently on oncoming lane, check 'own original' lane to go back during 'on-the-road-mode' */
    roi_overtake_originalLane.bottomleft_corner.x = OD_ROI_OVERTAKE_ORIGINALLANE_BOTTOMLEFT_X;
    roi_overtake_originalLane.bottomleft_corner.y = OD_ROI_OVERTAKE_ORIGINALLANE_BOTTOMLEFT_Y;
    roi_overtake_originalLane.m_f32Roi_y_width = OD_ROI_OVERTAKE_ORIGINALLANE_WIDTH_Y;
    roi_overtake_originalLane.m_f32Roi_x_height = OD_ROI_OVERTAKE_ORIGINALLANE_HEIGHT_X;

    /* static obstacle in front of the car, straight direction */
    roi_overtake_ownLaneStraight.bottomleft_corner.x = OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_BOTTOMLEFT_X;
    roi_overtake_ownLaneStraight.bottomleft_corner.y = OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_BOTTOMLEFT_Y;
    roi_overtake_ownLaneStraight.m_f32Roi_y_width = OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_WIDTH_Y;
    roi_overtake_ownLaneStraight.m_f32Roi_x_height = OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_HEIGHT_X;

    /* static obstacle in front left half of the car, straight direction */
    roi_overtake_ownLaneStraight_lefthalf.bottomleft_corner.x = OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_LEFTHALF_BOTTOMLEFT_X;
    roi_overtake_ownLaneStraight_lefthalf.bottomleft_corner.y = OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_LEFTHALF_BOTTOMLEFT_Y;
    roi_overtake_ownLaneStraight_lefthalf.m_f32Roi_y_width = OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_LEFTHALF_WIDTH_Y;
    roi_overtake_ownLaneStraight_lefthalf.m_f32Roi_x_height = OD_ROI_OVERTAKE_OBSTACLESTRAIGHT_LEFTHALF_HEIGHT_X;

    /* PARKING MODE */
    /* overturn: currently on oncoming lane, check 'own original' lane to go back during 'on-the-road-mode' */
    roi_parking_cross_onComing.bottomleft_corner.x = OD_ROI_PARKING_CROSS_ONCOMINGLANE_BOTTOMLEFT_X;
    roi_parking_cross_onComing.bottomleft_corner.y = OD_ROI_PARKING_CROSS_ONCOMINGLANE_BOTTOMLEFT_Y;
    roi_parking_cross_onComing.m_f32Roi_y_width = OD_ROI_PARKING_CROSS_ONCOMINGLANE_WIDTH_Y;
    roi_parking_cross_onComing.m_f32Roi_x_height = OD_ROI_PARKING_CROSS_ONCOMINGLANE_HEIGHT_X;

    /* static obstacle in front of the car, straight direction */
    roi_parking_cross_crossTraffic.bottomleft_corner.x = OD_ROI_PARKING_CROSS_CROSSTRAFFIC_RIGHT_BOTTOMLEFT_X;
    roi_parking_cross_crossTraffic.bottomleft_corner.y = OD_ROI_PARKING_CROSS_CROSSTRAFFIC_RIGHT_BOTTOMLEFT_Y;
    roi_parking_cross_crossTraffic.m_f32Roi_y_width = OD_ROI_PARKING_CROSS_CROSSTRAFFIC_RIGHT_WIDTH_Y;
    roi_parking_cross_crossTraffic.m_f32Roi_x_height = OD_ROI_PARKING_CROSS_CROSSTRAFFIC_RIGHT_HEIGHT_X;

    //************************************************************************
    // Read properties of tolerated values and necessary number of free frames
    //************************************************************************
    /* intersection mode */

    m_ui32Intrsc_freeFrameCounterLowerBound = GetPropertyInt(OD_CHECK_ROI_INTERSECTION_FREEFRAME_LOWERBOUND);

    m_ui32FreeFrameCounterLowerBound[REGIONTYPE_INTRSC_CROSSRIGHT] = GetPropertyInt(OD_CHECK_ROI_INTERSECTION_FREEFRAME_LOWERBOUND);
    m_ui32FreeFrameCounterLowerBound[REGIONTYPE_INTRSC_ONCOMING] = GetPropertyInt(OD_CHECK_ROI_INTERSECTION_FREEFRAME_LOWERBOUND);

    m_ui32OccupiedFrameCounterLowerBound[REGIONTYPE_INTRSC_CROSSRIGHT] = GetPropertyInt(OD_CHECK_ROI_INTERSECTION_OCCUPIEDCOUNTER_LOWERBOUND);
    m_ui32OccupiedFrameCounterLowerBound[REGIONTYPE_INTRSC_ONCOMING] = GetPropertyInt(OD_CHECK_ROI_INTERSECTION_OCCUPIEDCOUNTER_LOWERBOUND);

    m_ui32OccupiedCoordinateCounterUpperBound[REGIONTYPE_INTRSC_CROSSRIGHT] = GetPropertyInt(OD_CHECK_ROI_INTERSECTION_OCCUPIED_CROSSRIGHT_UPPERBOUND);
    m_ui32OccupiedCoordinateCounterUpperBound[REGIONTYPE_INTRSC_ONCOMING] = GetPropertyInt(OD_CHECK_ROI_INTERSECTION_OCCUPIED_ONCOMING_UPPERBOUND);

    m_ui32FreeFrameCounter = 0;
    m_ui32Intrsc_occupiedFrameCounterLowerBound = GetPropertyInt(OD_CHECK_ROI_INTERSECTION_OCCUPIEDCOUNTER_LOWERBOUND);

    /* overtake */

    m_ui32FreeFrameCounterLowerBound[REGIONTYPE_OVRTK_ONCOMING] = GetPropertyInt(OD_CHECK_ROI_OVERTAKE_FREEFRAME_LOWERBOUND);
    m_ui32FreeFrameCounterLowerBound[REGIONTYPE_OVRTK_OWNLANE] =  GetPropertyInt(OD_CHECK_ROI_OVERTAKE_OBSTACLESTRAIGHT_FREEFRAME_LOWERBOUND);

    m_ui32OccupiedFrameCounterLowerBound[REGIONTYPE_OVRTK_ONCOMING] =  GetPropertyInt(OD_CHECK_ROI_OVERTAKE_ONCOMINGLANE_OCCUPIEDCOUNTER_LOWERBOUND);
    m_ui32OccupiedFrameCounterLowerBound[REGIONTYPE_OVRTK_OWNLANE] = GetPropertyInt(OD_CHECK_ROI_OVERTAKE_OBSTACLESTRAIGHT_OCCUPIEDCOUNTER_LOWERBOUND);

    m_ui32OccupiedCoordinateCounterUpperBound[REGIONTYPE_OVRTK_ONCOMING] = GetPropertyInt(OD_CHECK_ROI_OVERTAKE_OCCUPIED_ONCOMING_UPPERBOUND);
    m_ui32OccupiedCoordinateCounterUpperBound[REGIONTYPE_OVRTK_OWNLANE] =  GetPropertyInt(OD_CHECK_ROI_OVERTAKE_OBSTACLESTRAIGHT_OCCUPIED_UPPERBOUND);

    m_ui32OccupiedFrameCounter = 0;

    /* parking mode */
    m_ui32FreeFrameCounterLowerBound[REGIONTYPE_CRS_PRKNG_CROSSTRAFIC] = GetPropertyInt(OD_CHECK_ROI_PARKING_CROSSTRAFFIC_FREEFRAME_LOWERBOUND);
    m_ui32FreeFrameCounterLowerBound[REGIONTYPE_CRS_PRKNG_ONCOMING] = GetPropertyInt(OD_CHECK_ROI_PARKING_ONCOMING_FREEFRAME_LOWERBOUND);

    m_ui32OccupiedFrameCounterLowerBound[REGIONTYPE_CRS_PRKNG_CROSSTRAFIC] = GetPropertyInt(OD_CHECK_ROI_PARKING_OCCUPIEDCOUNTER_LOWERBOUND);
    m_ui32OccupiedFrameCounterLowerBound[REGIONTYPE_CRS_PRKNG_ONCOMING] = GetPropertyInt(OD_CHECK_ROI_PARKING_OCCUPIEDCOUNTER_LOWERBOUND);

    m_ui32OccupiedCoordinateCounterUpperBound[REGIONTYPE_CRS_PRKNG_CROSSTRAFIC] = GetPropertyInt(OD_CHECK_ROI_PARKING_OCCUPIED_CROSSRIGHT_UPPERBOUND);
    m_ui32OccupiedCoordinateCounterUpperBound[REGIONTYPE_CRS_PRKNG_ONCOMING] = GetPropertyInt(OD_CHECK_ROI_PARKING_OCCUPIED_ONCOMING_UPPERBOUND);

    /* read property whether debug mode is required */
    m_bDebugModeEnabled = GetPropertyBool(OD_DEBUG_TO_CONSOLE);
    m_bExtendeddebugModeEnabled = GetPropertyBool(OD_EXTENDED_DEBUG_TO_CONSOLE);
#ifdef DEBUG_MODE_OUTPUT_VIDEO
    m_ui32DebugType = GetPropertyInt(OD_DEBUG_TYPE);
#endif
    m_bShowGCLDebug = GetPropertyBool(OD_GCL_DEBUG_INFO);
    m_bShowGCLDebug_extendedLog = GetPropertyBool(OD_GCL_DEBUG_INFO_LOG);

    m_bDebugACCObstaclepixelcount = GetPropertyBool(OD_OACC_DEBUG_INFO);
    m_bShowOaccBinaryImageDebug = GetPropertyBool(OD_OACC_DEBUG_SHOW_OAAC_BINARYIMAGE_DYNAMIC);

    /* get information about pitch of camera */
    m_filePitch = GetPropertyStr(OD_PITCH_STORAGE);
    m_bUsePitchFromFileh = GetPropertyBool(OD_USE_PITCH_FROM_FILE);
    m_f32PitchInDegree = GetPropertyFloat(OD_CAMERA_PITCH);
//TODO: port ReadFromFile if pitch correction is needed for fisheye
    if(m_bUsePitchFromFileh) RETURN_IF_FAILED(ReadFromFile(&m_f32PitchInDegree));
    m_f32PitchCorrection = GetPropertyFloat(OD_CAMERA_PITCH_CORRECTION);

    /* convert pitch from 'deg' to 'rad', taking correction factor into account */
    m_f32PitchInRad = ((m_f32PitchInDegree + m_f32PitchCorrection)/ 180.0f) * static_cast<tFloat32>(M_PI);
    m_f32CosPitch = cosf(m_f32PitchInRad);
    m_f32SinPitch = sinf(m_f32PitchInRad);
    if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("ObstacleDetec: Pitch in degree %f, pitch in rad %f",m_f32PitchInDegree, m_f32PitchInRad));

    m_f32SinDynRange = sinf(-m_f32PitchInRad);
    m_f32CosDynRange = cosf(-m_f32PitchInRad);
    OACC_operational_status = OPT_ACC_ENABLED;

#ifdef DEBUG_MODE_OUTPUT_VIDEO
    m_ui32DebugLogEventToggle = 0;
#endif

    //load xml files for linear interpolation
    THROW_IF_FAILED(LoadConfigurationData(m_xml_configFileOpticalACC,o_f32Acc_xml_xValues,o_f32Acc_xml_yValues));
    RETURN_NOERROR;
}

tResult ObstacleDetection::ProcessLaserScannerInput()
{
    TLaserScannerData::Data laserScannerData_tmp;
    tResult res;
    // Read from input Pins
    if(IS_FAILED(res = o_laserScannerData.readPin(m_ReaderLaserScannerData, (void *) &laserScannerData_tmp, m_pClock->GetStreamTime())))
        RETURN_ERROR(res);
    m_curScannerInput = laserScannerData_tmp;
    SetLaserScannerData(laserScannerData_tmp);
    RETURN_NOERROR;
}
tResult ObstacleDetection::ProcessActionInput()
{
    TActionStruct::Data actionCommand_tmp;
    tResult res;
    // Read from input Pins
    if(IS_FAILED(res = o_actionStruct.readPin(m_ReaderAction, (void *) &actionCommand_tmp, m_curActionStruct.ui32ArduinoTimestamp)))
        RETURN_ERROR(res);
    m_curActionStruct = actionCommand_tmp;

    if (actionCommand_tmp.bEnabled
                    && actionCommand_tmp.ui32Command == AC_OD_OPTICAL_ACC_ENABLE) {
            SetOperationalStatus(OPT_ACC_ENABLED);
    } else if (actionCommand_tmp.bEnabled
                    && actionCommand_tmp.ui32Command == AC_OD_OPTICAL_ACC_DISABLE) {
            SetOperationalStatus(OPT_ACC_DISABLED);
    } else {
            tBool runningState = GetRunningState();
            if (actionCommand_tmp.bEnabled && actionCommand_tmp.bStarted && !runningState) {
                    SetCurrActionStruct(actionCommand_tmp);
                    SetRunningState(tTrue);
            }
            /* if other actionCommand_tmp has been send, stop processing of current actionSub.command and deactivate */
            else if (!actionSub.enabled && runningState) {
                    SetRunningState(tFalse);
            }
    }

    // Call of Process Action ["process incoming action command; if enabled and started, process corresponding command"]
    ProcessAction(actionCommand_tmp);

    RETURN_NOERROR;
}
tResult ObstacleDetection::ProcessSteeringAngleInput()
{
    TSignalValue::Data steeringAngle_tmp;
    tResult res;
    // Read from input Pins
    if(IS_FAILED(res = o_signalValue.readPin(m_ReaderSteeringAngle, (void *) &steeringAngle_tmp, m_curSteeringAngle.ui32ArduinoTimestamp)))
        RETURN_ERROR(res);
    m_curSteeringAngle = steeringAngle_tmp;
    SetSteeringAngle(steeringAngle_tmp);
    RETURN_NOERROR;
}
tResult ObstacleDetection::ProcessTargetSpeedInput()
{
    TSignalValue::Data targetSpeed_tmp;
    tResult res;
    // Read from input Pins
    if(IS_FAILED(res = o_signalValue.readPin(m_ReaderTargetSpeed, (void *) &targetSpeed_tmp, m_curSpeedControllerInput.ui32ArduinoTimestamp)))
        RETURN_ERROR(res);

    targetSpeed_tmp.f32Value = -targetSpeed_tmp.f32Value; //Negating speed - Hardware counts in wrong direction
    m_curSpeedControllerInput = targetSpeed_tmp;

    /* Check whether target speed is above recommended speed limit, and adjust accordingly */
    tFloat32 m_f32Cur_targetSpeedLimit_tmp = GetTargetSpeedLimit();

    if (m_bDebugModeEnabled) {
            LOG_INFO(
                            cString::Format("ObstacleDetection: org. speed: %f",
                                            targetSpeed_tmp.f32Value));
    }

    if (targetSpeed_tmp.f32Value > m_f32Cur_targetSpeedLimit_tmp) {
            targetSpeed_tmp.f32Value = m_f32Cur_targetSpeedLimit_tmp;
    }

    RETURN_IF_FAILED(TransmitTargetSpeed(targetSpeed_tmp));

    if (m_bDebugModeEnabled) {
            LOG_INFO(
                            cString::Format("ObstacleDetection: output. speed: %f",
                                            targetSpeed_tmp.f32Value));
    }
    RETURN_NOERROR;
}
tResult ObstacleDetection::ProcessVideoInput()
{
//    TVideoStream::Data videoStream;
//    tResult res;
//    // Read from input Pins
//    if(IS_FAILED(res = o_videoStream.readPin(m_ReaderVideoPin, (void *) &videoStream, videoStreamInput.ui32ArduinoTimestamp)))
//        RETURN_ERROR(res);
//    videoStreamInput = videoStream;


    /* always activate filter and image processing since some functionalities like obstacle-detection in current driving lane and
     *  validation of recognized lines for lanefollower (done by linespecifier) is running in reflex-mode (always on) */


    //#ifdef DEBUG_MODE_OUTPUT_VIDEO
//    TActionStruct::ActionSub actionSub_sim;
//    actionSub_sim.enabled = tTrue;
//    actionSub_sim.started = tTrue;
//    switch(m_ui32DebugType) {
//            case 1: actionSub_sim.command = AC_OD_INTERSECTION_CHECK_ALL;
//            break;
//            case 2: actionSub_sim.command = AC_OD_INTERSECTION_CHECK_ONCOMING_TRAFFIC;
//            break;
//            case 3: actionSub_sim.command = AC_OD_INTERSECTION_CHECK_CROSS_TRAFFIC_RIGHT;
//            break;
//            case 4: break;
//            case 5: actionSub_sim.command = AC_OD_OVERTAKE_CHECK_ONCOMING_TRAFFIC;
//            break;
//            case 6: actionSub_sim.command = AC_OD_OVERTAKE_CHECK_OVERTAKE_ORIGINAL_LANE;
//            break;
//            case 7: actionSub_sim.command = AC_OD_OVERTAKE_CHECK_OWN_LANE_STRAIGHT;
//            break;
//            case 8: actionSub_sim.command = AC_OD_PARKING_CROSS_CHECK_ONCOMING_TRAFFIC;
//            break;
//            case 9: actionSub_sim.command = AC_OD_PARKING_CROSS_PULLOUT_CHECK_CROSS_TRAFFIC;
//            break;
//    }
//#endif

//    SetVideo(??);

    RETURN_NOERROR;
}

//TODO: do we need to port ObstacleDetection::PropertyChanged?

/*ProcessAction does process incoming action command; if enabled and started, call corresponding process */
tResult ObstacleDetection::ProcessAction(TActionStruct::Data actionStruct){
    if (actionStruct.bEnabled && actionStruct.bStarted) {

        /* check command input */
        if (actionStruct.ui32Command == AC_OD_INTERSECTION_CHECK_ALL) {
            /* call method to check both oncoming traffic and traffic from right-hand-side */
            occupancy occupancy_status = INITIALILZED;
            occupancy_status = CheckROIforObstacles(
                    roi_intersection_onComingTraffic,
                    roi_intersection_crossTrafficRight, _pointCloud);
            if (occupancy_status == FREE_SPACE) {
                RETURN_IF_FAILED(TransmitFeedbackNoObstacle());
                SetRunningState(tFalse);
                                if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 1) {
                    LOG_WARNING(
                            cString::Format(
                                    "ObstacleDetec: No obstacle detected at all (in intersection mode), free space!"));
                                        m_ui32DebugLogEventToggle = 1;
                }
            } else if (occupancy_status == ERROR
                    || occupancy_status == INITIALILZED) {
                LOG_ERROR(
                        cString::Format(
                                "ObstacleDetec: Error occurred during region-check 'INTERSECTION - CHECK_ALL'. Return-status: %d",
                                occupancy_status));
            } else if (occupancy_status == OCCUPIED_SPACE) {
                                if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 2) {
                    LOG_WARNING(
                            cString::Format(
                                    "ObstacleDetec: Oncoming or crossing obstacle detected (in intersection mode)!"));
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
                }
            }
        } else if (actionStruct.command == AC_OD_INTERSECTION_CHECK_ONCOMING_TRAFFIC) {
            /* call method to check oncoming traffic only*/
            occupancy occupancy_status = INITIALILZED;
            occupancy_status = CheckROIforObstacles(
                    roi_intersection_onComingTraffic,
                    REGIONTYPE_INTRSC_ONCOMING, _pointCloud);
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
            }

        }
        } else if (actionStruct.command
                == AC_OD_INTERSECTION_CHECK_CROSS_TRAFFIC_RIGHT) {
            /* call method to check traffic from right-hand-side only */
        occupancy occupancy_status = INITIALILZED;
        occupancy_status = CheckROIforObstacles(
                roi_intersection_crossTrafficRight,
                REGIONTYPE_INTRSC_CROSSRIGHT, _pointCloud);
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
        }
            } else if (actionStruct.command == AC_OD_OVERTAKE_CHECK_ONCOMING_TRAFFIC) {
        /* call method to check traffic on oncoming lane only */
        occupancy occupancy_status = INITIALILZED;
        occupancy_status = CheckROIforObstacles(
                roi_overtake_onComingTraffic, REGIONTYPE_OVRTK_ONCOMING,
                _pointCloud);
        if (occupancy_status == FREE_SPACE) {
            RETURN_IF_FAILED(TransmitFeedbackNoObstacle());
            SetRunningState(tFalse);
                            if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 1) {
                LOG_WARNING(
                        cString::Format(
                                "ObstacleDetec: No oncoming obstacle detected (in driving mode), free space!"));
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
                                "ObstacleDetec: Oncoming obstacle detected (in driving mode)!"));
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
        } else if (actionStruct.command
            == AC_OD_OVERTAKE_CHECK_OVERTAKE_ORIGINAL_LANE) {
            /* call method to check traffic on actual own original lane after overtake */
            occupancy occupancy_status = INITIALILZED;
            occupancy_status = CheckROIforObstacles(roi_overtake_originalLane,
                    REGIONTYPE_OVRTK_ONCOMING, _pointCloud);
            if (occupancy_status == FREE_SPACE) {
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
         } else if (actionStruct.command
            == AC_OD_OVERTAKE_CHECK_OWN_LANE_STRAIGHT) {
            /* call method to check traffic on actual own lane after overtake */
            occupancy occupancy_status = INITIALILZED;
            occupancy_status = CheckROIforObstacles(
                    roi_overtake_ownLaneStraight, REGIONTYPE_OVRTK_OWNLANE,
                    _pointCloud);
            if (occupancy_status == FREE_SPACE) {
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
                                    "ObstacleDetec: Static obstacle detected!"));
                                        m_ui32DebugLogEventToggle = 3;
                }
        }
        } else if (actionStruct.command
            == AC_OD_OVERTAKE_CHECK_OWN_LANE_STRAIGHT_LEFTHALF) {
            /* call method to check traffic on actual own lane after overtake */
            occupancy occupancy_status = INITIALILZED;
            occupancy_status = CheckROIforObstacles(
                    roi_overtake_ownLaneStraight_lefthalf,
                    REGIONTYPE_OVRTK_OWNLANE, _pointCloud);
            if (occupancy_status == FREE_SPACE) {
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
                                    "ObstacleDetec: Static obstacle detected!"));
                                        m_ui32DebugLogEventToggle = 3;
                }
        }
        } else if (actionStruct.command
            == AC_OD_PARKING_CROSS_CHECK_ONCOMING_TRAFFIC) {
            /* call method to check traffic on oncoming lane right before starting cross-parking maneuver */
            occupancy occupancy_status = INITIALILZED;
            occupancy_status = CheckROIforObstacles(roi_parking_cross_onComing,
                    REGIONTYPE_CRS_PRKNG_ONCOMING, _pointCloud);
            if (occupancy_status == FREE_SPACE) {
                RETURN_IF_FAILED(TransmitFeedbackNoObstacle());
                SetRunningState(tFalse);

                                if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 1) {
                    LOG_WARNING(
                            cString::Format(
                                    "ObstacleDetec: No oncoming obstacle detected (in parking mode, crossparking), free space!"));
                                        m_ui32DebugLogEventToggle = 1;
                }
            } else if (occupancy_status == ERROR
                    || occupancy_status == INITIALILZED) {
                LOG_ERROR(
                        cString::Format(
                                "ObstacleDetec: Error occurred during region-check 'PARKING_CROSS - ONCOMING'. Return-status: %d",
                                occupancy_status));
            } else if (occupancy_status == OCCUPIED_SPACE) {

                                if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 2) {
                    LOG_WARNING(
                            cString::Format(
                                    "ObstacleDetec: Oncoming obstacle detected (in cross-parking mode, parking)!"));
                                        m_ui32DebugLogEventToggle = 2;
                }
            } else if (occupancy_status == OCCUPIED_STATIC) {
                RETURN_IF_FAILED(TransmitFeedbackStaticObstacle());
                SetRunningState(tFalse);
                                if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 3) {
                    LOG_WARNING(
                            cString::Format(
                                    "ObstacleDetec: Static obstacle detected! (in cross-parking mode, parking)"));
                                        m_ui32DebugLogEventToggle = 3;
                }
        }
        } else if (actionStruct.command
            == AC_OD_PARKING_CROSS_PULLOUT_CHECK_CROSS_TRAFFIC) {
            /* call method to check traffic on oncoming lane right before starting cross-parking maneuver */
            occupancy occupancy_status = INITIALILZED;
            occupancy_status = CheckROIforObstacles(
                    roi_parking_cross_crossTraffic,
                    REGIONTYPE_CRS_PRKNG_CROSSTRAFIC, _pointCloud);
            if (occupancy_status == FREE_SPACE) {
                RETURN_IF_FAILED(TransmitFeedbackNoObstacle());
                SetRunningState(tFalse);

                                if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 1) {
                    LOG_WARNING(
                            cString::Format(
                                    "ObstacleDetec: No crosstraffic obstacle detected (in parking mode, crossparking), free space!"));
                                        m_ui32DebugLogEventToggle = 1;
                }
            } else if (occupancy_status == ERROR
                    || occupancy_status == INITIALILZED) {
                LOG_ERROR(
                        cString::Format(
                                "ObstacleDetec: Error occurred during region-check 'PARKING_CROSS - CROSSTRAFFIC'. Return-status: %d",
                                occupancy_status));
            } else if (occupancy_status == OCCUPIED_SPACE) {

                                if (m_bDebugModeEnabled && m_ui32DebugLogEventToggle != 2) {
                    LOG_WARNING(
                            cString::Format(
                                    "ObstacleDetec: CrossTraffic obstacle detected (in cross-parking mode, cross pullout)!"));
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
            } else {
            LOG_ERROR(
                    cString::Format(
                                                        "ObstacleDetec: Error occurred. Process Action was called with non-existing actionStruct command. Please check."));
            }
             else {
            LOG_ERROR(
            cString::Format(
                                            "ObstacleDetec: Error occurred. Process Action was called, but actionStruct command was disabled. Please check."));
        }
        RETURN_NOERROR;
}

//TODO: update ProcessOpticalAccAndGenerateValidationImage for use of LIDAR instead of 3D-Camera
/* Methods checks for Obstacles in current field of view, returns a reduced speed depending on position of recognized obstacles */
/* check the point-cloud for objects in the current driving lane */
// params: 	offset from front-of-car to wheels;
//			length of the circular segment, that means distance in front of car to check;
// 			oblique angle front;
// 			zones for points: FAR, INTERMEDIATE, NEAR in percent of line-segment?;
//			width of check-area in y-direction:  |<-- x -->|  ;
// inputs: steering angle, transformed point-cloud
// output: speed
ObstacleDetection::ProcessedCloudData ObstacleDetection::ProcessOpticalAccAndGenerateValidationImage(
        const TSignalValue::Data steeringAngle,
                const std::vector<cv::Point3f> &m_bTransformedCloud,
        const std::vector<cv::Point3f> &orgCloud) {
        cv::Mat validPixelDepthImage = Mat::ones(m_inputFormat.nHeight,
                        m_inputFormat.nWidth, CV_8UC1) * 255;
    tBool valid_corridor_size = tTrue;
    ProcessedCloudData returnStruct;
    tUInt32 debug_counter = 0;

    /* Gridcell element and 2dimensional vector for Occupancy Grid */
    GridcellElement gridCell;
    std::vector<std::vector<GridcellElement> > occupancyGrid(
                        m_inputFormat.nHeight / OACC_GridCellResolution,
            std::vector<GridcellElement>(
                                        m_inputFormat.nWidth / OACC_GridCellResolution));

//	LOG_WARNING(cString::Format("ObstacleDetection: OccupancyGrid - size of Gridmap: width %d, height %d", occupancyGrid[0].size(), occupancyGrid.size()));


    tFloat32 corrSteeringAngleAbs = ((fabsf(steeringAngle.f32_value) - OACC_obliqueAngleFront)/180.0f)* CV_PI;
    //	LOG_WARNING(cString::Format("ObstacleDetec: corrected_SteeringAngle absolute : %f",corrSteeringAngleAbs));

    /* handle case that steringAngle would be smaller than zero due to obliqueAngle */
    if(corrSteeringAngleAbs < 0.0f){
        corrSteeringAngleAbs = 0.0f;
        LOG_WARNING(cString::Format("ObstacleDetec: corrected_SteeringAngle absolute set to zero: %f",corrSteeringAngleAbs));
    }

    /* angle too small, thus radius will tend to infinity ->  handle by checking rectangular area right in front! */
    if(corrSteeringAngleAbs < 0.005f){
        //LOG_WARNING(cString::Format("ObstacleDetec: OpticACC- path with steering < 0.05 taken!"));
        /* global minimum and maximum values including the y-search-tolerance for shrinking the in this case rectangular search corridor */
        tFloat32 x_oacc_rect_min = 0.0f;
        tFloat32 x_oacc_rect_max = OACC_circSegmentDistance
                - OACC_frontAxisToFrontBumper;
        //LOG_WARNING(cString::Format("ObstacleDetec: OACC x-search-corridor -> xmin: %f, xmax: %f",x_oacc_rect_min,x_oacc_rect_max));
        if (x_oacc_rect_max <= 0.0f) {
            LOG_WARNING(
                    cString::Format(
                            "ObstacleDetec: OACC - search-corridor in x-direction too small, speed limit will be set to zero! Check search corridor properties."));
            valid_corridor_size = tFalse;
        }
        tFloat32 y_oacc_rect_min = -OACC_ySearchTolerance;
        tFloat32 y_oacc_rect_max = OACC_ySearchTolerance;
        //LOG_WARNING(cString::Format("ObstacleDetec: OACC y-search-corridor -> ymin: %f, ymax: %f",y_oacc_rect_min,y_oacc_rect_max));

        /* Iterate through all components of point cloud, check for matching data-points */
                for (tUInt32 i = 0; i < m_bTransformedCloud.size(); i++) {
            /* BEGIN: VALIDATION OUTPUT IMAGE */
            /* perform necessary splitting into parameters 'u' and  'v' needed for validation-image-creation */
                        tUInt32 u = i % m_inputFormat.nWidth; 		// columns
                        tUInt32 v = static_cast<tUInt32>(i / m_inputFormat.nWidth); 	// rows
            /* END: VALIDATION OUTPUT IMAGE */
            /* Occupancy Grid resolution scaling */
            tUInt32 a = static_cast<tUInt32>(u / OACC_GridCellResolution); // OCG columns
            tUInt32 b = static_cast<tUInt32>(v / OACC_GridCellResolution);// OCG rows
//			LOG_WARNING(cString::Format("ObstacleDetec: OccupancyGrid -rect- counter u: %d, v: %d, a: %d, b: %d",u,v,a,b));
            /* Check for proper height of obstacle, discard otherwise */
                        if (m_bTransformedCloud[i].z > OACC_obstacleHeightThreshold_min
                                        && m_bTransformedCloud[i].z
                            < OACC_obstacleHeightThreshold_max) {
                //LOG_WARNING(cString::Format("ObstacleDetec: OpticACC- pointcloud passed z-condition."));
                /* take only values in the valid rectangular search field */
                                if (m_bTransformedCloud[i].x >= x_oacc_rect_min
                                                && m_bTransformedCloud[i].x <= x_oacc_rect_max) {
                    //LOG_WARNING(cString::Format("ObstacleDetec: OpticACC- pointcloud passed x-condition."));
                                        if (m_bTransformedCloud[i].y >= y_oacc_rect_min
                                                        && m_bTransformedCloud[i].y <= y_oacc_rect_max) {
                        //LOG_WARNING(cString::Format("ObstacleDetec: OpticACC- pointcloud passed y-condition."));
                                                occupancyGrid[b][a].m_cvCoordinateSum.x +=
                                                                m_bTransformedCloud[i].x;
                                                occupancyGrid[b][a].m_cvCoordinateSum.y +=
                                                                m_bTransformedCloud[i].y;
                                                occupancyGrid[b][a].m_ui32ElemCounter++;
                        debug_counter++;

                    }
                }
            }
            /* BEGIN: VALIDATION OUTPUT IMAGE */
            /* check if height in the currently regarded pixel is too far above the ground */
                        if (m_bTransformedCloud[i].z > m_f32HeightThresholdValidmask) {
                /* reduce invalid measurement points (received with x-distance smaller/equal to zero) */
                                if (m_bTransformedCloud[i].x > 0) {
                    Point3f colorPoint = transformDepthToColor(orgCloud[i],
                            _depthToColor);
                    Point2f colorPixel = projectPointToPixel(colorPoint,
                            _colorIntrinsicData);
                    if (colorPixel.x >= 0.
                            && colorPixel.x < validPixelDepthImage.cols
                            && colorPixel.y >= 0.
                            && colorPixel.y < validPixelDepthImage.rows)
                        validPixelDepthImage.at<tUInt8>(colorPixel.y,
                                colorPixel.x) = 0; // black

                }
            }
            /* END: VALIDATION OUTPUT IMAGE */
        }
                if (debug_counter > 0 && m_bDebugACCObstaclepixelcount) {
            LOG_WARNING(
                    cString::Format(
                            "ObstacleDetec: OpticACC- found Obstacle-pixels in rect-area, counter: %d!",
                            debug_counter));
        }
    } else {
        /* radius between momentum-pole and middle of front-axis */
        tFloat32 segmentRadius = OACC_wheelbase / sinf(corrSteeringAngleAbs);
        tFloat32 segmentAngle = OACC_circSegmentDistance / segmentRadius;
        /* if segmentAngle + steeringAngle would be greater than pi/2, no proper checking would be possible (atm) */
        if ((segmentAngle + corrSteeringAngleAbs) > CV_PI / 2) {
            segmentAngle = CV_PI / 2 - corrSteeringAngleAbs;
        }

        /* minimum and maximum values of the reference line-segment */
        tFloat32 x_oacc_ref_min = 0.0f;
        tFloat32 x_oacc_ref_max = segmentRadius * sinf(corrSteeringAngleAbs + segmentAngle) - OACC_wheelbase - OACC_frontAxisToFrontBumper;
        tFloat32 y_oacc_ref_min = 0.0f;
        tFloat32 y_oacc_ref_max = (OACC_wheelbase / tanf(corrSteeringAngleAbs)) - (segmentRadius * cosf(corrSteeringAngleAbs + segmentAngle));
//		LOG_WARNING(cString::Format("ObstacleDetec: OACC x-search-corridor_REF -> xmin: %f, xmax: %f",x_oacc_ref_min,x_oacc_ref_max));
//		LOG_WARNING(cString::Format("ObstacleDetec: OACC y-search-corridor_REF -> ymin: %f, ymax: %f",y_oacc_ref_min,y_oacc_ref_max));

        /* global minimum and maximum values including the y-search-tolerance for shrinking the general rectangular search corridor */
        tFloat32 x_oacc_ref_tol_min = x_oacc_ref_min;
        tFloat32 x_oacc_ref_tol_max = x_oacc_ref_max;
        tFloat32 y_oacc_ref_tol_min = 0.0f;
        tFloat32 y_oacc_ref_tol_max = 0.0f;
        /* In case that steering to left hand side, that means in positive y-direction */
        if ((steeringAngle.f32_value) < 0) {
            y_oacc_ref_tol_min = y_oacc_ref_min - OACC_ySearchTolerance;
            y_oacc_ref_tol_max = y_oacc_ref_max + OACC_ySearchTolerance;
        } else { //steering to right hand side
            y_oacc_ref_tol_min = -y_oacc_ref_max - OACC_ySearchTolerance;
            y_oacc_ref_tol_max = -y_oacc_ref_min + OACC_ySearchTolerance;
        }
//		LOG_WARNING(cString::Format("ObstacleDetec: OACC y-search-corridor_REAL -> ymin: %f, ymax: %f, curr.steeringAngle: %f",y_oacc_ref_tol_min,y_oacc_ref_tol_max,steeringAngle.f32_value));

        /* Relation between currently viewed/checked x_oacc and y_oacc */
        tFloat32 y_curOacc_ref = 0.0f;
        /* Iterate through all components of point cloud, check for matching data-points */
                for (tUInt32 i = 0; i < m_bTransformedCloud.size(); i++) {
            /* BEGIN: VALIDATION OUTPUT IMAGE */
            /* perform necessary splitting into parameters 'u' and  'v' needed for validation-image-creation */
                        tUInt32 u = i % m_inputFormat.nWidth; // columns
                        tUInt32 v = i / m_inputFormat.nWidth; // rows
            /* END: VALIDATION OUTPUT IMAGE */
            /* Occupancy Grid resolution scaling */
            tUInt32 a = static_cast<tUInt32>(u / OACC_GridCellResolution); // OCG columns
            tUInt32 b = static_cast<tUInt32>(v / OACC_GridCellResolution);// OCG rows
//			LOG_WARNING(cString::Format("ObstacleDetec: OccupancyGrid -dyn- counter u: %d, v: %d, a: %d, b: %d",u,v,a,b));
            /* Check for proper height of obstacle, discard otherwise */
                        if (m_bTransformedCloud[i].z > OACC_obstacleHeightThreshold_min
                                        && m_bTransformedCloud[i].z
                            < OACC_obstacleHeightThreshold_max) {
                /* take only values in the valid rectangular search field */
                                if (m_bTransformedCloud[i].x >= x_oacc_ref_tol_min
                                                && m_bTransformedCloud[i].x <= x_oacc_ref_tol_max) {
                                        if (m_bTransformedCloud[i].y >= y_oacc_ref_tol_min
                                                        && m_bTransformedCloud[i].y <= y_oacc_ref_tol_max) {
                        /* check only values that fit the specific search corridor, given by the circular relation between x and y ; K-Y = y~ */
                        /* circle-formula returns only values for the case of steering left, that means (steeringAngle.f32_value ) < 0 */
                                                y_curOacc_ref = (OACC_wheelbase / tanf(corrSteeringAngleAbs)) - sqrt((segmentRadius * segmentRadius) - ((m_bTransformedCloud[i].x + OACC_wheelbase)*(m_bTransformedCloud[i].x + OACC_wheelbase)));
                        /* case of steering right: */
                        if ((steeringAngle.f32_value) > 0) {
                            y_curOacc_ref = -y_curOacc_ref;
                        }
                        /* actual check is performed */
                                                if ((m_bTransformedCloud[i].y
                                >= (y_curOacc_ref - OACC_ySearchTolerance))
                                                                && (m_bTransformedCloud[i].y
                                        <= y_curOacc_ref + OACC_ySearchTolerance)) {
                            /* At this point, currently checked data-point really is in the current sector of interest */
                                                        occupancyGrid[b][a].m_cvCoordinateSum.x +=
                                                                        m_bTransformedCloud[i].x;
                                                        occupancyGrid[b][a].m_cvCoordinateSum.y +=
                                                                        m_bTransformedCloud[i].y;
                                                        occupancyGrid[b][a].m_ui32ElemCounter++;
                            debug_counter++;
                        }
                    }
                }
            }
            /* BEGIN: VALIDATION OUTPUT IMAGE */
            /* check if height in the currently regarded pixel is too far above the ground */
                        if (m_bTransformedCloud[i].z > m_f32HeightThresholdValidmask) {
                /* reduce invalid measurement points (received with x-distance smaller/equal to zero) */
                                if (m_bTransformedCloud[i].x > 0) {
                    Point3f colorPoint = transformDepthToColor(orgCloud[i],
                            _depthToColor);
                    Point2f colorPixel = projectPointToPixel(colorPoint,
                            _colorIntrinsicData);

                    if (colorPixel.x >= 0.
                            && colorPixel.x < validPixelDepthImage.cols
                            && colorPixel.y >= 0.
                            && colorPixel.y < validPixelDepthImage.rows)
                        validPixelDepthImage.at<tUInt8>(colorPixel.y,
                                colorPixel.x) = 0; // black
                }
            }
            /* END: VALIDATION OUTPUT IMAGE */
        }

                if (debug_counter > 0 && m_bDebugACCObstaclepixelcount) {
            LOG_WARNING(
                    cString::Format(
                            "ObstacleDetection: OACC - found Obstacle-pixels in dynamic area, counter: %d",
                            debug_counter));
        }
    }

    /* Parse created occupancy grid for occupied cell elements */
    tFloat32 min_distanceToGridCell = 10.0f; // set to high value
    tFloat32 tmp_distanceToGridCell = 0.0f;
    cv::Point2f cellBalancePoint;
    cv::Point2i debug_log_closestCellCoords;
        for (tUInt32 row = 0; row < m_inputFormat.nHeight / OACC_GridCellResolution;
            row++) {
        for (tUInt32 col = 0;
                                col < m_inputFormat.nWidth / OACC_GridCellResolution; col++) {
                        if (occupancyGrid[row][col].m_ui32ElemCounter
                    >= OACC_GridCellOccupiedCounterThreshold) {
                cellBalancePoint.x =
                                                occupancyGrid[row][col].m_cvCoordinateSum.x
                                                                / static_cast<tFloat32>(occupancyGrid[row][col].m_ui32ElemCounter);
                cellBalancePoint.y =
                                                occupancyGrid[row][col].m_cvCoordinateSum.y
                                                                / static_cast<tFloat32>(occupancyGrid[row][col].m_ui32ElemCounter);
                tmp_distanceToGridCell = cv::norm(cellBalancePoint);
                if (tmp_distanceToGridCell < min_distanceToGridCell) {
                    min_distanceToGridCell = tmp_distanceToGridCell;
                    debug_log_closestCellCoords.x = row;
                    debug_log_closestCellCoords.y = col;
                }
            }
        }
    }
        if (b_debug_occupancyGrid) {
        LOG_WARNING(
                cString::Format(
                        "ObstacleDetec: OccupancyGrid - minimum distance to occupied cell: %f; cell row: %d, col: %d",
                        min_distanceToGridCell, debug_log_closestCellCoords.x,
                        debug_log_closestCellCoords.y));
    }

    /* If search corridor is valid, use minimum distance to read the speedlimit from xml-file ! */
    if (valid_corridor_size) {
        tFloat32 tmp_speedlimit = GetLinearInterpolatedValue(
                                min_distanceToGridCell, o_f32Acc_xml_xValues, o_f32Acc_xml_yValues);
                if (b_debug_occupancyGrid) {
            LOG_WARNING(
                    cString::Format(
                            "ObstacleDetec: OccupancyGrid - calculated speed limit: %f for distance : %f",
                            tmp_speedlimit, min_distanceToGridCell));
        }
        if (tmp_speedlimit < 0.0f) {
            LOG_ERROR(
                    cString::Format(
                            "ObstacleDetec: Invalid speed limit, error occurred in calculating interpolated value. Limit set to zero!"));
            tmp_speedlimit = 0.0f;
        }
                returnStruct.m_f32TargetSpeedLimit = tmp_speedlimit;
    } else {
                returnStruct.m_f32TargetSpeedLimit = 0.0f;
    }

        cv::resize(validPixelDepthImage, returnStruct.m_cvValidPixelDepthImage, Size(1440,1080));
    int borderWith = (1920 - 1440) / 2;

    //add white border to create 16:9 image format
        cv::copyMakeBorder(returnStruct.m_cvValidPixelDepthImage,
                        returnStruct.m_cvValidPixelDepthImage, 0, 0, borderWith, borderWith, cv::BORDER_CONSTANT, 255);
    return returnStruct;
}

//TODO: port ProcessVideo for reading/writing Video to ADTF3;
//TODO: update ProcessVideo needed from 3D-Camera to 2D-Camera + Lidar
/* Performs/Calls all necessary steps to read depth image from input and transform to output point cloud */
tResult ObstacleDetection::ProcessVideo() {

    /* Get current values associated with this video frame */
    TSignalValue::Data steeringAngle = GetSteeringAngle();

    /* Create new Mat matrix */
        Mat depthImage(m_inputFormat.nHeight, m_inputFormat.nWidth, CV_16UC1);
//TODO: change GetInputDepthImage to a function that gets LidarData
    RETURN_IF_FAILED(GetInputDepthImage(depthImage, sample));
    /* Original data is now referenced by 'depthImage' of type Mat, but is still pointing
     *  on the original input data! */

    std::vector<cv::Point3f> orgPointCloud;
    /* generate Point Cloud from Depth Image */
    //TODO: change generatePointCloudFromDepthImage, equivalent needed for LaserScanner or deprecated?
    RETURN_IF_FAILED(
            generatePointCloudFromDepthImage(depthImage, orgPointCloud));
//TODO: change transformPointCloudToCarPerspective to fit LaserScanner Data or deprecated?
        /* transform Point Cloud : m_f32Translation and m_f32Rotation */
    RETURN_IF_FAILED(transformPointCloudToCarPerspective(orgPointCloud, _pointCloud));

#ifdef DEBUG_MODE_OUTPUT_VIDEO
    /* Create output header for showing debug media stream on video output */
        Mat out(m_inputFormat.nHeight, m_inputFormat.nWidth, CV_16UC1);
//TODO: change generateDebugDepthFromTransformedCloud if needed for Debugging!
    out = generateDebugDepthFromTransformedCloud(_pointCloud);
    //generate output
    cObjectPtr<IMediaSample> depthSample;
    if (IS_OK(AllocMediaSample(&depthSample))) {
                tBitmapFormat outputFormat = m_inputFormat;
        outputFormat.nWidth = out.cols;
        outputFormat.nHeight = out.rows;
        outputFormat.nBitsPerPixel = out.channels() * 16;
        outputFormat.nBytesPerLine = out.cols * 2 * out.channels(); // * 2: 16 bit
        outputFormat.nSize = outputFormat.nBytesPerLine * out.rows;

        if (out.type() == CV_16UC1) {
            outputFormat.nPixelFormat = cImage::PF_GREYSCALE_16;
        }

        depthSample->Update(sample->GetTime(), out.data, tInt32(outputFormat.nSize), 0);

                m_WriterVideoPin.SetFormat(&outputFormat, NULL);
                m_WriterVideoPin.Transmit(depthSample);
    }
#endif

    /* Check for obstacles in current driving lane and additionally generate the validation image for line-checking */
    ProcessedCloudData processedCloudData =
            ProcessOpticalAccAndGenerateValidationImage(steeringAngle,
                    _pointCloud, orgPointCloud);

    /* only change input speed */
    if (GetOperationalStatus() == OPT_ACC_ENABLED) {
                SetTargetSpeedLimit(processedCloudData.m_f32TargetSpeedLimit);
    } else if (GetOperationalStatus() == OPT_ACC_DISABLED) {
        SetTargetSpeedLimit(10.0f);
    } else {
        LOG_ERROR(
                cString::Format(
                        "ObstacleDetec: Error occurred, could not read operational status! Please check."));
    }

    /* Create GCL output to show which region is checked (to be overlaid the RGB image in video display)*/
        if (m_WriterGCLOutput.IsConnected() && m_bShowGCLDebug) {
        RETURN_IF_FAILED(CreateAndTransmitGCL_dynamicDrive(steeringAngle));
    }

    /* Create output header for debug opticalACC video output if output is connected and property is set */
        if (m_WriterDebugOptACCPin.IsConnected() && m_bShowOaccBinaryImageDebug) {
        Mat out_OptACC_Debug = generateDebugOutputOpticACC(steeringAngle,
                _pointCloud);

        //generate output
        cObjectPtr<IMediaSample> optACCSample;
        if (IS_OK(AllocMediaSample(&optACCSample))) {
                        tBitmapFormat outputFormat_optACC = m_inputFormat;
            outputFormat_optACC.nWidth = out_OptACC_Debug.cols;
            outputFormat_optACC.nHeight = out_OptACC_Debug.rows;
            outputFormat_optACC.nBitsPerPixel = out_OptACC_Debug.channels()
                    * 16;
            outputFormat_optACC.nBytesPerLine = out_OptACC_Debug.cols * 2
                    * out_OptACC_Debug.channels(); // * 2: 16 bit
            outputFormat_optACC.nSize = outputFormat_optACC.nBytesPerLine
                    * out_OptACC_Debug.rows;

            if (out_OptACC_Debug.type() == CV_16UC1) {
                outputFormat_optACC.nPixelFormat = cImage::PF_GREYSCALE_16;
            }

            optACCSample->Update(sample->GetTime(), out_OptACC_Debug.data,
                    tInt32(outputFormat_optACC.nSize), 0);

                        m_WriterDebugOptACCPin.SetFormat(&outputFormat_optACC, NULL);
                        m_WriterDebugOptACCPin.Transmit(optACCSample);
        }
    }

    /* Create output header for validation video output */
    //Mat out_valid_image = generateValidPixelDepthFromTransformedCloud(m_cloud);
        Mat& outValidImage = processedCloudData.m_cvValidPixelDepthImage;
    //generate output
    cObjectPtr<IMediaSample> depthValidSample;
    if (IS_OK(AllocMediaSample(&depthValidSample))) {
                tBitmapFormat outputFormat_valid = m_inputFormat;
        outputFormat_valid.nWidth = outValidImage.cols;
        outputFormat_valid.nHeight = outValidImage.rows;
        outputFormat_valid.nBitsPerPixel = outValidImage.channels() * 8;
        outputFormat_valid.nBytesPerLine = outValidImage.cols * 1
                * outValidImage.channels(); // * 1: 8 bit
        outputFormat_valid.nSize = outputFormat_valid.nBytesPerLine
                * outValidImage.rows;

        if (outValidImage.type() == CV_8UC1) {
            outputFormat_valid.nPixelFormat = cImage::PF_GREYSCALE_8;
        }

        depthValidSample->Update(sample->GetTime(), outValidImage.data,
                tInt32(outputFormat_valid.nSize), 0);

                m_WriterVideoValidPin.SetFormat(&outputFormat_valid, NULL);
                m_WriterVideoValidPin.Transmit(depthValidSample);
    }

    RETURN_NOERROR;
}

//ÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜ
//ÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜ
//ÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜ
//ÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜÜ
// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------

// TODO: porting transformDepthToColor needed????
// TODO: porting projectPointToPixel needed????

// TODO: Update implementation and info text of overloaded function "CheckROIforObstacles" both and "CheckROI" (laserScannerData instead of point cloud..)

/*  check ROI passed to method for obstacles, just using counter in first version.
 *  -> functionality: check specified region of interest for objects, that means for points in point cloud that are in specified x-y-rectangle,
 *  and have a height greater than threshold-height; -> for each coordinate that fulfills this condition, increase region counter (one for each region);
 *  check counter-value at the end, and if limit is exceeded, then an obstacle is assumed to be there, so no feedback is sent (or static feedback)! (free-frame-counter is set to zero again);
 *  -> if the limit is NOT exceeded, that means there was supposedly no obstacle, then the "free-frame-counter" is increased by one;
 *  if free-frame-counter is greater than a certain number, the feedback "no obstacle" is sent to the SCM */
ObstacleDetection::occupancy ObstacleDetection::CheckROIforObstacles(const roi regionToCheck, regionType type, const std::vector<cv::Point3f> &cloud)
{

//    tUInt32 occupiedCoordinateCounter = CheckROI(regionToCheck, cloud);

//    if(m_bExtendeddebugModeEnabled) LOG_WARNING(cString::Format("ObstacleDetec: Type: %d, Occupied Counter: %d", type, occupiedCoordinateCounter));

//    /* recheck if correct type and if counter is below upper limit -> increase number of free frames in sequence */
//    if(type < 6 && occupiedCoordinateCounter <= m_ui32OccupiedCoordinateCounterUpperBound[type])
//    {
//        m_ui32FreeFrameCounter++;
//        m_ui32OccupiedFrameCounter = 0;
//    }
//    else
//    {
//        /* if frame is not free, reset free frame counter to zero, start new counting-sequence */
//        m_ui32FreeFrameCounter = 0;
//        /* in this case, the frame is occupied, so start counting how many frames in a row to see if it is a static obstacle */
//        m_ui32OccupiedFrameCounter++;

//        if(m_ui32OccupiedFrameCounter >= m_ui32OccupiedFrameCounterLowerBound[type])
//        {
//            m_ui32OccupiedFrameCounter = 0;
//            return OCCUPIED_STATIC;
//        }

//    return OCCUPIED_SPACE;
//    }

//    /* check if lower bound of frames without obstacle is reached */
//    if(m_ui32FreeFrameCounter >= m_ui32FreeFrameCounterLowerBound[type])
//    {
//        /* region to test is free, no obstacle detected for several frames; reset all counters, return FREE_SPACE */
//                    m_ui32FreeFrameCounter = 0;
//        return FREE_SPACE;
//    }
//    else
//    { // otherwise
        return STATE_PENDING;
//    }
}

ObstacleDetection::occupancy ObstacleDetection::CheckROIforObstacles(roi regionToCheck_OnComing, roi regionToCheck_CrossRight, const std::vector<cv::Point3f> &cloud) {

        /* necessary variables for region-checking */
        /* counters indicating occupied pixel */
        tUInt32 occupiedCoordinateCounter_onComing = CheckROI(regionToCheck_OnComing, cloud);
        tUInt32 occupiedCoordinateCounter_crossRight = CheckROI(regionToCheck_CrossRight, cloud);

                if(m_bExtendeddebugModeEnabled) LOG_WARNING(cString::Format("ObstacleDetec: Occupied Counter oncoming: %d, cross right: %d", occupiedCoordinateCounter_onComing, occupiedCoordinateCounter_crossRight));

        /* check if counters are !both! below upper limit -> increase number of free frames in sequence */
                if((occupiedCoordinateCounter_crossRight <= m_ui32OccupiedCoordinateCounterUpperBound[REGIONTYPE_INTRSC_CROSSRIGHT])
                                && (occupiedCoordinateCounter_onComing <= m_ui32OccupiedCoordinateCounterUpperBound[REGIONTYPE_INTRSC_ONCOMING])) {
                        m_ui32FreeFrameCounter++;
                        m_ui32OccupiedFrameCounter = 0;
        }
        else{
            /* if frame is not free, reset counter to zero, start new counting-sequence */
                        m_ui32FreeFrameCounter = 0;
                        m_ui32OccupiedFrameCounter++;
                        if(m_ui32OccupiedFrameCounter >= m_ui32Intrsc_occupiedFrameCounterLowerBound){
                                m_ui32OccupiedFrameCounter = 0;
                return OCCUPIED_STATIC;
            }
            return OCCUPIED_SPACE;
        }

        /* check if lower bound of frames without obstacle is reached */
                if(m_ui32FreeFrameCounter >= m_ui32Intrsc_freeFrameCounterLowerBound){
            /* region to test is free, no obstacle detected for several frames; reset all counters, return FREE_SPACE */
                        m_ui32FreeFrameCounter = 0;
            return FREE_SPACE;
        }
        else{ // otherwise
            return STATE_PENDING;
        }
    }

tUInt32 ObstacleDetection::CheckROI(roi regionToCheck, const std::vector<cv::Point3f> &cloud) {

    tUInt32 counter = 0;

//    /* Iterate through all components of point cloud, check set properties */
//    for(tUInt32 i = 0 ; i < cloud.size() ; i++){
//         /* first take only the valid field of view for the camera to check rest */
//         if(cloud[i].x > m_f32DistanceThresholdMin  && cloud[i].x < m_f32DistanceThresholdMax){
//                 if(cloud[i].z > m_f32HeightThresholdMin && cloud[i].z < m_f32HeightThresholdMax){

//                         /* check inside the propagated region for oncoming traffic */

//                         if(checkROI(cloud[i], regionToCheck)){
//                                 counter++;
//                         }
//                 }
//         }
//    }
    return counter;
}

//********************************************
//******    Getter- &Setterfunctions    ******
//******    (with synchronized access)  ******
//********************************************


TActionStruct::Data ObstacleDetection::GetCurractionStruct() {
        boost::lock_guard<boost::mutex> lock(criticalSection_actionStructAccess);
        return m_actionStruct;
}

tResult ObstacleDetection::SetCurrActionStruct(
                TActionStruct newActionStruct) {
        boost::lock_guard<boost::mutex> lock(criticalSection_ActionSubAccess);
        m_actionStruct = newActionStruct;
        RETURN_NOERROR;
}

ObstacleDetection::operationalStatus ObstacleDetection::GetOperationalStatus() {
        boost::lock_guard<boost::mutex> lock(criticalSection_OperationalStatusAccess);
        return OACC_operational_status;
}

tResult ObstacleDetection::SetOperationalStatus(operationalStatus opStatus) {
        boost::lock_guard<boost::mutex> lock(criticalSection_OperationalStatusAccess);
        OACC_operational_status = opStatus;
        RETURN_NOERROR;
}

tBool ObstacleDetection::GetRunningState() {
        boost::lock_guard<boost::mutex> lock(criticalSection_RunningStateAccess);
        return m_bRunningState;
}

tResult ObstacleDetection::SetRunningState(tBool state) {
        boost::lock_guard<boost::mutex> lock(criticalSection_RunningStateAccess);
        m_bRunningState = state;
    RETURN_NOERROR;
}

TSignalValue::Data ObstacleDetection::GetSteeringAngle() {
        boost::lock_guard<boost::mutex> lock(criticalSection_SteeringAngleAccess);
        return m_curSteeringAngle;
}

tResult ObstacleDetection::SetSteeringAngle(TSignalValue::Data steeringAngle) {
        boost::lock_guard<boost::mutex> lock(criticalSection_SteeringAngleAccess);

        //Hack 2017
        steeringAngle.f32_value /= 2;
        m_curSteeringAngle = steeringAngle;
        //LOG_WARNING(cString::Format("ObstacleDetec: OpticACC - steering angle set to: %f.",curSteeringAngle.f32_value));
        RETURN_NOERROR;
}

tFloat32 ObstacleDetection::GetTargetSpeedLimit() {
        boost::lock_guard<boost::mutex> lock(criticalSection_TargetSpeedLimitAccess);
        return m_f32CurTargetSpeedLimit;
}

tResult ObstacleDetection::SetTargetSpeedLimit(tFloat32 m_f32TargetSpeedLimit) {
        boost::lock_guard<boost::mutex> lock(criticalSection_TargetSpeedLimitAccess);
        m_f32CurTargetSpeedLimit = m_f32TargetSpeedLimit;
    RETURN_NOERROR;
}

tResult ObstacleDetection::TransmitTargetSpeed(
                TSignalValue::Data newTargetSpeed_tmp) {
        boost::lock_guard<boost::mutex> lock(criticalSection_TransmitSpeed);
        RETURN_IF_FAILED(o_signalValue.writePin(m_WriterTargetSpeed, (void *) &newTargetSpeed_tmp, m_pClock->GetStreamTime()));
        RETURN_NOERROR;
}

/* Transmit feedback struct with status no_obstacle to SCM */
tResult ObstacleDetection::TransmitFeedbackNoObstacle() {
    boost::lock_guard<boost::mutex> lock(criticalSection_TransmitFeedback);
    TFeedbackStruct::Data feedback_tmp;
    feedback_tmp.ui32FilterId = F_OBSTACLE_DETECTION;
    feedback_tmp.ui32FeedbackStatus = FB_OD_NO_OBSTACLE;
    RETURN_IF_FAILED(
                        o_feedbackStruct.writePin(m_WriterFeedback, (void *) &feedback_tmp,
                    m_pClock->GetStreamTime()));
    RETURN_NOERROR;
}

/* Transmit feedback struct with status 'Obstacle' to SCM */
tResult ObstacleDetection::TransmitFeedbackStaticObstacle() {
    boost::lock_guard<boost::mutex> lock(criticalSection_TransmitFeedback);
    TFeedbackStruct::Data feedback_tmp;
    feedback.ui32_filterID = F_OBSTACLE_DETECTION;
    feedback.ui32_status = FB_OD_STATIC_OBSTACLE;
    RETURN_IF_FAILED(
                        o_feedbackStruct.writePin(m_WriterFeedback, (void *) &feedback_tmp,
                    m_pClock->GetStreamTime()));
    RETURN_NOERROR;
}

tResult ObstacleDetection::SetLaserScannerData(TLaserScannerData::Data laserScannerData) {
        boost::lock_guard<boost::mutex> lock(criticalSection_LaserScannerDataAccess);

        //Hack 2017
        laserScannerData.f32_value /= 2;
        m_curScannerInput = laserScannerData;
        //LOG_WARNING(cString::Format("ObstacleDetec: OpticACC - steering angle set to: %f.",curSteeringAngle.f32_value));
        RETURN_NOERROR;
}

//TODO: Update CreateAndTransmitGCL_dynamicDrive for use with new data format (point cloud for 3D-Data deprecated)
/* Method printing the dynamic calculated driving lane - check area */
tResult ObstacleDetection::CreateAndTransmitGCL_dynamicDrive(
            const TSignalValue::Data steeringAngle) {
//        // just draw gcl if the pin is connected and debug mode is enabled
//            if (!m_WriterGCLOutput.IsConnected() || !m_bShowGCLDebug) {
//            RETURN_NOERROR;
//        }
//        std::vector<cv::Point3f> referenceLine;
//        cv::Point3f tmpPointLeft;
//        cv::Point3f tmpPointRight;
//        tFloat32 corrSteeringAngleAbs = ((fabsf(steeringAngle.f32_value) - OACC_obliqueAngleFront)/180.0f)* CV_PI;
//        tFloat32 interval = 0.0f;
//        /* handle case that steringAngle would be smaller than zero due to obliqueAngle */
//        if(corrSteeringAngleAbs < 0.0f){
//            corrSteeringAngleAbs = 0.0f;
//        }
//        /* angle too small, thus radius will tend to infinity ->  handle by checking rectangular area right in front! */
//        if(corrSteeringAngleAbs < 0.005f){
//            /* global minimum and maximum values including the y-search-tolerance for shrinking the in this case rectangular search corridor */
//            tFloat32 x_oacc_rect_min = 0.0f;
//            tFloat32 x_oacc_rect_max = OACC_circSegmentDistance
//                    - OACC_frontAxisToFrontBumper;
//            if (x_oacc_rect_max < 0.0f) {
//                LOG_WARNING(
//                        cString::Format(
//                                "ObstacleDetec: OACC - search-corridor in x-direction too small, received input target speed will be returned unmodified!"));
//            }

//            /* Creating 50 x-values in car-coordinate system as supporting points*/
//            interval = (x_oacc_rect_max - x_oacc_rect_min) / 50.0f;
//            for (tUInt32 i = 0; i < 50; i++) {
//                tmpPointLeft.x = static_cast<tFloat32>(i) * interval;
//                tmpPointLeft.y = 0.0f - OACC_ySearchTolerance;
//                tmpPointLeft.z = 0.0f;
//                tmpPointRight.x = static_cast<tFloat32>(i) * interval;
//                tmpPointRight.y = 0.0f + OACC_ySearchTolerance;
//                tmpPointRight.z = 0.0f;
//                //LOG_WARNING(cString::Format("ObstacleDetec: calculated points straight-mode - x: %f, y: %f",tmpPoint.x,tmpPoint.y));
//                referenceLine.push_back(tmpPointLeft);
//                referenceLine.push_back(tmpPointRight);
//            }

//        } else {
//            /* radius between momentum-pole and middle of front-axis */
//            tFloat32 segmentRadius = OACC_wheelbase / sinf(corrSteeringAngleAbs);
//            tFloat32 segmentAngle = OACC_circSegmentDistance / segmentRadius;
//            /* if segmentAngle + steeringAngle would be greater than pi/2, no proper checking would be possible (atm) */
//            if((segmentAngle + corrSteeringAngleAbs) > CV_PI/2){
//                segmentAngle = CV_PI/2 - corrSteeringAngleAbs;
//            }

//            /* minimum and maximum values of the reference line-segment */
//            tFloat32 x_oacc_ref_min = 0.0f;
//            tFloat32 x_oacc_ref_max = segmentRadius * sinf(corrSteeringAngleAbs + segmentAngle) - OACC_wheelbase - OACC_frontAxisToFrontBumper;

//            /* Creating 50 x-values in car-coordinate system as supporting points*/
//            interval = (x_oacc_ref_max - x_oacc_ref_min) / 50.0f;
//            for (tUInt32 i = 0; i < 50; i++) {
//                tmpPointLeft.x = static_cast<tFloat32>(i) * interval;
//                tmpPointRight.x = static_cast<tFloat32>(i) * interval;
//                /* circle-formula returns only values for the case of steering left, that means (steeringAngle.f32_value ) < 0 */
//                tmpPointLeft.y = (OACC_wheelbase / tanf(corrSteeringAngleAbs)) - sqrt((segmentRadius * segmentRadius) - ((tmpPointLeft.x + OACC_wheelbase)*(tmpPointLeft.x + OACC_wheelbase))) - OACC_ySearchTolerance;
//                tmpPointRight.y = (OACC_wheelbase / tanf(corrSteeringAngleAbs)) - sqrt((segmentRadius * segmentRadius) - ((tmpPointRight.x + OACC_wheelbase)*(tmpPointRight.x + OACC_wheelbase))) + OACC_ySearchTolerance;
//                if((steeringAngle.f32_value ) > 0){
//                    tmpPointLeft.y = - tmpPointLeft.y;
//                    tmpPointRight.y = -tmpPointRight.y;
//                }
//                //LOG_WARNING(cString::Format("ObstacleDetec: calculated points dyn-mode - x: %f, y: %f",tmpPoint.x,tmpPoint.y));
//                tmpPointLeft.z = 0.0f;
//                tmpPointRight.z = 0.0f;
//                referenceLine.push_back(tmpPointLeft);
//                referenceLine.push_back(tmpPointRight);
//            }

//        }

//            /* Transformation matrix coefficients; m_f32Rotation and m_f32Translation ;
//         * z points straight in front of car , y points downwards to the ground, x points to the right hand side! */
//        tFloat32 RT_xx = 1.0f;
//        tFloat32 RT_xy = 0.0f;
//        tFloat32 RT_xz = 0.0f;
//            tFloat32 RT_xo = m_f32CameraOffsetY + m_f32Camera_DepthToRgbOffest; // 0.0f; property is in car-coordinates !
//        tFloat32 RT_yx = 0.0f;
//    //	tFloat32 RT_yy = cos_m_pitch_rad;
//            tFloat32 RT_yy = m_f32CosDynRange;
//    //	tFloat32 RT_yz = -sin_m_pitch_rad;
//            tFloat32 RT_yz = -m_f32SinDynRange;
//            tFloat32 RT_yo = m_f32CameraOffsetZ; // -0.22f; property is in car-coordinates !
//        tFloat32 RT_zx = 0.0f;
//    //	tFloat32 RT_zy = sin_m_pitch_rad;
//            tFloat32 RT_zy = m_f32SinDynRange;
//    //	tFloat32 RT_zz = cos_m_pitch_rad;
//            tFloat32 RT_zz = m_f32CosDynRange;
//            tFloat32 RT_zo = -m_f32CameraOffsetX; //-0.2f; property is in car-coordinates !
//    //	const tFloat32 T_ox = 0.0f;
//    //	const tFloat32 T_oy = 0.0f;
//    //	const tFloat32 T_oz = 0.0f;
//    //	const tFloat32 T_oo = 1.0f;

//        if (referenceLine.size() <= 0) {
//            RETURN_AND_LOG_ERROR_STR(ERR_INVALID_INDEX,
//                    cString::Format(
//                            "ObstDetec: GCL-dynamic: Invalid size of vector: %d",
//                            referenceLine.size()));
//        }

//        /* perform actual transformation */
//        for (tUInt32 i = 0; i < referenceLine.size(); i++) {

//                    /* Re-assign m_bTransformed point cloud values to original pointcloud values, change from car-coordinates to depth-image coordinates */
//            tFloat32 tmp_z = referenceLine[i].x;
//            tFloat32 tmp_x = -referenceLine[i].y;
//            tFloat32 tmp_y = -referenceLine[i].z;

//                    /* m_f32Translation backwards */
//            tmp_x = tmp_x - (RT_xo * 1.0f);
//            tmp_y = tmp_y - (RT_yo * 1.0f);
//            tmp_z = tmp_z - (RT_zo * 1.0f);
//    //		LOG_WARNING(cString::Format("ObstacleDetec: After back-m_f32Translation - x: %f, y: %f, z: %f",tmp_x,tmp_y,tmp_z));
//                    /* m_f32Rotation backwards-- maybe first rotate pi/2, then translate, then rotate pitch */

//            referenceLine[i].x = (RT_xx * tmp_x) + (RT_xy * tmp_y)
//                    + (RT_xz * tmp_z);
//            referenceLine[i].y = (RT_yx * tmp_x) + (RT_yy * tmp_y)
//                    + (RT_yz * tmp_z);
//            referenceLine[i].z = (RT_zx * tmp_x) + (RT_zy * tmp_y)
//                    + (RT_zz * tmp_z);

//    //		LOG_WARNING(cString::Format("ObstacleDetec: After back-m_f32Rotation- x: %f, y: %f, z: %f",referenceLine[i].x,referenceLine[i].y,referenceLine[i].z));
//    //		LOG_WARNING(cString::Format("ObstacleDetec: After back-m_f32Rotation- sin: %f, cos: %f",sin_m_dynRange_rad, cos_m_dynRange_rad));

//        }
//            /* At this point, the point-cloud data is m_bTransformed back to original orientation from camera view */
//            /* Now, point cloud needs to be m_bTransformed back to depth-image frame-resolution and format */

//        for (tUInt32 i = 0; i < referenceLine.size(); i++) {

//                    referenceLine[i].x = ((referenceLine[i].x * _depthIntrinsicData.m_f32Fx)
//                                    / referenceLine[i].z) + _depthIntrinsicData.m_f32Cx;
//                    referenceLine[i].y = ((referenceLine[i].y * _depthIntrinsicData.m_f32Fy)
//                                    / referenceLine[i].z) + _depthIntrinsicData.m_f32Cy;
//            //referenceLine[i].z = referenceLine[i].z;
//    //		LOG_WARNING(cString::Format("ObstacleDetec: After depth-back-trafo- x: %f, y: %f, z: %f",referenceLine[i].x,referenceLine[i].y,referenceLine[i].z));
//        }

//            if (m_bShowGCLDebug_extendedLog) {
//            LOG_WARNING(
//                    cString::Format(
//                            "ObstacleDetec: GCL -> should print %d elements.",
//                            referenceLine.size()));
//        }

//        // create a mediasample for GCL output
//        cObjectPtr<IMediaSample> pSample;
//        RETURN_IF_FAILED(AllocMediaSample((tVoid** )&pSample));

//        RETURN_IF_FAILED(pSample->AllocBuffer(8192));

//            pSample->SetTime(m_pClock->GetStreamTime());

//        tUInt32* aGCLProc;
//        RETURN_IF_FAILED(pSample->WriteLock((tVoid** )&aGCLProc));

//        tUInt32* pc = aGCLProc;

//        /* Do the actual drawing -> create rectangles representing the search corridor */
//        cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 70, 0).GetRGBA());
//        /* Draw a rectangle to scale the GCL output to the video display */
//        cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, 0, 0,
//                            2 * m_inputFormat.nWidth, 2 * m_inputFormat.nHeight);

//            if (m_bShowGCLDebug_extendedLog)
//            LOG_WARNING(
//                    cString::Format(
//                            "ObstDetec: GCL-dynamic: Point to plot: x:%d, y: %d",
//                                                    m_inputFormat.nWidth, m_inputFormat.nHeight));
//    //	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,70,0).GetRGBA());
//        if ((referenceLine.size() - 1) * 24 < 8192) {
//            for (tUInt32 i = 0; i < referenceLine.size(); i = i + 2) {
//                /* Set Color */
//                //cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, ((static_cast<tInt>(referenceLine[i].x) + (inputFormat.nWidth / 2)) * 2) - 2, ((static_cast<tInt>(referenceLine[i].y) + (inputFormat.nHeight / 2)) * 2) - 2,((static_cast<tInt>(referenceLine[i].x) + (inputFormat.nWidth / 2)) * 2) + 2, ((static_cast<tInt>(referenceLine[i].y) + (inputFormat.nHeight / 2)) * 2) + 2);
//    //			if(i < 40){
//    //				cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(238,0,0).GetRGBA()); // red
//    //			}
//    //			else if((i >= 40) && (i < 80)){
//    //				cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(238,238,0).GetRGBA()); // yellow
//    //			}
//    //			else{ // i>=80
//    //				cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(0,238,0).GetRGBA()); // green
//    //			}
//                if (interval
//                        > ((OACC_circSegmentDistance - OACC_frontAxisToFrontBumper)
//                                / 50.0f) * 0.92f) {
//                    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL,
//                            cColor(0, 238, 0).GetRGBA()); // green
//                } else if ((interval
//                        < ((OACC_circSegmentDistance - OACC_frontAxisToFrontBumper)
//                                / 50.0f) * 0.92f)
//                        && (interval
//                                > ((OACC_circSegmentDistance
//                                        - OACC_frontAxisToFrontBumper) / 50.0f)
//                                        * 0.35f)) {
//                    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL,
//                            cColor(238, 238, 0).GetRGBA()); // yellow
//                } else {
//                    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL,
//                            cColor(238, 0, 0).GetRGBA()); // red
//                }
//                /* Draw Rectangle */
//                cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,
//                        (static_cast<tInt>(referenceLine[i].x)) * 2 - 2,
//                        (static_cast<tInt>(referenceLine[i].y) * 2) - 2,
//                        (static_cast<tInt>(referenceLine[i + 1].x) * 2) + 2,
//                        (static_cast<tInt>(referenceLine[i + 1].y) * 2) + 2);
//                            if (m_bShowGCLDebug_extendedLog) {
//                    LOG_WARNING(
//                            cString::Format(
//                                    "ObstDetec: GCL-dynamic: Point to plot: x_left: %f, y_left: %f, x_right: %f, y_right: %f",
//                                    referenceLine[i].x, referenceLine[i].y,
//                                    referenceLine[i + 1].x,
//                                    referenceLine[i + 1].y));
//                }
//            }
//        } else {
//            LOG_WARNING(
//                    cString::Format(
//                            "ObstacleDetec: GCL -> too many points to draw, exit."));
//        }
//        cGCLWriter::StoreCommand(pc, GCL_CMD_END);

//        pSample->Unlock(aGCLProc);
//            RETURN_IF_FAILED(m_WriterGCLOutput.Transmit(pSample));

        RETURN_NOERROR;
}

//TODO: update CreateAndTransmitGCL to new data format
tResult ObstacleDetection::CreateAndTransmitGCL(
        const std::vector<cv::Point2i> &referenceSegmentLine) {
//	LOG_WARNING(
//			cString::Format("ObstacleDetec: GCL -> should print %d circles.",
//					referenceSegmentLine.size()));
//	// just draw gcl if the pin is connected and debug mode is enabled
//        if (!m_WriterGCLOutput.IsConnected() || !m_bShowGCLDebug) {
//		RETURN_NOERROR;
//	}

//	// create a mediasample
//	cObjectPtr<IMediaSample> pSample;
//	RETURN_IF_FAILED(AllocMediaSample((tVoid** )&pSample));

//	RETURN_IF_FAILED(pSample->AllocBuffer(8192));

//        pSample->SetTime(m_pClock->GetStreamTime());

//	tUInt32* aGCLProc;
//	RETURN_IF_FAILED(pSample->WriteLock((tVoid** )&aGCLProc));

//	tUInt32* pc = aGCLProc;

//	// draw rectangle to scale the video display correctly
//	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(0, 0, 0).GetRGBA());
//	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, 0, 0,
//                        2 * m_inputFormat.nWidth, 2 * m_inputFormat.nHeight);

//	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 70, 0).GetRGBA());
//	if (referenceSegmentLine.size() * 24 < 8192) {
//		for (tUInt32 i = 0; i < referenceSegmentLine.size(); i++) {
//			cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,
//					referenceSegmentLine[i].x * 2 - 2,
//					referenceSegmentLine[i].y * 2 - 2,
//					referenceSegmentLine[i].x * 2 + 2,
//					referenceSegmentLine[i].y * 2 + 2);
//		}
//	} else {
//		LOG_WARNING(
//				cString::Format(
//						"ObstacleDetec: GCL -> too many points to draw, exit."));
//	}
//	//	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, inputFormat.nWidth - inputFormat.nWidth/2, inputFormat.nHeight - inputFormat.nHeight/2, inputFormat.nWidth + inputFormat.nWidth/2, inputFormat.nHeight + inputFormat.nHeight/2);

////    // draw the min and max lane width
////    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(150,150,150).GetRGBA());
////    cGCLWriter::StoreCommand(pc, GCL_CMD_FILLRECT, referenceSegmentLine[i].x - m_i16LaneWidthMaxFar/2, m_nFarLine, m_i16FarLaneCenter + m_i16LaneWidthMaxFar/2, m_nFarLine + 5);
////    cGCLWriter::StoreCommand(pc, GCL_CMD_FILLRECT, m_i16FarLaneCenter - m_i16LaneWidthMinFar/2, m_nFarLine - 5, m_i16FarLaneCenter + m_i16LaneWidthMinFar/2, m_nFarLine);
////
////    // draw near and far line
////    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,70,0).GetRGBA());
////    cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, 0, m_nCurrentNearLine, m_sInputFormat.nWidth, m_nCurrentNearLine);
////    cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, 0, m_nFarLine, m_sInputFormat.nWidth, m_nFarLine);

//	// draw the found near lines
////    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,50,100).GetRGBA());
////    cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, referenceSegment[0].x*2 , referenceSegment[0].y*2,referenceSegment[0].x*2 + 10, referenceSegment[0].y*2 + 10);

////    // draw the found far lines
////    for (tInt nIdx = 0; nIdx < m_ui8FarPointsCount; ++nIdx)
////    {
////        cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, m_asAllpointsFar[nIdx].x, m_nFarLine - 20, m_asAllpointsFar[nIdx].x, m_nFarLine + 20);
////    }

//	cGCLWriter::StoreCommand(pc, GCL_CMD_END);

//	pSample->Unlock(aGCLProc);

//        RETURN_IF_FAILED(m_WriterGCLOutput.Transmit(pSample));
    RETURN_NOERROR;

}


tResult ObstacleDetection::LoadConfigurationData(cFilename& m_fileConfig,
        vector<tFloat32>& m_xValues, vector<tFloat32>& m_yValues) {

    // check if file exits
    if (m_fileConfig.IsEmpty()) {
        RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,
                cString::Format(
                        "ObstacleDetec: Configuration xml-file not found."));
    }
    // create absolute path
    ADTF_GET_CONFIG_FILENAME(m_fileConfig);
    m_fileConfig = m_fileConfig.CreateAbsolutePath(".");

    //Load file, parse configuration, print the data

    if (cFileSystem::Exists(m_fileConfig)) {
        cDOM oDOM;
        oDOM.Load(m_fileConfig);
        //load supporting points
        cDOMElementRefList oElems;
        if (IS_OK(
                oDOM.FindNodes("calibrationOpticACC/supportingPoints/point",
                        oElems))) {
            for (cDOMElementRefList::iterator itElem = oElems.begin();
                    itElem != oElems.end(); ++itElem) {
                cDOMElement* pConfigElement;
                if (IS_OK((*itElem)->FindNode("xValue", pConfigElement))) {
                    m_xValues.push_back(
                            tFloat32(
                                    cString(pConfigElement->GetData()).AsFloat64()));
                }
                if (IS_OK((*itElem)->FindNode("yValue", pConfigElement))) {
                    m_yValues.push_back(
                            tFloat32(
                                    cString(pConfigElement->GetData()).AsFloat64()));
                }
            }
        }
        if (oElems.size() > 0) {
                        if (m_xml_PrintInitialtable) {
                for (tUInt i = 0; i < m_xValues.size(); i++) {
                    if (i > m_yValues.size()) {
                        break;
                    } else {
                        LOG_WARNING(
                                cString::Format(
                                        "ObstacleDetec: supporting point #%d: (%lf / %lf)",
                                        i, m_xValues[i], m_yValues[i]));
                    }
                }
            }
        } else {
            RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,
                    "ObstacleDetec: no supporting points in given file found!");
        }
        //checks if data are valid
        RETURN_IF_FAILED(CheckConfigurationData(m_fileConfig, m_xValues));

    } else {
        RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,
                "ObstacleDetec: Configured configuration file not found");
    }

    RETURN_NOERROR;
}

//called by LoadConfigurationData
/*checks if Configuration data (lookUpTable with speed, depending on distance to an obstacle)*/
/*is in increasing order (has to be otherwise configuration data is corrupt..) */
tResult ObstacleDetection::CheckConfigurationData(cFilename m_fileConfig,
        vector<tFloat32> m_xValues) {
    //checks if the xValues of the calibration table are increasing
    for (vector<tFloat32>::iterator it = m_xValues.begin();
            it != m_xValues.end(); it++) {
        vector<tFloat32>::iterator it2 = it;
        it2++;
        if (it2 != m_xValues.end()) {
            // next values is smaller than current value
            if ((tFloat32(*it) > tFloat32(*it2))) {
                RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,
                        cString::Format(
                                "ObstacleDetec: The x-values in the file %s are not in increasing order. Please reorder the points!",
                                m_fileConfig.GetPtr()));
            }
        }
    }

    RETURN_NOERROR;
}

//Linear Interpolation for InputValues that lay between the point from configuration data
tFloat32 ObstacleDetection::GetLinearInterpolatedValue(tFloat32 fl32InputValue,
        vector<tFloat32> m_xValues, vector<tFloat32> m_yValues) {
    // requested value is smaller than smallest value in table
    if (fl32InputValue < m_xValues.front()) {
                if (m_xml_BorderWarningModeEnabled) {
            LOG_WARNING(
                    cString::Format(
                            "ObstacleDetec: requested x-value %f is lower than smallest x-value in calibration table",
                            fl32InputValue));
        }
        return m_yValues.front();
    }
    // requested value is bigger than biggest value in table
    else if (fl32InputValue > m_xValues.back()) {
                if (m_xml_BorderWarningModeEnabled) {
            LOG_WARNING(
                    cString::Format(
                            "ObstacleDetec: requested x-value %f is higher than highes x-value in calibration table",
                            fl32InputValue));
        }
        return m_yValues.back();
    }
    // search in vector for corresponding index (smallest neighbor)
    tUInt iIndex;
    if (m_xValues.size() > 2) {
        for (iIndex = 0; iIndex < m_xValues.size(); iIndex++) {
            if (m_xValues[iIndex] >= fl32InputValue) {
                break;
            }
        }
        // get smaller neighbor
        if (iIndex != 0) {
            iIndex = iIndex - 1;
        }
    } else {
        iIndex = 0;
    }
    if ((m_xValues[iIndex + 1] - m_xValues[iIndex]) != 0) {
        // doing the linear interpolation
        tFloat32 f32Value = (fl32InputValue - m_xValues[iIndex])
                * (m_yValues[iIndex + 1] - m_yValues[iIndex])
                / (m_xValues[iIndex + 1] - m_xValues[iIndex])
                + m_yValues[iIndex];

        //tFloat32 security check to send only minimum or maximum value of table
        if (f32Value > *max_element(m_yValues.begin(), m_yValues.end())) {
            return *max_element(m_yValues.begin(), m_yValues.end());
        } else if (f32Value
                < *min_element(m_yValues.begin(), m_yValues.end())) {
            return *min_element(m_yValues.begin(), m_yValues.end());
        } else {
            return f32Value;
        }
    } else {
        LOG_ERROR(
                "ObstacleDetec: Error occured. Invalid table in xml-config-file!");
        return -1.0f;
    }
}

//****************************************************************************************
//******                    TODO:      NOT ported Functions                         ******
//****** Debug Functions, Deprecated Functions etc. - might be needed to be ported! ******
//****************************************************************************************


//cv::Mat ObstacleDetection::generateDebugOutputOpticACC(
//		TSignalValue::Data steeringAngle,
//		const std::vector<cv::Point3f> &cloud) {
//	//LOG_WARNING(cString::Format("ObstacleDetec: generateDebugOutputOpticACC called with steering angle: %f.",steeringAngle.f32_value));
//        cv::Mat checked_pixel_depth_image(m_inputFormat.nHeight, m_inputFormat.nWidth,
//			CV_16UC1);
//	tBool pixel_no_obstacle = tTrue;
//	tUInt64 counter_tmp = 0;

//	tFloat32 corrSteeringAngleAbs = ((fabsf(steeringAngle.f32_value) - OACC_obliqueAngleFront)/180.0f)* CV_PI;
////	LOG_WARNING(cString::Format("ObstacleDetec: corrected_SteeringAngle absolute : %f",corrSteeringAngleAbs));

//	/* handle case that steringAngle would be smaller than zero due to obliqueAngle */
//	if(corrSteeringAngleAbs < 0.0f){
//		corrSteeringAngleAbs = 0.0f;
//		LOG_WARNING(cString::Format("ObstacleDetec: corrected_SteeringAngle absolute set to zero: %f",corrSteeringAngleAbs));
//	}
//	/* angle too small, thus radius will tend to infinity ->  handle by checking rectangular area right in front! */
//	if(corrSteeringAngleAbs < 0.005f){
//		//LOG_WARNING(cString::Format("ObstacleDetec: OpticACC- path with steering < 0.05 taken!"));
//		/* global minimum and maximum values including the y-search-tolerance for shrinking the in this case rectangular search corridor */
//		tFloat32 x_oacc_rect_min = 0.0f;
//		tFloat32 x_oacc_rect_max = OACC_circSegmentDistance
//				- OACC_frontAxisToFrontBumper;
//		//LOG_WARNING(cString::Format("ObstacleDetec: OACC x-search-corridor -> xmin: %f, xmax: %f",x_oacc_rect_min,x_oacc_rect_max));
//		if (x_oacc_rect_max < 0.0f) {
//			LOG_WARNING(
//					cString::Format(
//							"ObstacleDetec: OACC - search-corridor in x-direction too small, received input target speed will be returned unmodified!"));
//		}
//		tFloat32 y_oacc_rect_min = -OACC_ySearchTolerance;
//		tFloat32 y_oacc_rect_max = OACC_ySearchTolerance;
//		//LOG_WARNING(cString::Format("ObstacleDetec: OACC y-search-corridor -> ymin: %f, ymax: %f",y_oacc_rect_min,y_oacc_rect_max));
//		/* Iterate through all components of point cloud, check for matching data-points */
//		for (tUInt32 i = 0; i < cloud.size(); i++) {
//			//LOG_WARNING(cString::Format("ObstacleDetec: OpticACC- path1 - checking pointcloud!"));
//                        const tUInt32 u = i % m_inputFormat.nWidth; // columns
//                        const tUInt32 v = i / m_inputFormat.nWidth; // rows
//			pixel_no_obstacle = tTrue;
//			/* Check for proper height of obstacle, discard otherwise */
//			if (cloud[i].z > OACC_obstacleHeightThreshold_min
//					&& cloud[i].z < OACC_obstacleHeightThreshold_max) {
//				//LOG_WARNING(cString::Format("ObstacleDetec: OpticACC- pointcloud passed z-condition."));
//				/* take only values in the valid rectangular search field */
//				if (cloud[i].x >= x_oacc_rect_min
//						&& cloud[i].x <= x_oacc_rect_max) {
//					//LOG_WARNING(cString::Format("ObstacleDetec: OpticACC- pointcloud passed x-condition."));
//					if (cloud[i].y >= y_oacc_rect_min
//							&& cloud[i].y <= y_oacc_rect_max) {
//						//LOG_WARNING(cString::Format("ObstacleDetec: OpticACC- pointcloud passed y-condition."));

//						pixel_no_obstacle = tFalse;
//						counter_tmp++;
//						//LOG_WARNING(cString::Format("ObstacleDetec: OpticACC- found Obstacle-pixel, counter: %d!",counter_tmp));
//					}
//				}
//			}

//			if (pixel_no_obstacle) {
//				checked_pixel_depth_image.at<tUInt16>(v, u) =
//						OD_VI_CV16UC1_MAX_RANGE; // white
//			} else {
//				checked_pixel_depth_image.at<tUInt16>(v, u) = 0; // black
//			}
//		}
//		//LOG_WARNING(cString::Format("ObstacleDetec: OpticACC- found Obstacle-pixel, counter: %d!",counter_tmp));
//	} else {
//		/* radius between momentum-pole and middle of front-axis */
//		tFloat32 segmentRadius = OACC_wheelbase / sinf(corrSteeringAngleAbs);
//		tFloat32 segmentAngle = OACC_circSegmentDistance / segmentRadius;
//		/* if segmentAngle + steeringAngle would be greater than pi/2, no proper checking would be possible (atm) */
//		if((segmentAngle + corrSteeringAngleAbs) > CV_PI/2){
//			segmentAngle = CV_PI/2 - corrSteeringAngleAbs;
//		}

//		/* minimum and maximum values of the reference line-segment */
//		tFloat32 x_oacc_ref_min = 0.0f;
//		tFloat32 x_oacc_ref_max = segmentRadius * sinf(corrSteeringAngleAbs + segmentAngle) - OACC_wheelbase - OACC_frontAxisToFrontBumper;
//		tFloat32 y_oacc_ref_min = 0.0f;
//		tFloat32 y_oacc_ref_max = (OACC_wheelbase / tanf(corrSteeringAngleAbs)) - (segmentRadius * cosf(corrSteeringAngleAbs + segmentAngle));
////		LOG_WARNING(cString::Format("ObstacleDetec: OACC x-search-corridor_REF -> xmin: %f, xmax: %f",x_oacc_ref_min,x_oacc_ref_max));
////		LOG_WARNING(cString::Format("ObstacleDetec: OACC y-search-corridor_REF -> ymin: %f, ymax: %f",y_oacc_ref_min,y_oacc_ref_max));

//		/* global minimum and maximum values including the y-search-tolerance for shrinking the general rectangular search corridor */
//		tFloat32 x_oacc_ref_tol_min = x_oacc_ref_min;
//		tFloat32 x_oacc_ref_tol_max = x_oacc_ref_max;
//		tFloat32 y_oacc_ref_tol_min = 0.0f;
//		tFloat32 y_oacc_ref_tol_max = 0.0f;
//		/* In case that steering to left hand side, that means in positive y-direction */
//		if ((steeringAngle.f32_value) < 0) {
//			y_oacc_ref_tol_min = y_oacc_ref_min - OACC_ySearchTolerance;
//			y_oacc_ref_tol_max = y_oacc_ref_max + OACC_ySearchTolerance;
//		} else { //steering to right hand side
//			y_oacc_ref_tol_min = -y_oacc_ref_max - OACC_ySearchTolerance;
//			y_oacc_ref_tol_max = -y_oacc_ref_min + OACC_ySearchTolerance;
//		}
////		LOG_WARNING(cString::Format("ObstacleDetec: OACC y-search-corridor_REAL -> ymin: %f, ymax: %f, curr.steeringAngle: %f",y_oacc_ref_tol_min,y_oacc_ref_tol_max,steeringAngle.f32_value));
//		/* Relation between currently viewed/checked x_oacc and y_oacc */
//		tFloat32 y_curOacc_ref = 0.0f;
//		/* Iterate through all components of point cloud, check for matching data-points */
//		for (tUInt32 i = 0; i < cloud.size(); i++) {
//                        const tUInt32 u = i % m_inputFormat.nWidth; // columns
//                        const tUInt32 v = i / m_inputFormat.nWidth; // rows
//			pixel_no_obstacle = tTrue;
//			/* Check for proper height of obstacle, discard otherwise */
//			if (cloud[i].z > OACC_obstacleHeightThreshold_min
//					&& cloud[i].z < OACC_obstacleHeightThreshold_max) {
//				/* take only values in the valid rectangular search field */
//				if (cloud[i].x >= x_oacc_ref_tol_min
//						&& cloud[i].x <= x_oacc_ref_tol_max) {
//					if (cloud[i].y >= y_oacc_ref_tol_min
//							&& cloud[i].y <= y_oacc_ref_tol_max) {
//						/* check only values that fit the specific search corridor, given by the circular relation between x and y ; K-Y = y~ */
//						/* circle-formula returns only values for the case of steering left, that means (steeringAngle.f32_value ) < 0 */
//						y_curOacc_ref = (OACC_wheelbase / tanf(corrSteeringAngleAbs)) - sqrt((segmentRadius * segmentRadius) - ((cloud[i].x + OACC_wheelbase)*(cloud[i].x + OACC_wheelbase)));
//						/* case of steering right: */
//						if ((steeringAngle.f32_value) > 0) {
//							y_curOacc_ref = -y_curOacc_ref;
//						}
//						/* actual check is performed */
//						if ((cloud[i].y
//								>= (y_curOacc_ref - OACC_ySearchTolerance))
//								&& (cloud[i].y
//										<= y_curOacc_ref + OACC_ySearchTolerance)) {
//							/* At this point, currently checked data-point really is in the current sector of interest */
//							pixel_no_obstacle = tFalse;
//							counter_tmp++;
//						}
//					}
//				}
//			}
//			if (pixel_no_obstacle) {
//				checked_pixel_depth_image.at<tUInt16>(v, u) =
//						OD_VI_CV16UC1_MAX_RANGE; // white
//			} else {
//				checked_pixel_depth_image.at<tUInt16>(v, u) = 0; // black
//			}
//		}
//		//LOG_WARNING(cString::Format("ObstacleDetec: OpticACC- found Obstacle-pixel dynamic, counter: %d!",counter_tmp));
//	}

//	return checked_pixel_depth_image;
//}

/*****************************************************************************************/
/*****************************************************************************************/

//tResult ObstacleDetection::GetInputDepthImage(cv::Mat &image,
//		IMediaSample* sample) {
//        if (m_inputFormat.nPixelFormat != cImage::PF_GREYSCALE_16) {
//		RETURN_AND_LOG_ERROR_STR(ERR_NOT_SUPPORTED,
//				"ObstDetec: Only 16 bit greyscale images are supported.");
//	}

//	/* Create buffer-pointer to set on data of input stream and create temporary header for referencing to data */
//	const tVoid* srcBuffer;
//	IplImage* img = cvCreateImageHeader(
//                        cvSize(m_inputFormat.nWidth, m_inputFormat.nHeight), IPL_DEPTH_16U,
//			1);
//	RETURN_IF_FAILED(sample->Lock(&srcBuffer));
//	img->imageData = (char*) srcBuffer;
//	/* set reference onto data to 'image' (method argument)*/
//	image = cvarrToMat(img); //convert to Mat-type
//	cvReleaseImage(&img);// release img to decrease reference-counter, expressing img-reference will not be needed anymore
//	sample->Unlock(srcBuffer); //unlock buffer again

//	/*IMPORTANT NOTICE: During this procedure, data is NOT copied to a new memory location nor is it duplicated!
//	 * Only a new reference (new header) is now used to express the location of the original data!  (managed via a reference-counting mechanism of opencv) */

//	RETURN_NOERROR;
//}

/*****************************************************************************************/
/*****************************************************************************************/

//tResult ObstacleDetection::generatePointCloudFromDepthImage(
//		const cv::Mat& depthImage, vector<Point3f>& pointCloud) {
//	/* Check if depthImage has valid dataPointer data*/
//	if (depthImage.data == NULL) {
//		RETURN_AND_LOG_ERROR_STR(ERR_INVALID_ADDRESS,
//				cString::Format("ObstacleDetect: Bad data in depthImage."));
//	}

//	/* clear existing pointcloud-list and reserve space for new list;
//	 *  DO NOT use resize (with resize, list is filled with dummy-values) */
//	pointCloud.clear();
//	pointCloud.reserve(depthImage.rows * depthImage.cols);

//	//tUInt16* ptr = (tUInt16*) (depthImage.data);
//	for (tUInt32 y = 0; y < static_cast<tUInt32>(depthImage.rows); y++) {
//		for (tUInt32 x = 0; x < static_cast<tUInt32>(depthImage.cols); x++) {
//			cv::Point3f p;
//			tFloat32 z = ((tFloat32) depthImage.at<tUInt16>(y, x))
//                                        * m_f32DepthImageScaleFactor; // Convert to meters

//                        p.x = ((tFloat32) x - _depthIntrinsicData.m_f32Cx) * z
//                                        / _depthIntrinsicData.m_f32Fx;
//                        p.y = ((tFloat32) y - _depthIntrinsicData.m_f32Cy) * z
//                                        / _depthIntrinsicData.m_f32Fy;
//			p.z = z;

//			#ifdef DEBUG_TO_FILE_POINTCLOUD
//                        if (m_bPointcloud_to_file) {
//				fstream filePointcloud;
//				//filePointcloud.open("/tmp/point_cloud.dat", ios::out | ios::app);
//                                m_file_pointcloud << p.x << " " << p.y << " " << p.z << "\n";
//				//filePointcloud.close();
//			}
//			#endif
//			/* if debug video should be created, all values are necessary for re-transformation from point cloud vector to image-array
//			 *  therefore, no preprocessing will be performed; If NO DEBUG is requested, values in non-interesting/invalid area are
//			 *  instantly deleted and not be taken into account in further processing (range can be set via properties) */
//#ifdef DEBUG_MODE_OUTPUT_VIDEO
//			pointCloud.push_back(p);
//#else
//			//if(p.z > distance_threshold_min  && p.z < distance_threshold_max){ // z-value has to be checked, as it corresponds to  x-value in car-coordinates
//			//	if(p.y > height_threshold_max && p.y < height_threshold_min){ // inverted y-value has to be checked, as it corresponds to z-value in car-coordinates
//			pointCloud.push_back(p);
//			//	}
//			//}
//#endif
//			/* increase pointer to jump to memory location of next float value */
//			//ptr++;
//		}
//	}

//	#ifdef DEBUG_TO_FILE_POINTCLOUD

//                if (m_bPointcloud_to_file) {
//                        m_file_pointcloud.close();
//			cv::imwrite( "/tmp/Test3.bmp", depthImage );
//		}

//		static int c = 0;
//		if (c < 10) {
//			c++;
//		} else if (c == 10) {
//			c++;
//                        m_bPointcloud_to_file = tTrue;
//		} else {
//                        m_bPointcloud_to_file = tFalse;
//		}
//		#endif

//	RETURN_NOERROR;
//}

/*****************************************************************************************/
/*****************************************************************************************/

//tResult ObstacleDetection::transformPointCloudToCarPerspective(const std::vector<cv::Point3f> &cloud, std::vector<cv::Point3f> &m_bTransformedCloud)
//{
//	if(cloud.size() <= 0){
//		RETURN_AND_LOG_ERROR_STR(ERR_INVALID_INDEX, cString::Format("ObstDetec: Referenced pointcloud has invalid size: %d",cloud.size()));
//	}
////	if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("TranformPointCloud started"));

//        /* Rotational Matrix around x-axis, with m_f32Translation in negative y and positive z direction;
//	 *  pitch angle might have to be adapted (sign) */
////	cv::Mat T = (cv::Mat_<tFloat32>(4, 4) <<
////		1.0,  0.0, 0.0, 0.0,
////		0.0,  m_f32CosPitch, -m_f32SinPitch, -0.22,
////		0.0,  m_f32SinPitch,  m_f32CosPitch, 0.2,
////		0.0,  0.0, 0.0, 1.0
////	);
//        /* Transformation matrix coefficients; m_f32Rotation and m_f32Translation ;
//	 * z points straight in front of car , y points downwards to the ground, x points to the right hand side! */
//	const tFloat32 T_xx = 1.0f;
//	const tFloat32 T_xy = 0.0f;
//	const tFloat32 T_xz = 0.0f;
//        const tFloat32 T_xo = m_f32CameraOffsetY; // 0.0f; property is in car-coordinates !

//	const tFloat32 T_yx = 0.0f;
//        const tFloat32 T_yy = m_f32CosPitch;
//        const tFloat32 T_yz = -m_f32SinPitch;
//        const tFloat32 T_yo = m_f32CameraOffsetZ; // -0.22f; property is in car-coordinates !

//	const tFloat32 T_zx = 0.0f;
//        const tFloat32 T_zy = m_f32SinPitch;
//        const tFloat32 T_zz = m_f32CosPitch;
//        const tFloat32 T_zo = -m_f32CameraOffsetX; //-0.2f; property is in car-coordinates !
////	const tFloat32 T_ox = 0.0f;
////	const tFloat32 T_oy = 0.0f;
////	const tFloat32 T_oz = 0.0f;
////	const tFloat32 T_oo = 1.0f;

//        m_bTransformedCloud.clear();
//        m_bTransformedCloud.reserve(cloud.size());

//#ifdef DEBUG_TO_FILE_POINTCLOUD_TRANSFORMED
//        if (!m_bTransformed_pointcloud_to_file && m_bPointcloud_to_file) {
//                if (m_bDebugModeEnabled)
//			LOG_WARNING(cString::Format("Transformed logging."));
//                fstream m_file_pointcloud_trans;
//                m_file_pointcloud_trans.open("/tmp/point_cloud_trans.dat",
//				ios::out | ios::app);
//	}
//#endif

//	/* perform actual transformation */
//	for (tUInt32 i = 0; i < cloud.size(); i++) {
//		tFloat32 tmpX = (T_xx * cloud[i].x) + (T_xy * cloud[i].y)
//				+ (T_xz * cloud[i].z) + (T_xo);
//		tFloat32 tmpY = (T_yx * cloud[i].x) + (T_yy * cloud[i].y)
//				+ (T_yz * cloud[i].z) + (T_yo);
//		tFloat32 tmpZ = (T_zx * cloud[i].x) + (T_zy * cloud[i].y)
//				+ (T_zz * cloud[i].z) + (T_zo);

//		/* assign tmp_vals to point cloud values, change to car-coordinates */
//                Point3f m_bTransformedPoint;
//                m_bTransformedPoint.x = tmpZ;
//                m_bTransformedPoint.y = -tmpX;
//                m_bTransformedPoint.z = -tmpY;
//                m_bTransformedCloud.push_back(m_bTransformedPoint);

//#ifdef DEBUG_TO_FILE_POINTCLOUD_TRANSFORMED
//                if (!m_bTransformed_pointcloud_to_file && m_bPointcloud_to_file) {
//                        m_file_pointcloud_trans << cloud[i].x << " " << cloud[i].y << " "
//					<< cloud[i].z << "\n";

//		}
//#endif
//	}

//#ifdef DEBUG_TO_FILE_POINTCLOUD_TRANSFORMED
//        if (!m_bTransformed_pointcloud_to_file && m_bPointcloud_to_file) {
//		file_pointcloud_trans.close();
//                if (!m_bTransformed_logging_completed) {
//                        m_bTransformed_logging_completed = tTrue;
//                        m_bTransformed_pointcloud_to_file = tTrue;
//		}
//	}
//#endif
//	RETURN_NOERROR;
//}

/*****************************************************************************************/
/*****************************************************************************************/

//#ifdef DEBUG_MODE_OUTPUT_VIDEO
//cv::Mat ObstacleDetection::generateDebugDepthFromTransformedCloud(const std::vector<cv::Point3f> &cloud)
//{
//        cv::Mat depthWithObstacles(m_inputFormat.nHeight, m_inputFormat.nWidth, CV_16UC1);

//        tBool adaptDepthColor = m_ui32DebugType == 4;

//	for(tUInt32 i = 0; i<cloud.size(); i++) {
//                const tUInt32 u = i % m_inputFormat.nWidth; // columns
//                const tUInt32 v = i / m_inputFormat.nWidth;// rows

//		tBool obstacleDetected = tFalse;

//                if(m_ui32DebugType != 4) {
//			/* first take only the valid field of view for the camera to check rest */
//                        if(cloud[i].x > m_f32DistanceThresholdMin && cloud[i].x < m_f32DistanceThresholdMax) {
//                                if(cloud[i].z > m_f32HeightThresholdMin && cloud[i].z < m_f32HeightThresholdMax) {

//					/* check conditions depending on debug mode, e.g. OnComing, Cross, both */
//                                        // m_ui32DebugType: 1 for 'check both areas', 2 for 'oncoming traffic', 3 for 'crosstrafic right', 4 for 'no modification'
//                                        switch(m_ui32DebugType) {
//						case 1: /* check both areas for obstacles and mark them in white */
//						/* check for oncoming traffic at intersection */
//                                                if((cloud[i].x >= roi_intersection_onComingTraffic.bottomleft_corner.x) && (cloud[i].x <= (roi_intersection_onComingTraffic.bottomleft_corner.x + roi_intersection_onComingTraffic.m_f32Roi_x_height))) {
//                                                        if((cloud[i].y <= roi_intersection_onComingTraffic.bottomleft_corner.y) && (cloud[i].y >= (roi_intersection_onComingTraffic.bottomleft_corner.y + roi_intersection_onComingTraffic.m_f32Roi_y_width))) {

//								obstacle_detected = tTrue;
//							}
//						}
//						/* check for crossTraffic from right hand side at intersection */
//                                                if((cloud[i].x >= roi_intersection_crossTrafficRight.bottomleft_corner.x) && (cloud[i].x <= (roi_intersection_crossTrafficRight.bottomleft_corner.x + roi_intersection_crossTrafficRight.m_f32Roi_x_height))) {
//                                                        if((cloud[i].y <= roi_intersection_crossTrafficRight.bottomleft_corner.y) && (cloud[i].y >= (roi_intersection_crossTrafficRight.bottomleft_corner.y + roi_intersection_crossTrafficRight.m_f32Roi_y_width))) {

//                                        switch(m_ui32DebugType){
//							case 1:	/* check both areas for obstacles and mark them in white */
//								/* check for oncoming traffic at intersection */

//								obstacleDetected |= checkROI(cloud[i], roi_intersection_onComingTraffic);
//								obstacleDetected |= checkROI(cloud[i], roi_intersection_crossTrafficRight);
//								break;
//							case 2: /* check for oncoming traffic at intersection */
//								obstacleDetected |= checkROI(cloud[i], roi_intersection_onComingTraffic);
//								break;
//							case 3: /* check for crosstraffic from right hand side at intersection */
//								obstacleDetected |= checkROI(cloud[i], roi_intersection_crossTrafficRight);
//								break;
//							case 4: /* this point cannot be reached */
//								LOG_ERROR(cString::Format("ObstacleDetec: Debug Error, invalid state reached during debug-video-creation!"));
//								break;
//							case 5: /* check for oncoming traffic in driving mode */
//								obstacleDetected |= checkROI(cloud[i], roi_overtake_onComingTraffic);
//								break;
//							case 6: /* check for traffic on 'own' lane during overtake, before going back into lane */
//								obstacleDetected |= checkROI(cloud[i], roi_overtake_originalLane);
//								break;
//							case 7: /* check for traffic on 'own' lane during overtake, before going back into lane */
//								obstacleDetected |= checkROI(cloud[i], roi_overtake_ownLaneStraight);
//								break;
//							case 8: /* check for traffic on oncoming lane, right before parking trans */
//								obstacleDetected |= checkROI(cloud[i], roi_parking_cross_onComing);
//								break;
//							case 9: /* check for cross-traffic, right before pulling out of maneuver 'parking trans' */
//								obstacleDetected |= checkROI(cloud[i], roi_parking_cross_crossTraffic);
//								break;
//					}
//				}
//			}
//		}

//		if(!adaptDepthColor){
//			if(obstacleDetected){
//				depthWithObstacles.at<tUInt16>(v, u) = 256*256-1; // white
//			}
//			else{
//				depthWithObstacles.at<tUInt16>(v, u) = 0; // black
//			}
//		}
//		else { //adapt color, but leave rest unmanipulated
//			tFloat32 x_tmp = cloud[i].x;
//			if(cloud[i].x > 3.0f) x_tmp = 3.0f;
//			depthWithObstacles.at<tUInt16>(v, u) = static_cast<tUInt16>((x_tmp /3.0f)*65535.0f);
//		}

//	}

//	return depthWithObstacles;
//}
//#endif

/*****************************************************************************************/
/*****************************************************************************************/

///* method to read the pitch from a previously created file, supposedly by camera-pitch-estimation functionality */
//tResult ObstacleDetection::ReadFromFile(tFloat32 *pitch) {
//        if (m_filePitch.GetLength() < 2) {
//		RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,
//				"ObstacleDetec: No filename for the pitch angle is given! If default should be used, set corresponding property.");
//	}

//	// create absolute path
//        ADTF_GET_CONFIG_FILENAME(m_filePitch);
//        cFilename absFilename = m_filePitch.CreateAbsolutePath(".");

//        if (cFileSystem::Exists(m_filePitch) == tFalse) {
//		RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,
//				"ObstacleDetec: File for stored pitch angle does not exist.");
//	}

//	cFile file;
//	file.Open(absFilename.GetPtr(), cFile::OM_Read);

//	cString tmp;
//	file.ReadLine(tmp); // comment line
//	file.ReadLine(tmp);

//	if (tmp.GetLength() >= 1) {
//		*pitch = tmp.AsFloat64();
//	} else {
//		RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,
//				cString::Format(
//						"ObstacleDetec: Error in opening the file %s. If value set by property should be used, please set corresponding flag!",
//						absFilename.GetPtr()));
//	}

//	RETURN_NOERROR;
//}

/*****************************************************************************************/
/*****************************************************************************************/


///* returns gridcell resolution based on property */
//tUInt32 ObstacleDetection::GetResolutionFromIndicator(
//		tUInt32 OACC_GridCellResolutionIndicator) {
//	tUInt32 gridCellResolution = 0;
//	switch (OACC_GridCellResolutionIndicator) {
//	case 1:
//		gridCellResolution = 2;
//		break;
//	case 2:
//		gridCellResolution = 4;
//		break;
//	case 3:
//		gridCellResolution = 5;
//		break;
//	case 4:
//		gridCellResolution = 8;
//		break;
//	case 5:
//		gridCellResolution = 10;
//		break;
//	case 6:
//		gridCellResolution = 16;
//		break;
//	case 7:
//		gridCellResolution = 20;
//		break;
//	}
//	return gridCellResolution;
//}

/*****************************************************************************************/
/*****************************************************************************************/
