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
#include "DriverModuleWidget.h"
#include <a_utils/core/a_utils_core.h>
#include <aadc_structs.h>
#include <boost/thread.hpp>

// namespace
using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace aadc::jury;
#define CID_CAR_CONTROLLER  "driver_module.filter.user.aadc.cid"
#define LABEL_CAR_CONTROLLER  "Driver Module"

class DriverModule : public QObject, virtual public cQtUIFilter
{
    Q_OBJECT

public slots:
    /*! function which transmits the state
    * \param stateID state to be sent; -1: error, 0: Ready, 1: Running
    * \param i16ManeuverEntry current entry to be sent
    */
    tResult OnSendState(aadc::jury::stateCar stateID, tInt16 i16ManeuverEntry);
    tResult TransmitDriverStruct(tDriverStruct& driverStruct);

    /*! this functions loads the maneuver list given in the properties
    * \result Returns a standard result code.
    */
    tResult LoadManeuverList();

signals:
    /*! signal to the gui to show the command "run" from the jury
    * \param entryId current entry to be sent
    */
    void SendRun(int entryId);

    /*! signal to the gui to show the command "stop" from the jury
    * \param entryId current entry to be sent
    */
    void SendStop(int entryId);

    /*! signal to the gui to show the command "request ready" from the jury
    * \param entryId current entry to be sent
    */
    void SendRequestReady(int entryId);

    /*! Trigger to load maneuver list. */
    void TriggerLoadManeuverList();

public:
    ADTF_CLASS_ID_NAME(DriverModule, CID_CAR_CONTROLLER, LABEL_CAR_CONTROLLER);
    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::ui::ant::IQtXSystem),
                            REQUIRE_INTERFACE(adtf::services::IReferenceClock));

private:
    /*! The property enable console output */
    adtf::base::property_variable<tBool>        m_propEnableConsoleOutput       = tFalse;
    adtf::base::property_variable<tBool>        m_bDebugModeEnabled             = tTrue;    /*! whether output to console is enabled or not*/

    /*! sample writer of The output driver structure */
    cPinWriter     m_oOutputDriverStruct;
    /*! sample reader The input jury structure */
    cPinReader     m_oInputJuryStruct;
    /*! sample reader List of input maneuvers */
    cPinReader     m_oInputManeuverList;

    //Media Descriptions
    struct tJuryStructId
    {
        tSize actionId;
        tSize maneuverEntry;
    } m_ddlJuryStructId;

    /*! The jury structure sample factory */
    cSampleCodecFactory m_juryStructSampleFactory;

    struct tDriverStructId
    {
        tSize stateId;
        tSize maneuverEntry;
    } m_ddlDriverStructId;

    /*! The driver structure sample factory */
    cSampleCodecFactory m_driverStructSampleFactory;

    TActionStruct o_TActionStruct;
    TFeedbackStruct o_TFeedbackStruct;

    TActionStruct::Data m_dataActionIn;

    iActionStruct m_ActionStructId;
    iFeedbackStruct m_FeedbackStructId;

    cPinReader m_ReaderActionStruct;
    cPinWriter m_WriterFeedbackStruct;


    adtf::mediadescription::cSampleCodecFactory m_actionSampleFactory;
    adtf::mediadescription::cSampleCodecFactory m_feedbackSampleFactory;

    tUInt32 m_ui32TimestampAction;
    tUInt32 counter_process = 0;
    /*! ui. */
    DisplayWidgetDriver*     m_pWidget;

    /*! The maneuver file string */
    cString     m_strManeuverFileString;

    /*! this is the list with all the loaded sections from the maneuver list*/
    aadc::jury::maneuverList m_sectorList;

    /*! The reference clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    /*! The mutex */
    std::mutex m_oMutex;

    /** Variables, parameters and other necessary declarations **/

        /* bool showing whether all maneuvers have been successfully completed */
        tBool m_bCompletedJuryList;

        /* flag expressing if jury-START-signal was already sent */
        tBool m_startSignalreceived;
        /* flag expressing if jury-START-signal was already requested */
        tBool m_startSignalrequested;
        /* flag expressing if maneuver list was already sent */
        tBool m_maneuverListLoaded;
        /* flag expressing if first maneuver was already sent */
        tBool m_maneuverFirstTransmitrequested;

        /** Managing the states of the Car via List **/
        /*! holds the current state of the car */
        stateCar m_CarState;

        /*! holds the (global) current maneuver id of the car*/
        tInt16 m_i16CurrentManeuverID;

        /*! holds the current index of the maneuvers in the list in the section */
        tInt16 m_i16ManeuverListIndex;

        /*! holds the current index in the lists of sections */
        tInt16 m_i16SectionListIndex;

    //    /*! handle for the timer */
    //    tHandle m_hTimer;

        /*! variable to temporarily save the previous, already successfully executed maneuver */
        tUInt32 tmpPrevManeuverInt_fb;




        /** Declaration of Critical Section Parameters **/
        /* Critical Section for Output Pins */
        /*! the critical section of the transmit of carState*/
        boost::mutex m_oCriticalSectionTransmitCarState;
        /*! the critical section of the transmit of ActionStruct */
        boost::mutex m_oCriticalSectionTransmitFeedbackStruct;
        /* Critical Section for Timer Setup */
        boost::mutex m_oCriticalSectionTimerSetup;
        /* Critical Section for CarState changes*/
        boost::mutex m_oCriticalSectionCarStatus;
        /* Critical Section for ManeuverList */
        boost::mutex m_oCriticalSectionManeuverList;
        /* Critical Section for BoolValue expressing whether maneuver list was already Loaded */
        boost::mutex m_oCriticalSectionManListLoaded;
        /* Critical Section for BoolValue expressing whether first transmit was requested */
        boost::mutex m_oCriticalSectionFirstTransmitReq;
        /* Critical Section for BoolValue expressing whether first transmit was requested */
        boost::mutex m_oCriticalSectionStartSignalReq;
        /* Critical Section for BoolValue expressing whether first transmit was requested */
        boost::mutex m_oCriticalSectionStartSignalReceived;
        /* Critical Section for BoolValue expressing whether jury maneuver list was completed */
        boost::mutex m_oCriticalSectionCompletedJuryList;
        /* Critical Section for accessing info regarding the previously executed maneuver */
        boost::mutex m_oCriticalSectionPreviousManeuverAccess;

public:
    /*! Default constructor. */
    DriverModule();
    /*! Destructor. */
    virtual ~DriverModule();




//        /*! creates the timer for the cyclic transmits*/
//        tResult createTimer();

//       /*! destroys the timer for the cyclic transmits*/
//        tResult destroyTimer();

        /* increments the id of the maneuver id by one and updates the list indexes;
         *  returns retval < 0 in case of error, retval = 0 for normal behaviour, retval > 0 for end of list reached */
        tInt32 incrementManeuverID();



        inline cString maneuverToCString(maneuver man)
                        {
                                switch (man)
                                {
                                        case maneuver_left:
                                                return cString("left");
                                                break;
                        case maneuver_right:
                            return cString("right");
                            break;
                        case maneuver_straight:
                            return cString("straight");
                            break;
                        case maneuver_parallel_parking:
                            return cString("parallel_parking");
                            break;
                        case maneuver_cross_parking:
                            return cString("cross_parking");
                            break;
                        case maneuver_pull_out_left:
                            return cString("pull_out_left");
                            break;
                        case maneuver_pull_out_right:
                            return cString("pull_out_right");
                            break;
                        case maneuver_merge_left:
                            return cString("merge_left");
                            break;
                        case maneuver_merge_right:
                            return cString("merge_right");
                            break;
                                        default:
                            return cString("");
                                                break;
                                }
                        }

		inline int extraFromString(std::string man)
		{
			LOG_INFO("%s", man);
			if (man.compare("1") == 0)
			{
                return 7;
			}
            else if (man.compare("2") == 0)
            {
                return 6;
            }
            else if (man.compare("3") == 0)
            {
                return 5;
            }
            else if (man.compare("4") == 0)
            {
                return 4;
            }
            else if (man.compare("5") == 0)
            {
                return 3;
            }
            else if (man.compare("6") == 0)
            {
                return 2;
            }
            else if (man.compare("7") == 0)
            {
                return 1;
            }
            else if (man.compare("8") == 0)
            {
                return 0;
            }
            
		    return -1;
		}		


        /*! resets the counters to the start of the current section*/
        tResult resetSection();

       /*! changes the state of the car
        @param newState the new state of the car
        */
        tResult changeState(stateCar newState);

        /*! set the maneuver id and find the correct indexes
        @param maneuverId the id of the maneuver which has to be set*/
        tResult setManeuverID(tInt maneuverId);

        /* returns the current maneuver from the maneuverList sent from jury, as uint32 (refer to enumeration in StateControl.h) */
        tUInt32 getCurrentManeuverFromJuryList();

        /* returns the finishing maneuver from the maneuverList sent from jury, that means the maneuver that was executed
         * as the last maneuver on the list, as uint32 (refer to enumeration in StateControl.h) */
        tUInt32 getFinishingManeuverFromJuryList();

        /* decodes the jury-maneuver in 'cString' format and returns the corresponding predefined tUInt32 (refer to enumeration in StateControl.h)*/
        tUInt32 decodeManeuverFromJuryList(cString tmp_currJuryManeuver, int);

        /* Processes the received ActionStruct from SCM */
        tResult ProcessActionData(TActionStruct::Data  );

        /* Function to change or access information regarding the previously executed maneuver */
         tUInt32 getPreviousManeuver();
         tResult setPreviousManeuver(tUInt32 maneuver);
         tResult resetPreviousManeuver();

protected: // Implement cBaseQtFilter
    QWidget * CreateView() override;
    tVoid    ReleaseView() override;
    tResult OnIdle() override;
    tResult  OnTimer() override;
    tResult  Init(tInitStage eStage) override;
    tResult  Shutdown(cFilterLevelmachine::tInitStage eStage) override;

};
