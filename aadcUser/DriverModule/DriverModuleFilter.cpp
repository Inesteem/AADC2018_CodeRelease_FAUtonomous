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

//otherwise cDOM will cause a deprecated warning, however there is no alternative yet
#define A_UTILS_NO_DEPRECATED_WARNING

#include "DriverModuleFilter.h"
#include "DriverModuleWidget.h"
#include <aadc_structs.h>
#include "ScmCommunication.h"

#define CONSOLE_LOG(_text, _log_level) if (m_propEnableConsoleOutput) { LOG_ADD_ENTRY(_log_level, _text); }    //!< enables log function if console output is activated
#define CONSOLE_//LOG_INFO(_text)      CONSOLE_LOG(_text, A_UTILS_NS::log::tLogLevel::Info)                        //!< log info messages


ADTF_PLUGIN(LABEL_CAR_CONTROLLER, DriverModule)

using namespace aadc::jury;

#define DEBUG_TO_CONSOLE "Debug::Debug output to console"
#define DEBUG_PRINT_STRUCTURE "Debug::Print Structure to Console"
#define DEBUG_SENDSTATE_OUTPUT "Debug::Print SendState Outputs to Console (cyclic)"
#define SC_PROP_FILENAME "Maneuver File"

DriverModule::DriverModule() : m_pWidget(nullptr)
{
    //Register Properties
    RegisterPropertyVariable("Enable console log", m_propEnableConsoleOutput);
    RegisterPropertyVariable("Enable console log111", m_bDebugModeEnabled);

    //Get Media Descriptions
    object_ptr<IStreamType> pTypeJuryStruct;
    cString structName = "tJuryStruct";
    if(IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(structName.GetPtr(), pTypeJuryStruct, m_juryStructSampleFactory)))
    {
        (adtf_ddl::access_element::find_index(m_juryStructSampleFactory, cString("i16ActionID"), m_ddlJuryStructId.actionId));
        (adtf_ddl::access_element::find_index(m_juryStructSampleFactory, cString("i16ManeuverEntry"), m_ddlJuryStructId.maneuverEntry));
    }
    else
    {
        LOG_WARNING(cString::Format("No mediadescription for %s found!", structName.GetPtr()));
    }
    create_pin(*this, m_oInputJuryStruct, "jury_struct", pTypeJuryStruct);
    
    object_ptr<IStreamType> pTypeDriverStruct;
    structName = "tDriverStruct";
    if (IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(structName.GetPtr(), pTypeDriverStruct, m_driverStructSampleFactory)))
    {
        (adtf_ddl::access_element::find_index(m_driverStructSampleFactory, cString("i16StateID"), m_ddlDriverStructId.stateId));
        (adtf_ddl::access_element::find_index(m_driverStructSampleFactory, cString("i16ManeuverEntry"), m_ddlDriverStructId.maneuverEntry));
    }
    else
    {
        LOG_WARNING(cString::Format("No mediadescription for %s found!", structName.GetPtr()));
    }
    create_pin(*this, m_oOutputDriverStruct, "driver_struct", pTypeDriverStruct);

    object_ptr<IStreamType> pTypeDefault = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_anonymous());
    create_pin(*this, m_oInputManeuverList, "maneuver_list", pTypeDefault);


    object_ptr<IStreamType> pActionStructSample;


    /*! get index of action struct */
    if(IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tActionStruct", pActionStructSample, m_actionSampleFactory)))
    {
        // get value
        adtf_ddl::access_element::find_index(m_actionSampleFactory, cString("bEnabled"), m_ActionStructId.bEnabled);
        adtf_ddl::access_element::find_index(m_actionSampleFactory, cString("bStarted"), m_ActionStructId.bStarted);
        adtf_ddl::access_element::find_index(m_actionSampleFactory, cString("ui32Command"),m_ActionStructId.ui32Command);
        adtf_ddl::access_element::find_index(m_actionSampleFactory, cString("ui8FilterId"),m_ActionStructId.ui8FilterId);

    }else
    {
        LOG_WARNING(cString::Format("No mediadescription for tActionStruct found!"));
    }
    create_pin(*this, m_ReaderActionStruct, "action_from_scm", pActionStructSample);

    object_ptr<IStreamType> pFeedbackStructSample;

    /*! get index of feedback struct */
    if(IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tFeedbackStruct", pFeedbackStructSample, m_feedbackSampleFactory)))
    {
        // get value
        adtf_ddl::access_element::find_index(m_feedbackSampleFactory, cString("ui8FilterId"), m_FeedbackStructId.ui8FilterId);
        adtf_ddl::access_element::find_index(m_feedbackSampleFactory, cString("ui32FeedbackStatus"),m_FeedbackStructId.ui32FeedbackStatus);
    }else
    {
        LOG_WARNING(cString::Format("No mediadescription for tFeedbackStruct found!"));
    }
    create_pin(*this, m_WriterFeedbackStruct, "feedback_to_scm", pFeedbackStructSample);


}

DriverModule::~DriverModule()
{
}

QWidget* DriverModule::CreateView()
{
    // use single UI File in background
    m_pWidget = new DisplayWidgetDriver(nullptr);

    qRegisterMetaType<aadc::jury::stateCar>("aadc::jury::stateCar");
    qRegisterMetaType<tInt16>("tInt16");

    tBool allConected = tTrue;
    allConected &= tBool(connect(m_pWidget, SIGNAL(sendStruct(aadc::jury::stateCar, tInt16)), this, SLOT(OnSendState(aadc::jury::stateCar, tInt16))));
    allConected &= tBool(connect(this, SIGNAL(SendRun(int)), m_pWidget, SLOT(OnDriverGo(int))));
    allConected &= tBool(connect(this, SIGNAL(SendStop(int)), m_pWidget, SLOT(OnDriverStop(int))));
    allConected &= tBool(connect(this, SIGNAL(SendRequestReady(int)), m_pWidget, SLOT(OnDriverRequestReady(int))));
    allConected &= tBool(connect(this, SIGNAL(TriggerLoadManeuverList()), this, SLOT(LoadManeuverList())));

    if (!allConected)
    {
        LOG_WARNING("DriverModule - not all Signal could be connected with an slot");
    }

    // disable maneuver group box until receiving maneuver list
    m_pWidget->EnableManeuverGroupBox(false);
    return m_pWidget;
}

tVoid DriverModule::ReleaseView()
{
    delete m_pWidget;
    m_pWidget = nullptr;
}

tResult DriverModule::OnIdle()
{
    std::lock_guard<std::mutex> oGuard(m_oMutex);

    RETURN_NOERROR;
}

tResult DriverModule::Init(tInitStage eStage)
{
    /* Jury and Maneuver communication*/
    m_i16CurrentManeuverID = 0;
    m_startSignalreceived = tFalse;
    m_maneuverFirstTransmitrequested = tFalse;
    m_maneuverListLoaded = tFalse;
    m_startSignalrequested = tFalse;
    m_bCompletedJuryList = tFalse;
    m_CarState = statecar_startup;
    /* using the method provides sending the state to the jury if module is already connected */
    //changeState(stateCar_STARTUP);
    tmpPrevManeuverInt_fb = FB_JURY_PREVIOUS_NO_MAN_EXISTING;

    /* Jury and Maneuver communication */
    m_i16SectionListIndex = -1;
    m_i16ManeuverListIndex =-1;


    RETURN_IF_FAILED(adtf::ui::cQtUIFilter::Init(eStage));
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}

tResult DriverModule::Shutdown(cFilterLevelmachine::tInitStage eStage)
{
    return cQtUIFilter::Shutdown(eStage);
}

tResult DriverModule::OnTimer()
{
    std::lock_guard<std::mutex> oGuard(m_oMutex);

    counter_process += 1;

    object_ptr<const ISample> pSample;
    while(IS_OK(m_oInputJuryStruct.GetNextSample(pSample)))
    {
        auto oDecoder = m_juryStructSampleFactory.MakeDecoderFor(*pSample);
        RETURN_IF_FAILED(oDecoder.IsValid());
        tJuryStruct juryInput;
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlJuryStructId.maneuverEntry, &juryInput.i16ManeuverEntry));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlJuryStructId.actionId, &juryInput.i16ActionID));

        tInt8 i8ActionID = juryInput.i16ActionID;
        tInt16 i16entry = juryInput.i16ManeuverEntry;

        switch (aadc::jury::juryAction(i8ActionID))
        {
        case action_getready:
//            CONSOLE_//LOG_INFO(cString::Format("Driver Module: Received Request Ready with maneuver ID %d", i16entry));
            emit SendRequestReady(static_cast<int>(i16entry));

            if(m_bDebugModeEnabled)  LOG_WARNING(cString::Format("JCom: Received 'Request Ready with maneuver ID %d'",i16entry));
            if(m_CarState==statecar_startup){
                if(m_maneuverListLoaded == tTrue){
                    changeState(statecar_ready);
                    setManeuverID(i16entry);
                }else{
                    LOG_ERROR(cString::Format("JCom: Received 'Request Ready', but maneuver list not loaded yet!"));
                    changeState(statecar_error);
                }
            }
            else{
                LOG_WARNING(cString::Format("JCom: Received 'Request Ready', but system not in startup state! Thus command will be ignored."));
            }
            break;
        case action_start:
//            CONSOLE_//LOG_INFO(cString::Format("Driver Module: Received Run with maneuver ID %d", i16entry));
            emit SendRun(static_cast<int>(i16entry));

            if(m_bDebugModeEnabled)  LOG_WARNING(cString::Format("JCom: Received: 'Action START with maneuver ID %d'",i16entry));
            /* (re)set previously executed maneuver to 'no_prev_man_existing' */
            tmpPrevManeuverInt_fb = FB_JURY_PREVIOUS_NO_MAN_EXISTING;
            if(m_startSignalreceived == tFalse){ // false for first time receiving start; set to true afterwards; to false again if STOP is sent
                if (i16entry == m_i16CurrentManeuverID){

                    changeState(statecar_running);
                }
                else{
                    LOG_WARNING(cString::Format("JCom: The id of the action_START corresponds not with the id of the last action_GETREADY! "
                                                "New ID %d will be used.",i16entry));
                    setManeuverID(i16entry);
                    changeState(statecar_running);
                }
                m_startSignalreceived = tTrue;
                if(m_startSignalrequested){
                    /* Send feedback to SCM if it was requested before */
                    TFeedbackStruct::Data feedback_tmp;
                    feedback_tmp.ui8FilterId = F_JURY_COMMUNICATION;
                    feedback_tmp.ui32FeedbackStatus = FB_JURY_COMMAND_RECEIVED_START;

                    RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedbackStruct, (void *) &feedback_tmp, m_pClock->GetStreamTime()));

                    m_startSignalrequested = tFalse;
                    if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("JCom: Sent requested feedback to SCM: ID %d, status %d",F_JURY_COMMUNICATION,FB_JURY_COMMAND_RECEIVED_START));
                }
            }
            else{
                if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("JCom: Action_START was received, but system already running due to previous start-signal!"));
            }

            break;
        case action_stop:
//            CONSOLE_//LOG_INFO(cString::Format("Driver Module: Received Stop with maneuver ID %d", i16entry));
            emit SendStop(static_cast<int>(i16entry));

            if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("JCom: Received: 'Stop with maneuver ID %d'",i16entry));
            /* (re)set previously executed maneuver to 'no_prev_man_existing' */
            tmpPrevManeuverInt_fb = FB_JURY_PREVIOUS_NO_MAN_EXISTING;
            m_startSignalreceived = tFalse;
            m_bCompletedJuryList = tFalse;
            changeState(statecar_startup);
            /* Send feedback to SCM, will have priority-handling; SCM has to stop current maneuver and jump to startup mode, all filters disabled*/
            TFeedbackStruct::Data feedback_tmp;
            feedback_tmp.ui8FilterId = F_JURY_COMMUNICATION;
            feedback_tmp.ui32FeedbackStatus = FB_JURY_COMMAND_RECEIVED_STOP;

            RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedbackStruct, (void *) &feedback_tmp, m_pClock->GetStreamTime()));
//

            break;
        }
    }


    object_ptr<const ISample> pSampleAnonymous;
    while (IS_OK(m_oInputManeuverList.GetNextSample(pSampleAnonymous)))
    {

        std::vector<tChar> data;
        object_ptr_shared_locked<const ISampleBuffer> pSampleBuffer;
        RETURN_IF_FAILED(pSampleAnonymous->Lock(pSampleBuffer));
        data.resize(pSampleBuffer->GetSize());
        memcpy(data.data(), pSampleBuffer->GetPtr(), pSampleBuffer->GetSize());
        if (data.size() > 0)
        {//maneuverlist
            m_strManeuverFileString.Set(data.data(), data.size());
            TriggerLoadManeuverList();
            /* Communicate with jury module about current car-state (supposedly STARTUP) */
            OnSendState(m_CarState,m_i16CurrentManeuverID);
            TFeedbackStruct::Data feedback_tmp;
            feedback_tmp.ui8FilterId = F_JURY_COMMUNICATION;
            feedback_tmp.ui32FeedbackStatus = FB_JURY_ALIVE_AND_READY;

            RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedbackStruct, (void *) &feedback_tmp, m_pClock->GetStreamTime()));
            if(m_bDebugModeEnabled) //LOG_INFO(cString::Format("JCom: ManeuverList loaded, ALIVE_AND_READY-status sent to scm."));

            // Check if first maneuver has already been requested by SCM
            if(m_maneuverFirstTransmitrequested)
            {
                 // get new/current maneuver from maneuverList
                tUInt32 currMan_tmp = getCurrentManeuverFromJuryList();
                if(currMan_tmp == 0)
                {
                    RETURN_AND_LOG_ERROR_STR(ERR_INVALID_STATE, cString::Format("JCom: No valid maneuver could be read from JuryManeuverList!"));

                }
                TFeedbackStruct::Data feedback_tmp;
                feedback_tmp.ui8FilterId = F_JURY_COMMUNICATION;
                feedback_tmp.ui32FeedbackStatus = currMan_tmp;
                //LOG_INFO("currMan_tmp");
                RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedbackStruct, (void *) &feedback_tmp, m_pClock->GetStreamTime()));
                m_maneuverFirstTransmitrequested = tFalse;
            }




        }
    }



        if(IS_OK(o_TActionStruct.readPin(m_ReaderActionStruct, (void *) & m_dataActionIn,m_ui32TimestampAction)))
        {
            // timestamps
            m_ui32TimestampAction = m_dataActionIn.ui32ArduinoTimestamp;
            // process data
            if (m_dataActionIn.ui8FilterId == F_JURY_COMMUNICATION)
            {
                RETURN_IF_FAILED(ProcessActionData(m_dataActionIn));
            }

        }




    RETURN_NOERROR;
}

tResult DriverModule::OnSendState(aadc::jury::stateCar stateID, tInt16 i16ManeuverEntry)
{

    tDriverStruct driverStruct;
    driverStruct.i16StateID = stateID;
    driverStruct.i16ManeuverEntry = i16ManeuverEntry;
    RETURN_IF_FAILED(TransmitDriverStruct(driverStruct));

    if (m_propEnableConsoleOutput)
    {
        switch (stateID)
        {
        case statecar_ready:
            //LOG_INFO(cString::Format("Driver Module: Send state: READY, Maneuver ID %d", i16ManeuverEntry));
            break;
        case statecar_running:
            //LOG_INFO(cString::Format("Driver Module: Send state: RUNNING, Maneuver ID %d", i16ManeuverEntry));
            break;
        case statecar_complete:
            //LOG_INFO(cString::Format("Driver Module: Send state: COMPLETE, Maneuver ID %d", i16ManeuverEntry));
            break;
        case statecar_error:
            //LOG_INFO(cString::Format("Driver Module: Send state: ERROR, Maneuver ID %d", i16ManeuverEntry));
            break;
        case statecar_startup:
            //LOG_INFO(cString::Format("Driver Module: Send state: STARTUP, Maneuver ID %d", i16ManeuverEntry));
            break;
        }
    }

    RETURN_NOERROR;
}

tResult DriverModule::TransmitDriverStruct(tDriverStruct& driverStruct)
{
    object_ptr<ISample> pWriteSample;

    RETURN_IF_FAILED(alloc_sample(pWriteSample, m_pClock->GetStreamTime()));
    {
        auto oCodec = m_driverStructSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDriverStructId.stateId, driverStruct.i16StateID));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDriverStructId.maneuverEntry, driverStruct.i16ManeuverEntry));
    }

    m_oOutputDriverStruct << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}


tResult DriverModule::LoadManeuverList()
{
    m_sectorList.clear();
    // create dom from string received from pin
    cDOM oDOM;
    oDOM.FromString(m_strManeuverFileString);
    cDOMElementRefList oSectorElems;
    cDOMElementRefList oManeuverElems;

    //read first Sector Elem
    if (IS_OK(oDOM.FindNodes("AADC-Maneuver-List/AADC-Sector", oSectorElems)))
    {
        //iterate through sectors
        for (cDOMElementRefList::iterator itSectorElem = oSectorElems.begin(); itSectorElem != oSectorElems.end(); ++itSectorElem)
        {
            //if sector found
            tSector sector;
            sector.id = (*itSectorElem)->GetAttributeUInt32("id");

            if (IS_OK((*itSectorElem)->FindNodes("AADC-Maneuver", oManeuverElems)))
            {
                //iterate through maneuvers
                for (cDOMElementRefList::iterator itManeuverElem = oManeuverElems.begin(); itManeuverElem != oManeuverElems.end(); ++itManeuverElem)
                {
                    tManeuver man;
                    man.id = (*itManeuverElem)->GetAttributeUInt32("id");


                    man.action = maneuverFromString((*itManeuverElem)->GetAttribute("action").GetPtr());
                    if(man.action == 6) // if cross parking, save id in extra
                    {
                        man.extra = extraFromString((*itManeuverElem)->GetAttribute("extra").GetPtr());
                        LOG_INFO("get parkid: %d", man.extra);
                    }

                        sector.sector.push_back(man);
                }
            }

            m_sectorList.push_back(sector);
        }
    }
    if (oSectorElems.size() > 0)
    {
        //LOG_INFO("DriverFilter: Loaded Maneuver file successfully.");
        m_pWidget->EnableManeuverGroupBox(true);
        m_maneuverListLoaded = tTrue;
    }
    else
    {
        LOG_ERROR("DriverFilter: no valid Maneuver Data found!");
        m_pWidget->EnableManeuverGroupBox(false);
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    // update the ui
    m_pWidget->SetManeuverList(m_sectorList);
    m_pWidget->ShowManeuverList();
    m_pWidget->FillComboBox();

    RETURN_NOERROR;
}


//// user defined functions:


tResult DriverModule::ProcessActionData(TActionStruct::Data inputAction)
{

    TFeedbackStruct::Data feedback_tmp;
    feedback_tmp.ui8FilterId = F_JURY_COMMUNICATION;

    /* set feedback.status according to input inputAction */
    if(inputAction.ui32Command == AC_JURY_ALIVE_AND_READY){
        /* send back status to prove 'up and running' */
        feedback_tmp.ui32FeedbackStatus = FB_JURY_ALIVE_AND_READY;

        RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedbackStruct, (void *) &feedback_tmp, m_pClock->GetStreamTime()));
    }
    else if(inputAction.ui32Command == AC_JURY_MAN_GET_FIRST){
        /* check if maneuver list was already loaded */
        if(m_maneuverListLoaded == tTrue){
            /* get new/current maneuver from maneuverList */
            tUInt32 currMan_tmp = getCurrentManeuverFromJuryList();
            if(currMan_tmp == 0){
                RETURN_AND_LOG_ERROR_STR(ERR_INVALID_STATE, cString::Format("JCom: No valid Maneuver could be received from JuryManeuverList!"));
            }
            feedback_tmp.ui32FeedbackStatus = currMan_tmp;

            RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedbackStruct, (void *) &feedback_tmp, m_pClock->GetStreamTime()));
        }

        else{
            /* maneuver list not loaded yet, so do nothing until it is loaded;
                     *  after successful loading, feedback will be transmitted to SCM */
            /* set flag for requested transmit of first maneuver to true */
            m_maneuverFirstTransmitrequested = tTrue;

            if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("JCom: first maneuver requested, but maneuver list not loaded yet!"));
        }
    }
    else if(inputAction.ui32Command == AC_JURY_MAN_GET_PREVIOUS){
        /* send back status to with previously executed maneuver */
        feedback_tmp.ui32FeedbackStatus = tmpPrevManeuverInt_fb;

        RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedbackStruct, (void *) &feedback_tmp, m_pClock->GetStreamTime()));

    }
    else if(inputAction.ui32Command == AC_JURY_MAN_FINISHED_GET_NEXT){
        /* before incrementing maneuver, save successfully executed maneuver as 'previous maneuver' */
        tUInt32 prevMan_tmp = getCurrentManeuverFromJuryList();
        if(prevMan_tmp == 0){
            RETURN_AND_LOG_ERROR_STR(ERR_INVALID_STATE, cString::Format("JCom: SavePreviousMan - No valid Maneuver could be received from JuryManeuverList!"));
        }
        setPreviousManeuver(prevMan_tmp);

        /* increment maneuver ID; returns retval < 0 in case of error, retval > 0 for completed list */
        tInt32 tmp_res = incrementManeuverID();
        if (tmp_res < 0){ // error occured
            RETURN_AND_LOG_ERROR_STR(ERR_END_OF_FILE,cString::Format("JCom: tried to increment maneuver ID, but failed."));
        }
        else if(tmp_res > 0){ // end of maneuverlist was reached
            if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("JCom: End of maneuverlist was reached! Switch to state COMPLETE."));
            m_bCompletedJuryList= tTrue;
            changeState(statecar_complete);
            feedback_tmp.ui32FeedbackStatus = FB_JURY_NO_MAN_REMAINING;

            RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedbackStruct, (void *) &feedback_tmp, m_pClock->GetStreamTime()));

        }
        else{ // ID could be incremented, tmp_res = 0 was returned
            /* get new/current maneuver from maneuverList */
            tUInt32 currMan_tmp = getCurrentManeuverFromJuryList();
            if(currMan_tmp == 0){
                RETURN_AND_LOG_ERROR_STR(ERR_INVALID_STATE, cString::Format("JCom: No valid Maneuver could be received from JuryManeuverList!"));
            }
            changeState(statecar_running);
            feedback_tmp.ui32FeedbackStatus = currMan_tmp;
            RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedbackStruct, (void *) &feedback_tmp, m_pClock->GetStreamTime()));

        }
    }
    else if(inputAction.ui32Command == AC_JURY_MAN_GET_FINISHING){
        /* get finishing maneuver from maneuverList, that means last one on the list (after state completed reached) */
        tUInt32 finishingMan_tmp = getFinishingManeuverFromJuryList();
        /* check if finishing maneuver (coded as tUInt32) can be received, that means != 0 */
        if(finishingMan_tmp == 0){
            RETURN_AND_LOG_ERROR_STR(ERR_INVALID_STATE, cString::Format("JCom: No valid finishing Maneuver could be received from JuryManeuverList, list might not be finished!"));
        }
        feedback_tmp.ui32FeedbackStatus = finishingMan_tmp;

        RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedbackStruct, (void *) &feedback_tmp, m_pClock->GetStreamTime()));


    }
    else if(inputAction.ui32Command == AC_JURY_SET_CARSTATE_ERROR_AND_RESET){
        /* call methods to change carState accordingly */
        changeState(statecar_error); // changes state, transmits to jury
        resetSection(); // after error, section has to be started from beginning
        TFeedbackStruct::Data feedback_tmp;
        feedback_tmp.ui8FilterId = F_JURY_COMMUNICATION;
        feedback_tmp.ui32FeedbackStatus = FB_JURY_CARSTATE_ERROR_SECTION_RESET;

        RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedbackStruct, (void *) &feedback_tmp, m_pClock->GetStreamTime()));


    }
    else if(inputAction.ui32Command == AC_JURY_WAIT_FOR_COMMAND_START){
        /* wait for the START command from jury, send feedback when received */
        if(m_startSignalreceived == tTrue){
            TFeedbackStruct::Data feedback_tmp;
            feedback_tmp.ui8FilterId = F_JURY_COMMUNICATION;
            feedback_tmp.ui32FeedbackStatus = FB_JURY_COMMAND_RECEIVED_START;

            RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedbackStruct, (void *) &feedback_tmp, m_pClock->GetStreamTime()));

            /* Reset sigsetFlagStartSignalReceivednal flag for next time */
            m_startSignalreceived = tFalse;
        }
        else{
            /* Do nothing, as Start signal has not been received yet;
                     *  -> waiting for Jury to send action_START */
            if(m_bDebugModeEnabled)  LOG_WARNING(cString::Format("JCom: waiting for jury to send 'action_START'!"));
            m_startSignalrequested = tTrue;
        }

    }
    RETURN_NOERROR;
}


tResult DriverModule::setPreviousManeuver(tUInt32 curManeuver){
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionPreviousManeuverAccess);
    tUInt32 tmp_prevManeuver = 0;
    switch (curManeuver){
    case FB_JURY_MAN_LEFT:
        tmp_prevManeuver = FB_JURY_PREVIOUS_MAN_LEFT;
        break;
    case FB_JURY_MAN_RIGHT:
        tmp_prevManeuver = FB_JURY_PREVIOUS_MAN_RIGHT;
        break;
    case FB_JURY_MAN_STRAIGHT:
        tmp_prevManeuver = FB_JURY_PREVIOUS_MAN_STRAIGHT;
        break;
    case FB_JURY_MAN_PARALLEL_PARKING:
        tmp_prevManeuver = FB_JURY_PREVIOUS_MAN_PARALLEL_PARKING;
        break;
    case FB_JURY_MAN_CROSS_PARKING:
        tmp_prevManeuver = FB_JURY_PREVIOUS_MAN_CROSS_PARKING;
        break;
    case FB_JURY_MAN_PULL_OUT_LEFT:
        tmp_prevManeuver = FB_JURY_PREVIOUS_MAN_PULL_OUT_LEFT;
        break;
    case FB_JURY_MAN_PULL_OUT_RIGHT:
        tmp_prevManeuver = FB_JURY_PREVIOUS_MAN_PULL_OUT_RIGHT;
        break;
    case FB_JURY_MAN_MERGE_LEFT:
        tmp_prevManeuver = FB_JURY_PREVIOUS_MAN_MERGE_LEFT;
        break;
    default:// FB_JURY_NO_MAN_REMAINING or anything else -> reset
        tmp_prevManeuver = FB_JURY_PREVIOUS_NO_MAN_EXISTING;
        break;
    }
    tmpPrevManeuverInt_fb = tmp_prevManeuver;
    RETURN_NOERROR;
}



/* Method for decoding given maneuver from Jury list and returning corresponding predefined tUint32 value */
tUInt32 DriverModule::decodeManeuverFromJuryList(cString tmp_juryManeuver, int ParkId){
    tUInt32 juryManeuverInt = 0;
    /* tInt CompareNoCase(const cString& strString) const; */
    if(tmp_juryManeuver.CompareNoCase("left") == 0)
    {
        /* action="left" */
        juryManeuverInt = FB_JURY_MAN_LEFT;
        if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: CurrentJuryManeuver: 'left'"));
    }
    else if(tmp_juryManeuver.CompareNoCase("right") == 0)
    {
        /* action="right" */
        juryManeuverInt = FB_JURY_MAN_RIGHT;
        if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: CurrentJuryManeuver: 'right'"));
    }
    else if(tmp_juryManeuver.CompareNoCase("straight") == 0)
    {
        /* action="straight" */
        juryManeuverInt = FB_JURY_MAN_STRAIGHT;
        if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: CurrentJuryManeuver: 'straight'"));
    }
    else if(tmp_juryManeuver.CompareNoCase("merge_left") == 0)
    {
        /* action="merge_left" */
        juryManeuverInt = FB_JURY_MAN_MERGE_LEFT;
        if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: CurrentJuryManeuver: 'merge_left'"));
    }
    else if(tmp_juryManeuver.CompareNoCase("parallel_parking") == 0)
    {
        /* action="parallel_parking" */
        juryManeuverInt = FB_JURY_MAN_PARALLEL_PARKING;
        if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: CurrentJuryManeuver: 'parallel_parking'"));
    }
    //	else if(tmp_juryManeuver.CompareNoCase("cross_parking") == 0)
    //	{
    //		/* action="cross_parking" */
    //		juryManeuverInt = FB_JURY_MAN_CROSS_PARKING;
    //		if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: CurrentJuryManeuver: 'cross_parking'"));
    //	}
    // --- handle cross parking with parking spot IDs
    else if(tmp_juryManeuver.StartsWith("cross_parking"))
    {
        // get the ID
        switch (ParkId)
        {
        case 0:
            juryManeuverInt = FB_JURY_CROSS_PARKING_SPOT_EIGHT;
            break;

        case 1:
            juryManeuverInt = FB_JURY_CROSS_PARKING_SPOT_SEVEN;
            break;

        case 2:
            juryManeuverInt = FB_JURY_CROSS_PARKING_SPOT_SIX;
            break;

        case 3:
            juryManeuverInt = FB_JURY_CROSS_PARKING_SPOT_FIVE;
            break;
        case 4:
            juryManeuverInt = FB_JURY_CROSS_PARKING_SPOT_FOUR;
            break;

        case 5:
            juryManeuverInt = FB_JURY_CROSS_PARKING_SPOT_THREE;
            break;

        case 6:
            juryManeuverInt = FB_JURY_CROSS_PARKING_SPOT_TWO;
            break;

        case 7:
            juryManeuverInt = FB_JURY_CROSS_PARKING_SPOT_ONE;
            break;

        }

        /* action="cross_parking" */

        if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: CurrentJuryManeuver: 'cross_parking' in parking spot ID %d", ParkId));
    }
    // ---
    else if(tmp_juryManeuver.CompareNoCase("pull_out_left") == 0)
    {
        /* action="pull_out_left" */
        juryManeuverInt = FB_JURY_MAN_PULL_OUT_LEFT;
        if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: CurrentJuryManeuver: 'pull_out_left'"));
    }
    else if(tmp_juryManeuver.CompareNoCase("pull_out_right") == 0)
    {
        /* action="pull_out_right */
        juryManeuverInt = FB_JURY_MAN_PULL_OUT_RIGHT;
        if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: CurrentJuryManeuver: 'pull_out_right'"));
    }
    else{
        LOG_ERROR(adtf_util::cString::Format("JCom: DecodeJuryManeuver: Maneuver could not be decoded, ERROR occurred!"));
    }

    return juryManeuverInt;
}


/* Method for returning the current maneuver from Jury list */
tUInt32 DriverModule::getCurrentManeuverFromJuryList(){
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionManeuverList);
    if (m_maneuverListLoaded == tFalse){
        LOG_ERROR( cString::Format("JCom: getCurrentManeuver -  ManeuverList not loaded, could not get next maneuver!"));
        return 0;
    }
    cString tmp_currJuryManeuver = " ";
    tUInt32 currentJuryManeuverInt = 0;
    int ParkId = -1;
    tmp_currJuryManeuver = maneuverToCString(m_sectorList[m_i16SectionListIndex].sector[m_i16ManeuverListIndex].action);
    if(m_sectorList[m_i16SectionListIndex].sector[m_i16ManeuverListIndex].action == 6)
    {
        ParkId = m_sectorList[m_i16SectionListIndex].sector[m_i16ManeuverListIndex].extra;
    }
    currentJuryManeuverInt = decodeManeuverFromJuryList(tmp_currJuryManeuver, ParkId);

    return currentJuryManeuverInt;
}


/* Method for returning the finishing maneuver from Jury list, that means the maneuver that was executed as the last maneuver on the list */
tUInt32 DriverModule::getFinishingManeuverFromJuryList(){
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionManeuverList);
    if (m_maneuverListLoaded == tFalse){
        LOG_ERROR( cString::Format("JCom: getFinishingManeuver -  ManeuverList not loaded, could not get finishing maneuver!"));
        return 0;
    }
    cString tmp_finishingJuryManeuver = " ";
    tUInt32 finishingJuryManeuverInt = 0;
    int ParkId = -1;
    /* check if the maneuver-list was actually completed */
    if (m_bCompletedJuryList == tTrue){
        /* after completing the maneuver list, the id is was NOT incremented anymore (avoiding index out of bounds) */
        tmp_finishingJuryManeuver =maneuverToCString(m_sectorList[m_i16SectionListIndex].sector[m_i16ManeuverListIndex].action);
        if(m_sectorList[m_i16SectionListIndex].sector[m_i16ManeuverListIndex].action == 6)
        {
            ParkId = m_sectorList[m_i16SectionListIndex].sector[m_i16ManeuverListIndex].extra;
        }
        finishingJuryManeuverInt = decodeManeuverFromJuryList(tmp_finishingJuryManeuver, ParkId);
    }

    return finishingJuryManeuverInt;
}



/* Method for changing the State of the car; After change, new status is immediately sent to jury;
 *  just used for 'startup', 'error' ,'ready' and 'complete' */
tResult DriverModule::changeState(stateCar newState)
{
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionCarStatus);
    // if state is the same do nothing
    if (m_CarState == newState && m_CarState != statecar_running) RETURN_NOERROR;

    // to secure the state is sent at least one time
    OnSendState(newState, m_i16CurrentManeuverID);

    // handle the timer depending on the state
    switch (newState)
    {
    case statecar_error:
        //        destroyTimer();
        LOG_WARNING(cString::Format("JCom: State ERROR reached."));
        break;
    case statecar_startup:
        //        destroyTimer();
        LOG_WARNING(cString::Format("JCom: State STARTUP reached."));
        break;
    case statecar_ready:
        //        createTimer();
        LOG_WARNING(cString::Format("JCom: State READY reached (ID %d).",m_i16CurrentManeuverID));
        break;
    case statecar_running:
        if (m_CarState!=statecar_ready)
            LOG_WARNING(cString::Format("JCom: Invalid state change to state 'RUNNING'. State 'READY' was not reached before. Nevertheless, change will be executed."));
        LOG_WARNING(cString::Format("JCom: State RUNNING reached."));
        break;
    case statecar_complete:
        //        destroyTimer();
        LOG_WARNING(cString::Format("JCom: State COMPLETE reached."));
        break;
    }

    m_CarState = newState;
    RETURN_NOERROR;
}

/* Methods increments the current maneuver ID after maneuver is successfully finished */
tInt32 DriverModule::incrementManeuverID()
{
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionManeuverList);
    tInt32 retval = -1;
    // check if list was successfully loaded during initialization
    if (m_i16ManeuverListIndex!=-1 && m_i16SectionListIndex!=-1)
    {
        retval = 0;
        // check if end of section is reached
        // if NOT reached:
        if (m_sectorList[m_i16SectionListIndex].sector.size()>tUInt(m_i16ManeuverListIndex+1))
        {
            // increment only maneuver index, as there are still maneuvers to process in current section
            m_i16ManeuverListIndex++;
            // increment the current ManeuverID, representing the global position in list (above sector borders)
            m_i16CurrentManeuverID++;
        }
        else //end of section is reached
        {
            // end of section was reached and another section is in list
            if (m_sectorList.size() >tUInt(m_i16SectionListIndex+1))
            {
                //reset maneuver index to zero and increment section list index, since new sector is entered now
                m_i16SectionListIndex++;
                m_i16ManeuverListIndex = 0; // start at beginning of sector, with maneuverIndex zero
                m_i16CurrentManeuverID++; // just incremented, since it represents global position (corresponding to ID)
                if (m_sectorList[m_i16SectionListIndex].sector[m_i16ManeuverListIndex].id !=m_i16CurrentManeuverID)
                {
                    LOG_ERROR("JCom: inconsistency in maneuverfile detected, IDs don't match. Please check the file!");
                }
            }
            else
            {
                /* End of maneuver list is reached, all sectors completed */
                if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("JCom: End of maneuverlist reached, cannot increment any more. List completed!"));
                retval = 1;
                /* Change carState to Completed, go to startup or Completed-maneuver in SCM! */
            }
        }
    }
    else
    {
        LOG_ERROR("JCom: could not set new maneuver ID because no maneuver list was loaded!");
    }

    if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: Increment Maneuver ID: Sectionindex is %d, Maneuverindex is %d, ID is %d",m_i16SectionListIndex,m_i16ManeuverListIndex,m_i16CurrentManeuverID));

    return retval;
}

/* Method for reseting section;
 *  After execution, the maneuver that has to be processed next is reset to the first maneuver in the current sector */
tResult DriverModule::resetSection()
{
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionManeuverList);
    //maneuver list index to zero, and current maneuver id to first element in list
    m_i16ManeuverListIndex=0;
    m_i16CurrentManeuverID = m_sectorList[m_i16SectionListIndex].sector[m_i16ManeuverListIndex].id;

    if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: Reset section: Sectionindex is %d, Maneuverindex is %d, ID is %d",m_i16SectionListIndex,m_i16ManeuverListIndex,m_i16CurrentManeuverID));

    RETURN_NOERROR;
}

/* Method for directly setting the maneuver ID;
 *  searches saved maneuverlist for provided 'maneuverId' and sets the sectionListindex and ManeuverListIndex accordingly */
tResult DriverModule::setManeuverID(tInt maneuverId)
{
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionManeuverList);
    //look for the right section id and write it to section combobox
    for(unsigned int i = 0; i < m_sectorList.size(); i++)
    {
        for(unsigned int j = 0; j < m_sectorList[i].sector.size(); j++)
        {
            if(maneuverId == m_sectorList[i].sector[j].id)
            {
                m_i16SectionListIndex = i;
                m_i16ManeuverListIndex = j;
                m_i16CurrentManeuverID = maneuverId;
                if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: Sectionindex is %d, Maneuverindex is %d, ID is %d",m_i16SectionListIndex,m_i16ManeuverListIndex,m_i16CurrentManeuverID));
                break;
            }
        }
    }
    RETURN_NOERROR;
}

///* Method for creating a timer with name m_hTimer with 0.5 seconds */
//tResult DriverModule::createTimer()
//{
//    //    // creates timer with 0.5 sec
//    //    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionTimerSetup);
//    //    // additional check necessary because input jury structs can be mixed up because every signal is sent three times
//    //    if (m_hTimer == NULL)
//    //    {
//    //        m_hTimer = m_pKernel->TimerCreate(tTimeStamp(0.5*1000000), 0, static_cast<IRunnable*>(this),
//    //                                        NULL, NULL, 0, 0, adtf_util::cString::Format("%s.timer", OIGetInstanceName()));
//    //    }
//    //    else
//    //    {
//    //        LOG_ERROR("JCom: Timer is already running. Unable to create a new one.");
//    //    }
//    RETURN_NOERROR;
//}

///* Method for destroying the timer m_hTimer*/
//tResult DriverModule::destroyTimer()
//{
//    //    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionTimerSetup);
//    //    //destroy timer
//    //    if (m_hTimer != NULL)
//    //    {
//    //        tResult nResult = m_hTimer.Destroy();
//    //        if (IS_FAILED(nResult))
//    //        {
//    //            LOG_ERROR("JCom: Unable to destroy the timer.");
//    ////            THROW_ERROR(nResult);
//    //        }
//    //        m_hTimer = NULL;
//    //    }
//    //    //check if handle for some unknown reason still exists
//    //    else
//    //    {
//    //        //LOG_WARNING("JCom: Timer handle not set, but I should destroy the timer. Try to find a timer with my name.");
//    //        tHandle hFoundHandle = m_pKernel->FindHandle(adtf_util::cString::Format("%s.timer", OIGetInstanceName()));
//    //        if (hFoundHandle)
//    //        {
//    //            tResult nResult = hFoundHandle.Destroy();
//    //            if (IS_FAILED(nResult))
//    //            {
//    //                LOG_ERROR("JCom: Unable to destroy the specified timer.");
//    ////                THROW_ERROR(nResult);
//    //            }
//    //        }
//    //    }

//    RETURN_NOERROR;
//}


