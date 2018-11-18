/**
 *
 * ADTF Template Project
 *
 * @file
 * Copyright &copy; Audi Electronics Venture GmbH. All rights reserved
 *
 * $Author: voigtlpi $
 * $Date: 2017-02-24 23:49:43 +0100 (Fri, 24 Feb 2017) $
 * $Revision: 60174 $
 *
 * @remarks
 *
 */
#include <adtf3.h>
#include "synchronizer_filter.h"

// this macro creates the plugin DLL or shared object entry methods
// and provides the filter class via its class factory
ADTF_PLUGIN("TimeStamp Synchronizer Plugin",
            SynchronizerFilter);

SynchronizerFilter::SynchronizerFilter()
{
    LOG_INFO("1");
    RegisterPropertyVariable("queue_transfer_start_timeout", m_nQueueTransferStartTimeout);
    RegisterPropertyVariable("queue_transfer_end_timeout", m_nQueueTransferEndTimeout);
    RegisterPropertyVariable("synchronous_queue_processing111", m_bSynchronousQueueProcessing);

    // register a public runner that can be connected with a Timer Runner.
    adtf::ucom::object_ptr<adtf::streaming::IRunner> pRunner = adtf::ucom::make_object_ptr<adtf::streaming::cRunner>("process_queue", [&](tTimeStamp) -> tResult
    {
        // double check we have all available items in our queue
        // if connections are not synchronous and do not provide triggers
        for (auto& sStream: m_oStreams)
        {
            sStream.second->oReader.ReadAllAvailableItems();
        }

        // check if we need to forward some items
        CheckQueue();
        RETURN_NOERROR;
    });

    RegisterRunner(pRunner);
}

// this method is called whenever a new input pin is requested.
tResult SynchronizerFilter::RequestPin(const tChar* strName,
                                            const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType,
                                            adtf::ucom::ant::iobject_ptr<adtf::streaming::ant::IInPin>& pInPin)
{LOG_INFO("2");
    // create a new stream that holds our reader and writer
    auto& sStream = m_oStreams.emplace(strName, std::make_shared<tStream>(*this)).first->second;
    // initialize the reader
    RETURN_IF_FAILED(adtf::streaming::make_sample_reader(sStream->oReader, strName, pType));
    // and create an according pin
    RETURN_IF_FAILED(adtf::filter::filter_create_pin(*this, sStream->oReader, pInPin));

    // we create an internal trigger pipe in order to enqueue triggers as well
    adtf::streaming::create_inner_pipe(*this, adtf::util::cString::Format("%s_data_trigger", strName), strName, [&](tTimeStamp /* tmTime */) -> tResult
    {
        // just in case this was not a PushRead synchronous connection.
        sStream->oReader.ReadAllAvailableItems();

        // an empty stream item signals a trigger
        InsertQueueItem(adtf::streaming::cStreamItem(), *sStream);

        if (m_bSynchronousQueueProcessing)
        {
            CheckQueue();
        }

        RETURN_NOERROR;
    });

    RETURN_NOERROR;

}

// this method is called whenever a new output pin is requested.
tResult SynchronizerFilter::RequestPin(const tChar* strName,
                                            const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType,
                                            adtf::ucom::ant::iobject_ptr<adtf::streaming::ant::IOutPin>& pOutPin)
{LOG_INFO("3");
    adtf::util::cString strStreamName(strName);
    strStreamName = strStreamName.SubString(0, strStreamName.RFind('_'));
    auto itStream = m_oStreams.find(strStreamName);
    if (itStream == m_oStreams.end())
    {
        // this is not nice, but currently the plugin description generator will not recognise our
        // support for dynamic output pins otherwise
        LOG_WARNING("no input pin for %s found, ignoring", strName);
        RETURN_NOERROR;
    }

    // initalize the writer
    RETURN_IF_FAILED(adtf::streaming::make_sample_writer(itStream->second->oWriter, strName, pType));
    // and create a pin.
    return adtf::filter::filter_create_pin(*this, itStream->second->oWriter, pOutPin);
}

tVoid SynchronizerFilter::InsertQueueItem(adtf::streaming::cStreamItem&& oStreamItem,
                                               tStream& oStream)
{LOG_INFO("4");
    std::lock_guard<std::mutex> oGuard(m_oQueueMutex);

    // stream types and trigger get the timestamp of the last sample
    tTimeStamp tmTimeStamp = oStream.tmLastSampleTimeStamp;

    if (oStreamItem.GetType() == adtf::streaming::IStreamItem::tType::Sample)
    {
        adtf::ucom::object_ptr<const adtf::streaming::ant::ISample> pSample;
        if (IS_OK(oStreamItem.GetSample(pSample)))
        {
            tmTimeStamp = pSample->GetTime();
            oStream.tmLastSampleTimeStamp = tmTimeStamp;
        }
    }

    // we always insert new items sorted
    m_oQueue.emplace(std::upper_bound(m_oQueue.begin(),
                                      m_oQueue.end(),
                                      tmTimeStamp,
                                      [](tTimeStamp tmTime, tQueueItem& sItem)
                                      {
                                          return tmTime < sItem.tmTimeStamp;
                                      }),
                     tmTimeStamp, std::move(oStreamItem), oStream);
}

tVoid SynchronizerFilter::CheckQueue()
{LOG_INFO("5");
    std::lock_guard<std::mutex> oGuard(m_oQueueMutex);

    if (m_oQueue.empty())
    {
        return;
    }

    // if the queue has not reached the transfer start timeout we do nothing
    if (m_oQueue.back().tmTimeStamp - m_oQueue.front().tmTimeStamp < static_cast<tInt64>(m_nQueueTransferStartTimeout))
    {
        return;
    }

    // otherwise forward all items older than the transfer end timeout
    while (!m_oQueue.empty() &&
           m_oQueue.back().tmTimeStamp - m_oQueue.front().tmTimeStamp >= m_nQueueTransferEndTimeout)
    {
        ForwardItem(m_oQueue.front());
        m_oQueue.pop_front();
    }
}

tVoid SynchronizerFilter::ForwardItem(SynchronizerFilter::tQueueItem& sItem)
{LOG_INFO("6");
    switch (sItem.oStreamItem.GetType())
    {
        case adtf::streaming::ant::IStreamItem::tType::Sample:
        {
            adtf::ucom::object_ptr<const adtf::streaming::ant::ISample> pSample;
            if (IS_OK(sItem.oStreamItem.GetSample(pSample)))
            {
                sItem.pStream->oWriter << pSample;
            }
            break;
        }

        case adtf::streaming::ant::IStreamItem::tType::StreamType:
        {
            adtf::ucom::object_ptr<const adtf::streaming::ant::IStreamType> pType;
            if (IS_OK(sItem.oStreamItem.GetStreamType(pType)))
            {
                sItem.pStream->oWriter << pType;
            }
            break;
        }

        default:
        {
            sItem.pStream->oWriter << adtf::streaming::trigger;
            break;
        }
    }
}
