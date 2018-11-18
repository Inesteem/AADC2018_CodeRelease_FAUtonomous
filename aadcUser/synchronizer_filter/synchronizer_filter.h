/**
 *
 * ADTF Template Project Filter.
 *
 * @file
 * Copyright &copy; Audi Electronics Venture GmbH. All rights reserved
 *
 * $Author: VG8D3AW $
 * $Date: 2016-11-25 11:48:27 +0100 (Fri, 25 Nov 2016) $
 * $Revision: 59325 $
 *
 * @remarks
 *
 */
#pragma once

#include <deque>

//*************************************************************************************************
#define CID_SYNCHRONIZER_FILTER "sync_filter.filter.user.aadc.cid"

// we use the cDynamicFilter base class, as it export the IDynamicDataBinding interface for us
class SynchronizerFilter: public adtf::filter::cDynamicFilter
{
    public:
        // give our filter a name and class id
        ADTF_CLASS_ID_NAME(SynchronizerFilter, CID_SYNCHRONIZER_FILTER, "TimeStamp Synchronizer");

    public:
        SynchronizerFilter();

        // implement the method that is called when a new input pin is required
        tResult RequestPin(const tChar* strName,
                           const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType,
                           adtf::ucom::ant::iobject_ptr<adtf::streaming::ant::IInPin>& pInPin) override;

        // implement the method that is called when a new output pin is required
        tResult RequestPin(const tChar* strName,
                           const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType,
                           adtf::ucom::ant::iobject_ptr<adtf::streaming::ant::IOutPin>& pOutPin) override;    

    public:
        // import required methods in order for adtf::streaming::create_inner_pipe to work.
        using cRuntimeBehaviour::RegisterRunner;
        using cRuntimeBehaviour::RegisterInnerPipe;

    private:

        // we use an cExternalQueueSampleReader in order to grab the
        // stream items and put them in the queue. This class is a proxy between the sample streams
        // and the queue within the filter.
        struct tStream: public adtf::streaming::ISampleReaderQueue
        {
            tStream(SynchronizerFilter& oFilter):
                m_oFilter(oFilter)
            {
                oReader.RegisterExternalQueue(this);
            }

            tResult Push(const adtf::streaming::IStreamItem& oStreamItem, tTimeStamp /* tmTime */) override
            {
                m_oFilter.InsertQueueItem(oStreamItem, *this);
                RETURN_NOERROR;
            }

            tVoid Clear() override
            {
            }

            tResult Pop(adtf::streaming::IStreamItem& /* oStreamItem */) override
            {
                RETURN_ERROR(ERR_NOT_SUPPORTED);
            }

            adtf::streaming::cSampleWriter oWriter;
            adtf::streaming::cExternalQueueSampleReader oReader;

            // this will store the time stamp from the last sample so that we can use it for types
            // or triggers, which do not have a timestamp of their own. In our context of types and
            // triggers only ordering is relevant.
            tTimeStamp tmLastSampleTimeStamp = 0;
            SynchronizerFilter& m_oFilter;
        };

        // gcc 5.4 reqires a copy constructor for std::map::emplace, so we need to use a
        // shared_ptr as a workaround
        std::map<adtf::util::cString, std::shared_ptr<tStream>> m_oStreams;

        struct tQueueItem
        {
            tQueueItem(tTimeStamp tmTimeStamp,
                       const adtf::streaming::IStreamItem& oStreamItem,
                       tStream& oStream):
                tmTimeStamp(tmTimeStamp),
                oStreamItem(oStreamItem),
                pStream(&oStream)
            {
            }

            tTimeStamp tmTimeStamp;
            adtf::streaming::cStreamItem oStreamItem;
            tStream* pStream;
        };

        std::mutex m_oQueueMutex;
        std::deque<tQueueItem> m_oQueue;

        adtf::base::property_variable<tInt64> m_nQueueTransferStartTimeout = 20000;
        adtf::base::property_variable<tInt64> m_nQueueTransferEndTimeout = 10000;
        adtf::base::property_variable<tBool> m_bSynchronousQueueProcessing = tFalse;

    private:
        tVoid InsertQueueItem(adtf::streaming::cStreamItem&& oStreamItem, tStream& oStream);
        tVoid CheckQueue();
        tVoid ForwardItem(tQueueItem& sItem);
};

