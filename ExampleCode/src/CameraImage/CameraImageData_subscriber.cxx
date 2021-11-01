/** ------------------------------------------------------------------------
 * CameraImageData_subscriber.cxx
 * Subscribes to fixed-frame data arrays of type "CameraImageData" defined in
 * automotive.idl file, with options for "Flat Data" optimizations to be
 * included (note: "Zero Copy" optimizations do not require changes in this file.
 *
 * The data is generated from an LFSR function in the publisher, and can optionally
 * be verified here in the subscriber.  Timestamps have also been included in the
 * published packets, these are used here in the subscriber to measure the transit
 * times of each optimization mode.
 *
 * TO USE THE DIFFERENT OPTIMIZATION MODES(FlatData, ZeroCopy, or both)
 *  1. Uncomment the required #define DDS_LARGE_DATA_*
 *     (in this file (below) and in the complementary publisher.cxx file)
 *  2. Uncomment the supporting lines in the automotive.idl file.
 *  3. Rebuild all.
 *
 * (c) 2005-2019 Copyright, Real-Time Innovations, Inc.  All rights reserved.
 * RTI grants Licensee a license to use, modify, compile, and create derivative
 * works of the Software.  Licensee has the right to distribute object form
 * only for use with RTI products.  The Software is provided 'as is', with no
 * arranty of any type, including any warranty for fitness for any purpose. RTI
 * is under no obligation to maintain or support the Software.  RTI shall not
 * be liable for any incidental or consequential damages arising out of the
 * use or inability to use the software.
 **/
#include <stdio.h>
#include <stdlib.h>
#include "dataObject.h"
#include "Utils.h"
#include "automotive.h"
#include "automotiveSupport.h"
#include "ndds/ndds_cpp.h"

#ifdef _WIN32
#include <Windows.h>
#endif  // def _WIN32

// uncomment if using FLAT_DATA
//#define DDS_LARGE_DATA_FLAT_DATA

#ifdef _WIN32
// returns time in nanoSeconds
uint64_t UtcNowPrecise()
{
    const uint64_t OA_ZERO_TICKS = 94353120000000000; //12/30/1899 12:00am in ticks
    FILETIME ft;
    GetSystemTimePreciseAsFileTime(&ft);
    ULARGE_INTEGER dt; //needed to avoid alignment faults
    dt.LowPart = ft.dwLowDateTime;
    dt.HighPart = ft.dwHighDateTime;
    return ((uint64_t)(dt.QuadPart - OA_ZERO_TICKS) * 100);
}
#else
// Returns time in nanoSeconds
uint64_t UtcNowPrecise()
{
    timespec tsNow;
    clock_gettime(CLOCK_MONOTONIC, &tsNow);
    uint64_t tNow = ((uint64_t)tsNow.tv_sec * 1000000000) + tsNow.tv_nsec;
    return tNow;
}
#endif  // def _WIN32


class CameraImage_CameraImageDataListener : public DDSDataReaderListener {
  public:
    CameraImage_CameraImageDataListener(DDSDomainParticipant *participant) :
        participant_(participant) { }

    virtual void on_requested_deadline_missed(
        DDSDataReader* /*reader*/,
        const DDS_RequestedDeadlineMissedStatus& /*status*/) {}

    virtual void on_requested_incompatible_qos(
        DDSDataReader* /*reader*/,
        const DDS_RequestedIncompatibleQosStatus& /*status*/) {}

    virtual void on_sample_rejected(
        DDSDataReader* /*reader*/,
        const DDS_SampleRejectedStatus& /*status*/) {}

    virtual void on_liveliness_changed(
        DDSDataReader* /*reader*/,
        const DDS_LivelinessChangedStatus& /*status*/) {}

    virtual void on_sample_lost(
        DDSDataReader* /*reader*/,
        const DDS_SampleLostStatus& /*status*/) {}

    virtual void on_subscription_matched(
        DDSDataReader* /*reader*/,
        const DDS_SubscriptionMatchedStatus& /*status*/) {}

    virtual void on_data_available(DDSDataReader* reader);

    private:
        DDSDomainParticipant *participant_;
};

/** ----------------------------------------------------------------
 * calcAndPrintTransitTime()
 * given a send timestamp and a receive timestamp, calculate and print
 * the timing stats and sample counter.
 **/
void calcAndPrintTransitTime(DDS_Time_t &tSend, DDS_Time_t &tReceive)
{
    static double tMin = 1000; // Any number larger than the latency
    static double tMax = 0;
    static double tSum = 0;
    static uint32_t tSampleCount = 0;

    double tSend_s    = (double)tSend.sec + 1e-9*((double)tSend.nanosec);
    double tReceive_s = (double)tReceive.sec + 1e-9*((double)tReceive.nanosec);
    static double tStart_s = 0;
    if ( tStart_s == 0 ) {
        tStart_s = tReceive_s;
    }

    double tDelta = tReceive_s - tSend_s;
    if (tDelta < tMin) { tMin = tDelta; }
    if (tDelta > tMax) { tMax = tDelta; }
    tSum += tDelta;
    tSampleCount++;
    double tAvg = tSum / tSampleCount;
    fprintf(stdout, "CameraImageSub: t=%d.%d, recv=%u, lat: %2.3f (min: %2.3f, max: %2.3f, avg: %2.3f), BW: %3.3f MB/s\n",
        tReceive.sec%10000,
        tReceive.nanosec/1000000,
        tSampleCount,
        tDelta, tMin, tMax, tAvg, 
        1e-6*((double)(tSampleCount*MAX_IMAGE_SIZE))/(tReceive_s-tStart_s)
        );
    return;
}

#ifdef OLD
void calcAndPrintTransitTime(uint64_t tSend, uint64_t tReceive)
{
    static uint64_t tMin = (uint64_t)-1;
    static uint64_t tMax = 0;
    static uint64_t tSum = 0;
    static uint32_t tSampleCount = 0;

    double tReceive_s = ((double)tReceive)*1e-9;
    static double tStart_s = 0;
    if ( tStart_s == 0 ) {
        tStart_s = tReceive_s;
    }

    uint32_t tDelta = (uint32_t)(tReceive - tSend);
    if (tDelta < tMin)
        tMin = tDelta;
    if (tDelta > tMax)
        tMax = tDelta;
    tSum += tDelta;
    tSampleCount++;
    double tAvg = (((double)tSum / tSampleCount) / 1000000000);
    fprintf(stdout, "CameraImageSub: t=%lu.%lu, N=%u, t: %2.6f (min: %2.6f max: %2.3f, avg: %2.3f) size: %u (%3.3f MB/s)"
                    ", bandwidth: %3.3f MB/s\n",
        tReceive/1000000000, (tReceive%1000000000)/10000000,
        tSampleCount,
        ((double)tDelta / 1000000000),
        ((double)tMin / 1000000000),
        ((double)tMax / 1000000000),
        tAvg, MAX_IMAGE_SIZE,
        ((double)MAX_IMAGE_SIZE / tAvg) / 1000000,
        1e-6*((double)(tSampleCount*MAX_IMAGE_SIZE))/(tReceive_s-tStart_s)
        );
    return;
}
#endif

/** ----------------------------------------------------------------
 * checkLfsrDataInArray()
 * Check the u32 data in array for correct LFSR sequence, based on
 * value in position [0].  Scan until MAX_IMAGE_SIZE
 * returns true if no error
 **/
bool checkLfsrDataInArray(uint32_t *lfsrArray)
{
    uint32_t lfsr = *lfsrArray;
    for (int j = 4; j < MAX_IMAGE_SIZE; j += 4) {
        lfsrArray++;
        lfsr = (lfsr >> 1) ^ (-(lfsr & 1u) & 0xD0000001u);
        if (lfsr != *lfsrArray) {
            fprintf(stderr, "Image data received != sent [%08x != %08x] at %s:%d\n", *lfsrArray, lfsr, __FILE__, __LINE__);
            return false;
        }
    }
    return true;
}

/** ----------------------------------------------------------------
 * on_data_available()
 * Data listener -- called when new data has been received.
 **/
void CameraImage_CameraImageDataListener::on_data_available(DDSDataReader* reader)
{
    CameraImage_CameraImageDataDataReader *CameraImage_CameraImageData_reader = NULL;
    CameraImage_CameraImageDataSeq data_seq;
    DDS_SampleInfoSeq info_seq;
    DDS_ReturnCode_t retcode;
    int i;

    CameraImage_CameraImageData_reader = CameraImage_CameraImageDataDataReader::narrow(reader);
    if (CameraImage_CameraImageData_reader == NULL) {
        fprintf(stderr, "DataReader narrow error(%s:%d)\n", __FILE__, __LINE__);
        return;
    }

    retcode = CameraImage_CameraImageData_reader->take(
        data_seq, info_seq, DDS_LENGTH_UNLIMITED,
        DDS_ANY_SAMPLE_STATE, DDS_ANY_VIEW_STATE, DDS_ANY_INSTANCE_STATE);

    if (retcode == DDS_RETCODE_NO_DATA) {
        return;
    } else if (retcode != DDS_RETCODE_OK) {
        fprintf(stderr, "take error %d (%s:%d)\n", retcode, __FILE__, __LINE__);
        return;
    }

    for (i = 0; i < data_seq.length(); ++i) {
        if (info_seq[i].valid_data) {
            // get the current time value
            //uint64_t tReceive = UtcNowPrecise();            

#ifdef DDS_LARGE_DATA_FLAT_DATA
            // get the root to the Flat Data sample
            CameraImage_CameraImageDataOffset sample_root = data_seq[i].root();

            // get the send-timestamp value from the received packet
            // uint64_t tSend = (((uint64_t)sample_root.sec_()) * 1000000000) + sample_root.nanosec_();
            DDS_Time_t tSend;
            tSend.sec = sample_root.sec_();
            tSend.nanosec = sample_root.nanosec_();

            // verify the contents of the received data (optional)
            auto data_array = rti::flat::plain_cast(sample_root.data());
            uint32_t *rcvBuffer = (uint32_t *)&data_array[0];
            checkLfsrDataInArray(rcvBuffer);

#else  // ndef DDS_LARGE_DATA_FLAT_DATA
            // get the send-timestamp value from the received packet
            // uint64_t tSend = (((uint64_t)data_seq[i].sec_) * 1000000000) + data_seq[i].nanosec_;
            DDS_Time_t tSend;
            tSend.sec = data_seq[i].sec_;
            tSend.nanosec = data_seq[i].nanosec_;

            // verify the contents of the received data (optional)
            uint32_t *rcvBuffer = (uint32_t *) &data_seq[i].data[0];
            checkLfsrDataInArray(rcvBuffer);

#endif  // ndef DDS_LARGE_DATA_FLAT_DATA
            // print the transit timing
            calcAndPrintTransitTime(tSend, info_seq[i].reception_timestamp);
        }
    }

    retcode = CameraImage_CameraImageData_reader->return_loan(data_seq, info_seq);
    if (retcode != DDS_RETCODE_OK) {
        fprintf(stderr, "return loan error %d (%s:%d)\n", retcode, __FILE__, __LINE__);
    }
}

/* Delete all entities */
static int subscriber_shutdown(
    DDSDomainParticipant *participant)
{
    DDS_ReturnCode_t retcode;
    int status = 0;

    if (participant != NULL) {
        retcode = participant->delete_contained_entities();
        if (retcode != DDS_RETCODE_OK) {
            fprintf(stderr, "delete_contained_entities error %d (%s:%d)\n", retcode, __FILE__, __LINE__);
            status = -1;
        }

        retcode = DDSTheParticipantFactory->delete_participant(participant);
        if (retcode != DDS_RETCODE_OK) {
            fprintf(stderr, "delete_participant error %d (%s:%d)\n", retcode, __FILE__, __LINE__);
            status = -1;
        }
    }

    /* RTI Connext provides the finalize_instance() method on
    domain participant factory for people who want to release memory used
    by the participant factory. Uncomment the following block of code for
    clean destruction of the singleton. */
    /*

    retcode = DDSDomainParticipantFactory::finalize_instance();
    if (retcode != DDS_RETCODE_OK) {
        fprintf(stderr, "finalize_instance error %d (%s:%d)\n", retcode, __FILE__, __LINE__);
        status = -1;
    }
    */
    return status;
}

/** ----------------------------------------------------------------
* subscriber_main()
* create the topic subscriber & receive data.
**/extern "C" int subscriber_main(int sample_count)
{
    DDSDomainParticipant *participant = NULL;
    CameraImage_CameraImageDataListener *reader_listener = NULL;
    DDSDataReader *reader = NULL;
    int count = 0;
    DDS_Duration_t receive_period = {4,0};
    int status = 0;

    /* Get the configurtion properties from the camera_image.properties file */
    PropertyUtil* prop = new PropertyUtil("camera_image.properties");

    long time = prop->getLongProperty("config.pubInterval");
    if(time) {
        receive_period.sec = time / 1000;
        receive_period.nanosec = (time % 1000) * 1000 * 1000;
    }

    std::string topicName = prop->getStringProperty("topic.Sensor");
    if (topicName == "") {
        printf("No topic name specified (%s:%d)\n", __FILE__, __LINE__);
        return -1;
    }
    std::string qosLibrary = prop->getStringProperty("qos.Library");
    if (qosLibrary == "") {
        printf("No QoS Library specified (%s:%d)\n", __FILE__, __LINE__);
        return -1;
    }
#ifdef DDS_LARGE_DATA_FLAT_DATA
    std::string qosProfile = prop->getStringProperty("qos.XCDR2Profile");
#else
    std::string qosProfile = prop->getStringProperty("qos.Profile");
#endif  // def DDS_LARGE_DATA_FLAT_DATA
    if (qosProfile == "") {
        printf("No QoS Profile specified (%s:%d)\n", __FILE__, __LINE__);
        return -1;
    }

    if ( DDSTheParticipantFactory->register_type_support(
            CameraImage_CameraImageDataTypeSupport::register_type, 
            CameraImage_CameraImageDataTypeSupport::get_type_name())
        != DDS_RETCODE_OK) {

        fprintf(stderr, "register_type_support error(%s:%d)\n", __FILE__, __LINE__);
        subscriber_shutdown(participant);
        return -1;
    }


    /* To customize participant QoS, use the configuration file USER_QOS_PROFILES.xml */
    // create participant
	participant = DDSTheParticipantFactory->create_participant_from_config("automotive_lib::rearViewCameraObserver");
    if (participant == NULL) {
        fprintf(stderr, "create_participant_from_config error(%s:%d)\n", __FILE__, __LINE__);
        subscriber_shutdown(participant);
        return -1;
    }
    else {
        fprintf(stderr, "Created Participant from config: \"automotive_lib::rearViewCameraObserver\"\n");
    }

    reader = participant->lookup_datareader_by_name("CameraSubscriber::CameraReader");
    if (reader == NULL) {
        fprintf(stderr, "lookup_datareader_by_name error(%s:%d)\n", __FILE__, __LINE__);
        subscriber_shutdown(participant);
        return -1;
    }
    reader_listener = new CameraImage_CameraImageDataListener(participant);
    reader->set_listener(reader_listener, DDS_STATUS_MASK_ALL & ~~DDS_DATA_AVAILABLE_STATUS);

    printf("Start Receiving\n");
    /* Main loop */
    for (count=0; (sample_count == 0) || (count < sample_count); ++count) {
        NDDSUtility::sleep(receive_period);
    }

    /* Delete all entities */
    status = subscriber_shutdown(participant);
    delete reader_listener;

    return status;
}

int main(int argc, char *argv[])
{
    int sample_count = 0; /* infinite loop */

    if (argc >= 2) {
        sample_count = atoi(argv[1]);
    }

    /* Uncomment this to turn on additional logging
    NDDSConfigLogger::get_instance()->
    set_verbosity_by_category(NDDS_CONFIG_LOG_CATEGORY_API,
    NDDS_CONFIG_LOG_VERBOSITY_STATUS_ALL);
    */

    return subscriber_main(sample_count);
}

