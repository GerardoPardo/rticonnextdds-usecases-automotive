/** ------------------------------------------------------------------------
 * CameraImageData_publisher.cxx
 * Publishes fixed-frame data arrays of type "CameraImageData" defined in
 * automotive.idl file, with options for "Flat Data" and/or "Zero Copy"
 * optimizations to be included.
 * Parameters (such as publish rate) are set in camera_image.properties file.
 *
 * The data is generated from an LFSR function, with optional verification
 * at the receiving end.   Timestamps are included to measure the transit
 * times of each optimization mode.
 *
 * TO USE THE DIFFERENT OPTIMIZATION MODES(FlatData, ZeroCopy, or both)
 *  1. Uncomment the required #define DDS_LARGE_DATA_*
 *     (in this file (below) and in the complementary subscriber.cxx file)
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
#include <Windows.h>    // for timestamp/timing
#else   // Linux
#include <time.h>       // for timestamp/timing
#endif  // def _WIN32

/* uncomment one|other|both|neither of the following for
   FLAT_DATA | ZERO_COPY | FLAT and ZERO | (standard DDS) speed optimizations */
//#define DDS_LARGE_DATA_FLAT_DATA
//#define DDS_LARGE_DATA_ZERO_COPY

// seed for LFSR used for striping the data array
#define LFSR_SEED   (0x55555555)
uint32_t ddsId = 1;
uint32_t lfsr = LFSR_SEED;
uint32_t seqNum = 1;


#ifdef _WIN32
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

/* Delete all entities */
static int publisher_shutdown(
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

    /* RTI Connext provides finalize_instance() method on
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

#ifdef DDS_LARGE_DATA_FLAT_DATA
/** ----------------------------------------------------------------
 * build_data_sample()
 * Fill the sample's data array with LFSR data, then add a current
 * timestamp to measure the transfer time at the subscriper.
 * This uses a plain_cast to speed the fill of the sample array.
 * returns true
 **/
bool build_data_sample(CameraImage_CameraImageDataBuilder& builder, DDSDomainParticipant *participant)
{
    // Build the CameraImage_CameraImageData data sample
    builder.add_id(ddsId);
    builder.add_seqnum(seqNum++);
    auto data_offset = builder.add_data();
    auto data_array = rti::flat::plain_cast(data_offset);
    for (int i = 0; i < MAX_IMAGE_SIZE; i += 4) {
        uint32_t *lfsrVal = (uint32_t *)&data_array[i];
        *lfsrVal = lfsr;
        lfsr = (lfsr >> 1) ^ (-(lfsr & 1u) & 0xD0000001u);
    }
    // uint64_t tNow = UtcNowPrecise();
    // builder.add_sec_(tNow / 1000000000);
    // builder.add_nanosec_(tNow % 1000000000);

    DDS_Time_t time_now;
    participant->get_current_time(time_now);
    builder.add_sec_(time_now.sec);
    builder.add_nanosec_(time_now.nanosec);

    return true;
}
#endif 	// DDS_LARGE_DATA_FLAT_DATA

/** ----------------------------------------------------------------
 * publisher_main()
 * create the topic publisher & send data.
 **/
extern "C" int publisher_main(int sample_count)
{
    DDSDomainParticipant *participant = NULL;
    CameraImage_CameraImageDataDataWriter * CameraImage_CameraImageData_writer = NULL;
    CameraImage_CameraImageData *instance = NULL;
    DDS_ReturnCode_t retcode;
    DDS_InstanceHandle_t instance_handle = DDS_HANDLE_NIL;
    int count = 0;
    DDS_Duration_t send_period = {4,0};

    /* Get the configurtion properties from the camera_image.properties file */
    PropertyUtil* prop = new PropertyUtil("camera_image.properties");

    long time = prop->getLongProperty("config.pubInterval");
    if (time) {
        send_period.sec = time / 1000;
        send_period.nanosec = (time % 1000) * 1000 * 1000;
    }
    printf("CameraDataPub: publicationInterval = %ld millisec\n", time);

    ddsId    = (uint32_t) prop->getLongProperty("config.ddsId");
    if(ddsId == 0) ddsId = 404;

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
#endif
    if (qosProfile == "") {
        printf("No QoS Profile specified (%s:%d)\n", __FILE__, __LINE__);
        return -1;
    }

    if ( DDSTheParticipantFactory->register_type_support(
            CameraImage_CameraImageDataTypeSupport::register_type, 
            CameraImage_CameraImageDataTypeSupport::get_type_name())
        != DDS_RETCODE_OK) {

        fprintf(stderr, "register_type_support error(%s:%d)\n", __FILE__, __LINE__);
        publisher_shutdown(participant);
        return -1;
    }


    /* To customize participant QoS, use the configuration file USER_QOS_PROFILES.xml */
    // create participant
	participant = DDSTheParticipantFactory->create_participant_from_config("automotive_lib::rearViewCamera");
    if (participant == NULL) {
        fprintf(stderr, "create_participant_from_config error(%s:%d)\n", __FILE__, __LINE__);
        publisher_shutdown(participant);
        return -1;
    }
    else {
        fprintf(stderr, "Created Participant from config: \"automotive_lib::rearViewCamera\"\n");
    }

    CameraImage_CameraImageData_writer = CameraImage_CameraImageDataDataWriter::narrow(
        participant->lookup_datawriter_by_name("CameraPublisher::CameraWriter"));

    if (CameraImage_CameraImageData_writer == NULL) {
        fprintf(stderr, "DataWriter narrow error(%s:%d)\n", __FILE__, __LINE__);
        publisher_shutdown(participant);
        return -1;
    }

#ifndef DDS_LARGE_DATA_FLAT_DATA
#ifndef DDS_LARGE_DATA_ZERO_COPY
    /* Create data sample for writing */
    instance = CameraImage_CameraImageDataTypeSupport::create_data();
    if (instance == NULL) {
        fprintf(stderr, "CameraImage_CameraImageDataTypeSupport::create_data error(%s:%d)\n", __FILE__, __LINE__);
        publisher_shutdown(participant);
        return -1;
    }

    // CameraImageData type uses a @key; init and register it here
    instance->id = ddsId;
    instance_handle = CameraImage_CameraImageData_writer->register_instance(*instance);

#endif	// ndef DDS_LARGE_DATA_ZERO_COPY
#endif	// ndef DDS_LARGE_DATA_FLAT_DATA

    printf("start sending\n");

    DDS_Time_t time_begin;
    participant->get_current_time(time_begin);
    double time_begin_s  = time_begin.sec + 1e-9*time_begin.nanosec;
    double send_period_s = send_period.sec + 1e-9*send_period.nanosec;

    /* Main loop */
    for (count=0; (sample_count == 0) || (count < sample_count); ++count) {
#ifdef DDS_LARGE_DATA_FLAT_DATA
        CameraImage_CameraImageDataBuilder builder = rti::flat::build_data<CameraImage_CameraImageData>(CameraImage_CameraImageData_writer);
        if (builder.check_failure()) {
            printf("builder creation error (%s:%d)\n", __FILE__, __LINE__);
            publisher_shutdown(participant);
            return -1;
        }

        // Build the CameraImage_CameraImageData data sample using the builder
        if (!build_data_sample(builder, participant)) {
            printf("error building the sample(%s:%d)\n", __FILE__, __LINE__);
            publisher_shutdown(participant);
            return -1;
        }

        // Create the sample
        CameraImage_CameraImageData *instance = builder.finish_sample();
        if (instance == NULL) {
            printf("finish_sample() error(%s:%d)\n", __FILE__, __LINE__);
            publisher_shutdown(participant);
            return -1;
        }
#endif 	// def DDS_LARGE_DATA_FLAT_DATA
#ifdef DDS_LARGE_DATA_ZERO_COPY
#ifndef DDS_LARGE_DATA_FLAT_DATA
        /* Get a new sample before every write. This data sample can come from a
        free sample or a previously written sample which is ready for reuse. */

        retcode = CameraImage_CameraImageData_writer->get_loan(instance);
        if (retcode != DDS_RETCODE_OK) {
            fprintf(stderr, "CameraImage_CameraImageDataDataWriter_get_loan error(%s:%d)\n", __FILE__, __LINE__);
            publisher_shutdown(participant);
            return -1;
        }
#endif 	// ndef DDS_LARGE_DATA_FLAT_DATA
#endif	//  def DDS_LARGE_DATA_ZERO_COPY

#ifndef DDS_LARGE_DATA_FLAT_DATA
        /* If not using Flat Data, Modify the data to be sent here */
        for (int i = 0; i < MAX_IMAGE_SIZE; i+=4) {
            uint32_t *lfsrVal = (uint32_t *)&instance->data[i];
            *lfsrVal = lfsr;
            lfsr = (lfsr >> 1) ^ (-(lfsr & 1u) & 0xD0000001u);
        }
        instance->seqnum = seqNum++;
        // uint64_t tNow = UtcNowPrecise();
        // instance->sec_ = (tNow / 1000000000);
        // instance->nanosec_ = (tNow % 1000000000);
        //printf("tNow: %llu = %u.%u\n", tNow, instance->sec_, instance->nanosec_);

        DDS_Time_t instance_timestamp;
        participant->get_current_time(instance_timestamp);
        instance->sec_     = instance_timestamp.sec;
        instance->nanosec_ = instance_timestamp.nanosec;

#endif  // ndef DDS_LARGE_DATA_FLAT_DATA

        retcode = CameraImage_CameraImageData_writer->write(*instance, instance_handle);
        if (retcode != DDS_RETCODE_OK) {
            fprintf(stderr, "write error %d (%s:%d)\n", retcode, __FILE__, __LINE__);
        }
        printf("CameraImagePub: t=%d.%d, sent %d imgs (size: %.2f MB), BW: %3.3f MB/sec\n", 
            instance->sec_%10000, instance->nanosec_/1000000,
            count,
            1e-6*MAX_IMAGE_SIZE,
            1e-6*MAX_IMAGE_SIZE*send_period_s
            );


        DDS_Time_t time_now;
        participant->get_current_time(time_now);
        double time_now_s   = time_now.sec + 1e-9*time_now.nanosec;
        double time_next_s  = time_begin_s + send_period_s*(count+1);
        double time_sleep_s = time_next_s - time_now_s;
        
        DDS_Duration_t time_sleep;
        time_sleep.sec     = (long)time_sleep_s;
        time_sleep.nanosec = (long)(1e9*(time_sleep_s - time_sleep.sec));

        // printf("CameraImagePub: sleeping %f (%u.%u)\n", time_sleep_s, time_sleep.sec, time_sleep.nanosec);
        NDDSUtility::sleep(time_sleep);
    }

#ifndef DDS_LARGE_DATA_FLAT_DATA
#ifndef DDS_LARGE_DATA_ZERO_COPY
    /*
    retcode = CameraImage_CameraImageData_writer->unregister_instance(
        *instance, instance_handle);
    if (retcode != DDS_RETCODE_OK) {
        fprintf(stderr, "unregister instance error %d (%s:%d)\n", retcode, __FILE__, __LINE__);
    }
    */

    /* Delete data sample */
    retcode = CameraImage_CameraImageDataTypeSupport::delete_data(instance);
    if (retcode != DDS_RETCODE_OK) {
        fprintf(stderr, "CameraImage_CameraImageDataTypeSupport::delete_data error %d (%s:%d)\n", retcode, __FILE__, __LINE__);
    }
#endif // ndef DDS_LARGE_DATA_ZERO_COPY
#endif // ndef DDS_LARGE_DATA_FLAT_DATA

    /* Delete all entities */
    return publisher_shutdown(participant);
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

    return publisher_main(sample_count);
}

