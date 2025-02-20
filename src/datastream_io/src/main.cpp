/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     main.cpp: the main function to input and output data stream
 * @note      Each task is defined by "TODO"
 * ------------------------------------------------------------------------------------------------------------------------------*/

#include "datastream.h"

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        printf("arguments are invalied!\n");
        return false;
    }

    // TODO: extract imu data and write to the bag
    // // step 1: extract imu data from the ros bag file
    // std::string imutopic_in = dataio_common::VisionRTK2_imu_topic;
    // std::list<sensor_msgs::Imu> imudatas(0);
    // dataio_common::Extract_IMUdata_ROSFormat(argv[1], imutopic_in, imudatas);
    // // step 2: write imu data to the ros bag file
    // std::string imutopic_out = dataio_common::VisionRTK2_imu_topic;
    // dataio_common::Write_IMUdata_ROSFormat(argv[2], imutopic_out, imudatas, 1);

    // TODO: extract image data and write to the bag
    // // step 1: extract image data from the ros bag file
    // std::string imgtopic_in = dataio_common::VisionRTK2_image_topic;
    // std::list<sensor_msgs::Image> imgdatas;
    // dataio_common::Extract_ImageData_ROSFormat(argv[1], imgtopic_in, imgdatas);
    // // step 2: write image data to the ros bag file
    // std::string imgtopic_out = dataio_common::VisionRTK2_image_topic;
    // dataio_common::Write_ImageData_ROSFormat(argv[2], imgtopic_out, imgdatas, 2);

    // TODO: extract gnss solution data and write to bag
    // // step 1. extract gnss solution
    // std::list<datastreamio::RobotGVINS_GNSSSol> gnsssol_datas(0);
    // dataio_common::Extract_GNSSSolData_IPSPOSFMT(argv[1], gnsssol_datas);
    // // step 2. write gnss solution data
    // std::string gnsssol_topic = dataio_common::RobotGVINS_gnsssol_topic;
    // std::string imu_topic = dataio_common::RobotGVINS_imu_topic;
    // dataio_common::Write_GNSSSolData_RobotGVINSFormat(argv[2], gnsssol_topic, imu_topic, gnsssol_datas, 2);

    // TODO: extract gnss solution data and write to bag (ROS standard format)
    // step 1. extract gnss solution
    std::list<datastreamio::RobotGVINS_GNSSSol> gnsssol_datas(0);
    dataio_common::Extract_GNSSSolData_IPSPOSFMT(argv[1], gnsssol_datas);
    // step 2: convert data format
    std::list<sensor_msgs::NavSatFix> rosdatas(0);
    dataio_common::Convert_GNSSSolStruct_Alldata_RobotGVINS2ROSFormat(gnsssol_datas, rosdatas);
    // step 3. write gnss solution data
    std::string gnsssol_topic = dataio_common::ROS_gnsssol_topic;
    std::string imu_topic = dataio_common::RobotGVINS_imu_topic;
    dataio_common::Write_GNSSSolData_ROSFormat(argv[2], gnsssol_topic, imu_topic, gnsssol_datas, 2);

    // TODO: extract gnss observations and ephemeris data and write to the rosbag file
    // // step 1: extract gnss raw data from the ros bag file
    // std::string raw_topic = dataio_common::VisionRTK2_gnssraw_topic;
    // std::string imu_topic = dataio_common::VisionRTK2_imu_topic;
    // std::list<gnss_common::IPS_OBSDATA> ips_gnssobs(0);
    // std::list<gnss_common::IPS_GPSEPH> ips_gnsseph(0);
    // dataio_common::Extract_GNSSObsEphData_VisionRTK2Format(argv[1], NULL, raw_topic, ips_gnssobs, ips_gnsseph);
    // // step 2: write the GNSS observations data to bag file with RobotGVINS format
    // std::string gnss_obstopic = dataio_common::RobotGVINS_gnssobs_topic_rove;
    // std::list<datastreamio::RobotGVINS_GNSSObs> robotdata_gnssobs(0);
    // dataio_common::Convert_GNSSObsStruct_Alldata_IPS2RobotGVINS(ips_gnssobs, robotdata_gnssobs);
    // dataio_common::Write_GNSSObsData_RobotGVINSFormat(argv[2], gnss_obstopic, imu_topic, robotdata_gnssobs, 2);
    // // step 3: write the GNSS navigation data to bag file with RobotGVINS format
    // std::string gnss_ephtopic = dataio_common::RobotGVINS_gnsseph_topic_rove;
    // std::list<datastreamio::RobotGVINS_GNSSEph> robotdata_gnsseph(0);
    // dataio_common::Convert_GNSSEphStruct_Alldata_IPS2RobotGVINS(ips_gnsseph, robotdata_gnsseph);
    // dataio_common::Write_GNSSEphData_RobotGVINSFormat(argv[2], gnss_ephtopic, imu_topic, robotdata_gnsseph, 2);

    // TODO: read gnss rinex format observation file and write to ros bag file
    // // step 1: use the IPS function and store all observations data as IPS struct
    // std::list<gnss_common::IPS_OBSDATA> ips_gnssobs(0);
    // dataio_common::Extract_GNSSObsData_RINEX3Format(argv[1], ips_gnssobs);
    // // step 2: write the GNSS observations data to bag file
    // std::string imu_topic = dataio_common::VisionRTK2_imu_topic;
    // std::string gnss_obstopic = dataio_common::RobotGVINS_gnssobs_topic_base;
    // std::list<datastreamio::RobotGVINS_GNSSObs> robotdata_gnssobs(0);
    // dataio_common::Convert_GNSSObsStruct_Alldata_IPS2RobotGVINS(ips_gnssobs, robotdata_gnssobs);
    // dataio_common::Write_GNSSObsData_RobotGVINSFormat(argv[2], gnss_obstopic, imu_topic, robotdata_gnssobs, 2);

    return 0;
}