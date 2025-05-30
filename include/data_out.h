/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     data_out.h: wirte data to files
 * @note      the header file defines constants, strcuts, classes and function prototypes
 *
 * ------------------------------------------------------------------------------------------------------------------------------*/

#ifndef __DATAOUT_HEADER_H__
#define __DATAOUT_HEADER_H__

#include "datastream.h"
#include "gnss_common.h"

namespace dataio_common
{
    /**
     * @brief       Write imu data to bag file
     */
    extern bool Write_IMUdata_ROSBag(const char *bag_outfilepath, const std::string imu_topic, const std::list<sensor_msgs::Imu> &imudatas, int bagmode = 1);

    /**
     * @brief       Write image data to bag file
     */
    extern bool Write_ImageData_ROSBag(const char *bag_outfilepath, const std::string img_topic, const std::list<sensor_msgs::Image> &imgdatas, int bagmode = 1);

    /**
     * @brief       Write INS solution data
     */
    extern bool Write_INSSolution(const char *output_filepath, const std::list<Solution_INS> &sol_datas, dataformat datatype);

    /**
     * @brief       Write GNSS solution data to rosbag file
     * @note
     */
    extern bool Write_GNSSolution_ROSBag(const char *filepath, const std::list<Solution_GNSS> &soldatas, std::string &imu_topic, std::string &gnss_topic, dataformat datatype, int bagmode);
    extern void write_gnsssol_robotgvins_rosbag(rosbag::Bag &outfile_bag, const std::list<Solution_GNSS> &sol_datas, std::string &gnsssol_topic, const ros::Time &start_time, const ros::Time &end_time);

    /**
     * @brief       Write gnss observations data to ros bag file as RobotGVINS format
     */
    extern bool Write_GNSSObsData_RobotGVINSFormat(const char *bag_outfilepath, const std::string gnssobs_topic, const std::string imu_topic, const std::list<datastreamio::RobotGVINS_GNSSObs> &gnss_obsdata, int bagmode = 1);
    extern bool Write_GNSSObsData_RINEXFormat(const char *bag_outfilepath, const std::list<gnss_common::IPS_OBSDATA> &gnss_obsdata);

    /**
     * @brief       Write gnss ephemeris data to ros bag file as RobotGVINS format
     */
    extern bool Write_GNSSEphData_RobotGVINSFormat(const char *bag_outfilepath, const std::string gnss_ephtopic, const std::string imu_topic, const std::list<datastreamio::RobotGVINS_GNSSEph> &gnss_ephdata, int bagmode = 1, bool rospublish = true);
    extern bool Write_GNSSEphData_RINEXFormat(const char *bag_outfilepath, const std::list<gnss_common::IPS_GPSEPH> &gnss_ephdata);

    /// FIXME: need to delete

    /**
     * @brief       Write gnss solution data to ros bag file as RobotGVINS format
     */
    extern bool
    Write_GNSSSolData_RobotGVINSFormat(const char *bag_outfilepath, const std::string gnsssol_topic, const std::string imu_topic, const std::list<datastreamio::RobotGVINS_GNSSSol> &gnsssol_datas, int bagmode = 1);

    /**
     * @brief       Write gnss solution data to ros bag file as ros standard format
     */
    extern bool Write_GNSSSolData_ROSFormat(const char *bag_outfilepath, const std::string gnsssol_topic, const std::string imu_topic, const std::list<datastreamio::RobotGVINS_GNSSSol> &gnsssol_datas, int bagmode = 1);

}

#endif