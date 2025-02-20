/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     dataout.cpp: the source file defines functions to write and output data to files/serial/tcp
 * @note
 *
 * ------------------------------------------------------------------------------------------------------------------------------*/

#include "datastream.h"

namespace dataio_common
{
    /**
     * @brief       Write imu data to bag file as ros standard foramt
     * @note        1. The imu data to write should be ros standard format defaultly
     *              2. If need, the GPS/Linux time will be converted
     *
     * @param[in]  char*       bag_outfilepath      bag filepath to write data
     * @param[in]  string      imu_topic            ros topic
     * @param[in]  list        imudatas             imu observations data of all epochs
     * @param[in]  int         bagmode              1:write 2:app
     *
     * @return     bool       true       write successful
     *                        false      fail to write
     */
    extern bool Write_IMUdata_ROSFormat(const char *bag_outfilepath, const std::string imu_topic, const std::list<sensor_msgs::Imu> &imudatas, int bagmode)
    {

        // 1. Open bag file to write data
        rosbag::Bag outfile_bag;
        if (bagmode == 1)
            outfile_bag.open(bag_outfilepath, rosbag::bagmode::Write);
        else if (bagmode == 2)
            outfile_bag.open(bag_outfilepath, rosbag::bagmode::Append);
        else
        {
            printf("The bag mode is wrong!\n");
            return false;
        }
        if (!outfile_bag.isOpen())
        {
            printf("open ros bag file to write data unsuccessfully!\n");
            return false;
        }

        // 2. Write the imu data to bag file
        for (auto iter = imudatas.begin(); iter != imudatas.end(); ++iter)
        {
            sensor_msgs::Imu imu_msg = *iter;
            outfile_bag.write(imu_topic, imu_msg.header.stamp, imu_msg);
        }

        // close the file
        outfile_bag.close();

        return true;
    }

    /**
     * @brief       Write image data to bag file as ros standard foramt
     * @note        1. The image data to write should be ros standard format defaultly
     *              2. If need, the GPS/Linux time will be converted
     *
     * @param[in]   char*       bag_outfilepath      bag filepath to write data
     * @param[in]   string      img_topic            ros topic
     * @param[in]   list        imgdatas             image data of all epochs
     * @param[in]   int         bagmode              1:write 2:app
     *
     * @return      bool       true       write successful
     *                         false      fail to write
     */
    extern bool Write_ImageData_ROSFormat(const char *bag_outfilepath, const std::string img_topic, const std::list<sensor_msgs::Image> &imgdatas, int bagmode)
    {
        // 1. Open bag file to write data
        rosbag::Bag outfile_bag;
        if (bagmode == 1)
            outfile_bag.open(bag_outfilepath, rosbag::bagmode::Write);
        else if (bagmode == 2)
            outfile_bag.open(bag_outfilepath, rosbag::bagmode::Append);
        else
        {
            printf("The bag mode is wrong!\n");
            return false;
        }
        if (!outfile_bag.isOpen())
        {
            printf("open ros bag file to write data unsuccessfully!\n");
            return false;
        }

        // 2. Write the image data to bag file
        for (auto iter = imgdatas.begin(); iter != imgdatas.end(); ++iter)
        {
            sensor_msgs::Image img_msg = *iter;
            outfile_bag.write(img_topic, img_msg.header.stamp, img_msg);

            // convert cv::Mat to ros message
            // cv::Mat image_data = cv::imread(image_filename);
            // sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_data).toImageMsg();
        }

        // close file
        outfile_bag.close();

        return true;
    }

    /**
     * @brief       Write GNSS observations data to ros bag file as RobotGVINS format
     * @note        1. The gnss observation data to write should be ros RobotGVINS format defaultly
     *              2. If need, the GPS/Linux time will be converted
     *              3. If there is imu data, the start and end time should be determined by imu timestamp
     *
     * @param[in]   char*       bag_outfilepath      bag filepath to write data
     * @param[in]   string      gnss_obstopic        ros topic
     * @param[in]   string      imu_topic            ros imu topic
     * @param[in]   vector      gnss_obsdata         gnss observations data of all epochs
     * @param[in]   int         bagmode              1:write 2:app
     *
     * @return      bool       true       write successful
     *                         false      fail to write
     */
    extern bool Write_GNSSObsData_RobotGVINSFormat(const char *bag_outfilepath, const std::string gnss_obstopic, const std::string imu_topic, const std::list<datastreamio::RobotGVINS_GNSSObs> &gnss_obsdata, int bagmode)
    {

        // 0. Get the start and end time of imu message
        // open the bag file
        rosbag::Bag infile_bag;
        infile_bag.open(bag_outfilepath, rosbag::bagmode::Read);
        if (!infile_bag.isOpen())
        {
            printf("open ros bag file to get start and end time unsuccessfully!\n");
            return false;
        }

        // prepare variables
        bool first_imu = true; // the flag to check first message
        ros::Time start_time(0.0), end_time(0.0);
        std::vector<std::string> topics;
        topics.push_back(std::string(imu_topic));
        rosbag::View view(infile_bag, rosbag::TopicQuery(topics));

        foreach (rosbag::MessageInstance const m, view)
        {
            if (m.instantiate<sensor_msgs::Imu>() != nullptr)
            {
                // get one imu message
                sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
                double timestamp = imu_msg->header.stamp.sec + imu_msg->header.stamp.nsec * 1e-9;

                // record the start time only once
                if (first_imu)
                {
                    start_time = ros::Time(timestamp);
                    first_imu = false;
                }
                // record the end time
                end_time = ros::Time(timestamp);
            }
        }

        // need to close the file
        infile_bag.close();

        // check the start time and end time
        if (start_time <= ros::Time(0.0) || end_time <= ros::Time(0.0) || end_time <= start_time)
        {
            printf("the start and end time is abnormal!\n");
            return false;
        }

        // 1. Open bag file to write data
        rosbag::Bag outfile_bag;
        if (bagmode == 1)
            outfile_bag.open(bag_outfilepath, rosbag::bagmode::Write);
        else if (bagmode == 2)
            outfile_bag.open(bag_outfilepath, rosbag::bagmode::Append);
        else
        {
            printf("The bag mode is wrong!\n");
            return false;
        }
        if (!outfile_bag.isOpen())
        {
            printf("open ros bag file to write data unsuccessfully!\n");
            return false;
        }

        // 2. Write observations data in each epoch to bag file
        for (auto iter = gnss_obsdata.begin(); iter != gnss_obsdata.end(); ++iter)
        {
            // convert the IPS struct to RobotGVINS struct
            datastreamio::RobotGVINS_GNSSObs obs_msg = *iter;

            // write to bag file with specific topic
            if (obs_msg.header.stamp > start_time && obs_msg.header.stamp < end_time)
                outfile_bag.write(gnss_obstopic, obs_msg.header.stamp, obs_msg);
        }

        // close the file
        outfile_bag.close();

        return true;
    }

    /**
     * @brief       Write gnss ephemeris data to ros bag file as RobotGVINS format
     * @note        1. The gnss ephemeris data to write should be ros RobotGVINS format defaultly
     *              2. If need, the GPS/Linux time will be converted
     *              3. If there is imu data, the start and end time should be determined by imu timestamp
     *
     * @param[in]   char*       bag_outfilepath     bag filepath to write data
     * @param[in]   string      gnss_ephtopic       ros topic
     * @param[in]   string      imu_topic           ros imu topic
     * @param[in]   vector      gnss_ephdata        ephemeris data of all satellites in all epochs
     * @param[in]   int         bagmode             1:write 2:app
     *
     * @return     bool       true       write successful
     *                        false      fail to write
     */
    extern bool Write_GNSSEphData_RobotGVINSFormat(const char *bag_outfilepath, const std::string gnss_ephtopic, const std::string imu_topic, const std::list<datastreamio::RobotGVINS_GNSSEph> &gnss_ephdata, int bagmode)
    {
        // 1. Open bag file to write data
        rosbag::Bag outfile_bag;
        if (bagmode == 1)
            outfile_bag.open(bag_outfilepath, rosbag::bagmode::Write);
        else if (bagmode == 2)
            outfile_bag.open(bag_outfilepath, rosbag::bagmode::Append);
        else
        {
            printf("The bag mode is wrong!\n");
            return false;
        }
        if (!outfile_bag.isOpen())
        {
            printf("open ros bag file to write data unsuccessfully!\n");
            return false;
        }

        // 2. Write ephemeris data in each epoch
        for (auto iter = gnss_ephdata.begin(); iter != gnss_ephdata.end(); ++iter)
        {
            // convert the IPS struct to RobotGVINS struct
            datastreamio::RobotGVINS_GNSSEph eph_msg = *iter;

            // write to bag file with specific topic
            outfile_bag.write(gnss_ephtopic, eph_msg.header.stamp, eph_msg);
        }

        // close the file
        outfile_bag.close();

        return true;
    }

    /**
     * @brief       Write gnss solution data to ros bag file as RobotGVINS format
     * @note        1. The gnss solution data to write should be ros RobotGVINS format defaultly
     *              2. If need, the GPS/Linux time will be converted
     *              3. If there is imu data, the start and end time should be determined by imu timestamp
     *
     * @param[in]   char*       bag_outfilepath      bag filepath to write data
     * @param[in]   string      gnsssol_topic        ros topic
     * @param[in]   string      imu_topic            ros topic (used to get the start and end time)
     * @param[in]   list        gnsssol_datas        gnss solutions data of all epochs
     * @param[in]   int         bagmode              1:write 2:app
     *
     * @return     bool       true                write successful
     *                        false               fail to write
     */
    extern bool Write_GNSSSolData_RobotGVINSFormat(const char *bag_outfilepath, const std::string gnsssol_topic, const std::string imu_topic, const std::list<datastreamio::RobotGVINS_GNSSSol> &gnsssol_datas, int bagmode)
    {

        // 0. Get the start and end time of imu message
        // open the bag file
        rosbag::Bag infile_bag;
        infile_bag.open(bag_outfilepath, rosbag::bagmode::Read);
        if (!infile_bag.isOpen())
        {
            printf("open ros bag file to get start and end time unsuccessfully!\n");
            return false;
        }

        // prepare variables
        bool first_imu = true; // the flag to check first message
        ros::Time start_time(0.0), end_time(0.0);
        std::vector<std::string> topics;
        topics.push_back(std::string(imu_topic));
        rosbag::View view(infile_bag, rosbag::TopicQuery(topics));

        foreach (rosbag::MessageInstance const m, view)
        {
            if (m.instantiate<sensor_msgs::Imu>() != nullptr)
            {
                // get one imu message
                sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
                double timestamp = imu_msg->header.stamp.sec + imu_msg->header.stamp.nsec * 1e-9;

                // record the start time only once
                if (first_imu)
                {
                    start_time = ros::Time(timestamp);
                    first_imu = false;
                }
                // record the end time
                end_time = ros::Time(timestamp);
            }
        }

        // need to close the file
        infile_bag.close();

        // check the start time and end time
        if (start_time <= ros::Time(0.0) || end_time <= ros::Time(0.0) || end_time <= start_time)
        {
            printf("the start and end time is abnormal!\n");
            return false;
        }

        // 1. Open bag file to write GNSS solution data
        rosbag::Bag outfile_bag;
        if (bagmode == 1)
            outfile_bag.open(bag_outfilepath, rosbag::bagmode::Write);
        else if (bagmode == 2)
            outfile_bag.open(bag_outfilepath, rosbag::bagmode::Append);
        else
        {
            printf("The bag mode is wrong!\n");
            return false;
        }
        if (!outfile_bag.isOpen())
        {
            printf("open output data file unsuccessfully!\n");
            return false;
        }

        // 2. Write the GNSS solution data to the file
        for (const auto &iter : gnsssol_datas)
        {
            datastreamio::RobotGVINS_GNSSSol gnsssol_msg = iter;

            if (gnsssol_msg.header.stamp > start_time && gnsssol_msg.header.stamp < end_time)
                outfile_bag.write(gnsssol_topic, gnsssol_msg.header.stamp, gnsssol_msg);
        }

        // close file
        outfile_bag.close();

        return true;
    }

    /**
     * @brief       Write gnss solution data to ros bag file as ros standard format
     * @note        1. The gnss solution data should be converted from RobotGVINS format to ros standard format
     *              2. The gnss solution data to write should be ros ros standard format defaultly
     *              3. If need, the GPS/Linux time will be converted
     *              4. If there is imu data, the start and end time should be determined by imu timestamp
     *
     * @param[in]   char*       bag_outfilepath      bag filepath to write data
     * @param[in]   string      gnsssol_topic        ros topic
     * @param[in]   string      imu_topic            ros topic (used to get the start and end time)
     * @param[in]   list        gnsssol_datas        gnss solutions data of all epochs
     * @param[in]   int         bagmode              1:write 2:app
     *
     * @return     bool       true                write successful
     *                        false               fail to write
     */
    extern bool Write_GNSSSolData_ROSFormat(const char *bag_outfilepath, const std::string gnsssol_topic, const std::string imu_topic, const std::list<datastreamio::RobotGVINS_GNSSSol> &gnsssol_datas, int bagmode)
    {

        // 0. Get the start and end time of imu message
        // open the bag file
        rosbag::Bag infile_bag;
        infile_bag.open(bag_outfilepath, rosbag::bagmode::Read);
        if (!infile_bag.isOpen())
        {
            printf("open ros bag file to get start and end time unsuccessfully!\n");
            return false;
        }

        // prepare variables
        bool first_imu = true; // the flag to check first message
        ros::Time start_time(0.0), end_time(0.0);
        std::vector<std::string> topics;
        topics.push_back(std::string(imu_topic));
        rosbag::View view(infile_bag, rosbag::TopicQuery(topics));

        foreach (rosbag::MessageInstance const m, view)
        {
            if (m.instantiate<sensor_msgs::Imu>() != nullptr)
            {
                // get one imu message
                sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
                double timestamp = imu_msg->header.stamp.sec + imu_msg->header.stamp.nsec * 1e-9;

                // record the start time only once
                if (first_imu)
                {
                    start_time = ros::Time(timestamp);
                    first_imu = false;
                }
                // record the end time
                end_time = ros::Time(timestamp);
            }
        }

        // need to close the file
        infile_bag.close();

        // check the start time and end time
        if (start_time <= ros::Time(0.0) || end_time <= ros::Time(0.0) || end_time <= start_time)
        {
            printf("the start and end time is abnormal!\n");
            return false;
        }

        // 1. Open bag file to write GNSS solution data
        rosbag::Bag outfile_bag;
        if (bagmode == 1)
            outfile_bag.open(bag_outfilepath, rosbag::bagmode::Write);
        else if (bagmode == 2)
            outfile_bag.open(bag_outfilepath, rosbag::bagmode::Append);
        else
        {
            printf("The bag mode is wrong!\n");
            return false;
        }
        if (!outfile_bag.isOpen())
        {
            printf("open output data file unsuccessfully!\n");
            return false;
        }

        // 2. Write the GNSS solution data to the file
        for (const auto &iter : gnsssol_datas)
        {
            datastreamio::RobotGVINS_GNSSSol gnsssol_msg = iter;

            // skip data that is not within the time period
            if (gnsssol_msg.header.stamp < start_time || gnsssol_msg.header.stamp > end_time)
                continue;

            // convert the RobotGVINS format to the ros standard format
            // (1) convert position from LLH to ECEF
            double LLH[3] = {0.0}, XYZ[3] = {0.0};
            XYZ[0] = iter.pos_XYZ[0], XYZ[1] = iter.pos_XYZ[1], XYZ[2] = iter.pos_XYZ[2];
            gnss_common::XYZ2LLH(XYZ, LLH);
            LLH[0] *= IPS_R2D, LLH[1] *= IPS_R2D;
            // get the position covriance in XYZ
            Eigen::Matrix3d XYZCov = Eigen::Matrix3d::Zero();
            XYZCov(0, 0) = iter.cov_pos_XYZ[0], XYZCov(0, 1) = iter.cov_pos_XYZ[1], XYZCov(0, 2) = iter.cov_pos_XYZ[2];
            XYZCov(1, 0) = iter.cov_pos_XYZ[3], XYZCov(1, 1) = iter.cov_pos_XYZ[4], XYZCov(1, 2) = iter.cov_pos_XYZ[5];
            XYZCov(2, 0) = iter.cov_pos_XYZ[6], XYZCov(2, 1) = iter.cov_pos_XYZ[7], XYZCov(2, 2) = iter.cov_pos_XYZ[8];
            // (3) convert position covariance from ECEF to ENU
            Eigen::Matrix3d R_eTon = (gnss_common::ComputeRotMat_ENU2ECEF(LLH[0], LLH[1])).transpose();
            Eigen::Matrix3d ENUCov = R_eTon * XYZCov * (R_eTon.transpose());

            // write the gnss solution data
            sensor_msgs::NavSatFix one_data;
            one_data.header = iter.header;
            one_data.latitude = LLH[0], one_data.longitude = LLH[1], one_data.altitude = LLH[2];
            one_data.position_covariance[0] = ENUCov(0, 0), one_data.position_covariance[1] = ENUCov(0, 1), one_data.position_covariance[2] = ENUCov(0, 2);
            one_data.position_covariance[3] = ENUCov(1, 0), one_data.position_covariance[4] = ENUCov(1, 1), one_data.position_covariance[5] = ENUCov(1, 2);
            one_data.position_covariance[6] = ENUCov(2, 0), one_data.position_covariance[7] = ENUCov(2, 1), one_data.position_covariance[8] = ENUCov(2, 2);

            outfile_bag.write(gnsssol_topic, one_data.header.stamp, one_data);
        }

        // close file
        outfile_bag.close();

        return true;
    }
}