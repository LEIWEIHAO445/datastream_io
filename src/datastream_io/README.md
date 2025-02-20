# datastream_io
## A toolkit for data conversion and data stream management

**Introduction:**
The datastream_io is a toolkit used to convert data format and manage data stream, which supports the multiple data format conversion *(ros standard format, text format, binary format, ublox/rtcm and other GNSS data format)* and multiple data communication mode *(file, ros, tcp, serial)*. We contribute this toolkit to the community and hope that it will help researchers avoid the problem of data format conversion.

**Features:**
- multiple data format in multi-sensors fusion: ros standard, text format, binary format, ublox/rtcm and other GNSS data format
- data conversion
- data communication: file, ros, tcp, serial

**Authors:**
The [PLANET] in WHU-SGG

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit: 18.04 or later versions
ROS: Melodic or later versions. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2 **OpenCV**
OpenCV: both version 3 and 4 are avariable

## 2. Build
Clone the project to '~/catkin_ws/src'
'''
    cd ~/catkin_ws
    catkin_make
'''

## 3. Example
### 3.1 data conversion
For an example, we convert the *RINEX3.0 format* GNSS raw data file to the *RobotGVINS format(customized format)*, the steps are as follows:

    // step 1: read GNSS raw data from RINEX3.0 format
    std::list<gnss_common::IPS_OBSDATA> ips_gnssobs(0);
    dataio_common::Extract_GNSSObsData_RINEX3Format(argv[1], ips_gnssobs);
    
    // step 2: convert data format to *RobotGVINS format(customized format)*
    std::list<datastreamio::RobotGVINS_GNSSObs> robotdata_gnssobs(0);
    dataio_common::Convert_GNSSObsStruct_Alldata_IPS2RobotGVINS(ips_gnssobs, robotdata_gnssobs);
    
    // step 3: write GNSS data to the bag
    std::string imu_topic = dataio_common::VisionRTK2_imu_topic;
    std::string gnss_obstopic = dataio_common::RobotGVINS_gnssobs_topic_base;
    dataio_common::Write_GNSSObsData_RobotGVINSFormat(argv[2], gnss_obstopic, imu_topic, robotdata_gnssobs, 2);

### 3.2 data communication


## 4. Acknowledgements
We used [rtklib](https://github.com/tomojitakasu/RTKLIB/tree/rtklib_2.4.3) for GNSS format data conversion.

## 5. License
We are still working on improving the code reliability. For any technical issues, please contact Weihao Lei <whlei20000419@whu.edu.cn>.