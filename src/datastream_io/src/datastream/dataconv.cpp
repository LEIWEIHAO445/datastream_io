/** -------------------------------------------------------------------------------------------------------------------------------
 * @brief     dataconv.cpp: the source file defines functions to convert data format
 * @note
 *
 * ------------------------------------------------------------------------------------------------------------------------------*/

#include "datastream.h"

namespace dataio_common
{

    /**
     * @brief       Convert one gnss solution data from RobotGVINS format to ROS standard format
     * @note        1. It is used to process observation data in one epoch
     *
     * @param[in]   RobotGVINS_GNSSSol *      robotdata         gnss solution data in RobotGVINS format
     * @param[in]   NavSatFix                 rosdata           gnss solution data in ros standard format
     *
     * @return
     */
    void Convert_GNSSSolStruct_Onedata_RobotGVINS2ROSFormat(const datastreamio::RobotGVINS_GNSSSol *robotdata, sensor_msgs::NavSatFix &rosdata)
    {

        // header stamp
        rosdata.header = robotdata->header;

        // convert position from XYZ to LLH
        double XYZ[3] = {0.0}, LLH[3] = {0.0};
        XYZ[0] = robotdata->pos_XYZ[0], XYZ[1] = robotdata->pos_XYZ[1], XYZ[2] = robotdata->pos_XYZ[2];
        gnss_common::XYZ2LLH(XYZ, LLH);
        rosdata.latitude = LLH[0] * IPS_R2D, rosdata.longitude = LLH[1] * IPS_R2D, rosdata.altitude = LLH[2];

        // convert the position covariance from ECEF to ENU
        // (1) get the position covariance in ECEF
        Eigen::Matrix3d XYZCov = Eigen::Matrix3d::Zero();
        XYZCov(0, 0) = robotdata->cov_pos_XYZ[0], XYZCov(0, 1) = robotdata->cov_pos_XYZ[1], XYZCov(0, 2) = robotdata->cov_pos_XYZ[2];
        XYZCov(1, 0) = robotdata->cov_pos_XYZ[3], XYZCov(1, 1) = robotdata->cov_pos_XYZ[4], XYZCov(1, 2) = robotdata->cov_pos_XYZ[5];
        XYZCov(2, 0) = robotdata->cov_pos_XYZ[6], XYZCov(2, 1) = robotdata->cov_pos_XYZ[7], XYZCov(2, 2) = robotdata->cov_pos_XYZ[8];
        // (2) convert position covariance to ENU
        Eigen::Matrix3d R_eTon = gnss_common::ComputeRotMat_ENU2ECEF(LLH[0], LLH[1]);
        Eigen::Matrix3d ENUCov = R_eTon * XYZCov * (R_eTon.transpose());
        // (3) get the position covariance in ENU
        rosdata.position_covariance[0] = ENUCov(0, 0), rosdata.position_covariance[1] = ENUCov(0, 1), rosdata.position_covariance[2] = ENUCov(0, 2);
        rosdata.position_covariance[3] = ENUCov(1, 0), rosdata.position_covariance[4] = ENUCov(1, 1), rosdata.position_covariance[5] = ENUCov(1, 2);
        rosdata.position_covariance[6] = ENUCov(2, 0), rosdata.position_covariance[7] = ENUCov(2, 1), rosdata.position_covariance[8] = ENUCov(2, 2);
    }

    /**
     * @brief       Convert one gnss solution data from RobotGVINS format to ROS standard format
     * @note
     *
     * @param[in]   list      robotdata      gnss solution data in RobotGVINS format
     * @param[out]  list      robotdata      gnss solution data in ros standard format
     *
     * @return
     */
    extern void Convert_GNSSSolStruct_Alldata_RobotGVINS2ROSFormat(const std::list<datastreamio::RobotGVINS_GNSSSol> &robotdata, std::list<sensor_msgs::NavSatFix> &rosdata)
    {
        if (robotdata.size() <= 0)
            return;

        for (auto iter : robotdata)
        {
            sensor_msgs::NavSatFix one_msg;
            Convert_GNSSSolStruct_Onedata_RobotGVINS2ROSFormat(&iter, one_msg);
            rosdata.push_back(one_msg);
        }

        return;
    }

    /**
     * @brief       Convert the GNSS observation data from rtklib struct to IPS struct
     * @note        1. It is used to process observation data in one epoch
     *
     * @param[in]   obsd_t *          src       observation data in rtklib struct
     * @param[in]   int               n         satellite number in rtklib
     * @param[out]  IPS_OBSDATA *     dst       observation data in IPS struct
     *
     * @return
     */
    void Convert_GNSSObsStruct_RTKLIB2IPS(const obsd_t *src, int n, gnss_common::IPS_OBSDATA *dst)
    {
        if (!src || !dst)
            return;

        ///< 1. Prepare variables
        int index, sys = 0;
        char(*frq)[5] = NULL;
        char gs_strFrq_tmp[NFREQ][5] = {'\0'};
        gnss_common::IPS_GPSTIME gt;

        gnss_common::ConvertTime(src[0].time, &gt);
        if (MinusGPSTIME(gt, dst->gt) == 0.0)
            return;

        // clear the old data body
        dst->gt = gt;
        dst->flag = 0;
        dst->nsat = 0;
        memset(dst->ngnss, 0, sizeof(int) * IPS_NSYS);
        dst->obs.clear();

        for (int i = 0; i < n; i++)
        {
            if (dst->nsat >= MAXOBS)
                break;

            gnss_common::IPS_OBSDATA_t iobs;
            iobs.prn = gnss_common::ConvertPrn(src[i].sat);
            if (iobs.prn > IPS_NSATMAX || iobs.prn < 1)
                continue;

            for (int f = 0; f < NFREQ; f++)
            {
                std::memset(gs_strFrq_tmp[f], '\0', sizeof(gs_strFrq_tmp[f]));
            }

            gnss_common::satprn2no(iobs.prn, &sys);
            if (sys == IPS_SYSGPS)
            {
                dst->ngnss[0]++;
                for (int f = 0; f < NFREQ; f++)
                    std::memcpy(gs_strFrq_tmp[f], gnss_common::gs_strGPSFrq[f].c_str(), gnss_common::gs_strGPSFrq[f].size() + 1);
                frq = gs_strFrq_tmp;
            }
            else if (sys == IPS_SYSGLO)
            {
                dst->ngnss[2]++;
                for (int f = 0; f < NFREQ; f++)
                    std::memcpy(gs_strFrq_tmp[f], gnss_common::gs_strGLOFrq[f].c_str(), gnss_common::gs_strGLOFrq[f].size() + 1);
                frq = gs_strFrq_tmp;
            }
            else if (sys == IPS_SYSBD2)
            {
                dst->ngnss[2]++;
                for (int f = 0; f < NFREQ; f++)
                    std::memcpy(gs_strFrq_tmp[f], gnss_common::gs_strBD2Frq[f].c_str(), gnss_common::gs_strBD2Frq[f].size() + 1);
                frq = gs_strFrq_tmp;
            }
            else if (sys == IPS_SYSBD3)
            {
                dst->ngnss[3]++;
                for (int f = 0; f < NFREQ; f++)
                    std::memcpy(gs_strFrq_tmp[f], gnss_common::gs_strBD3Frq[f].c_str(), gnss_common::gs_strBD3Frq[f].size() + 1);
                frq = gs_strFrq_tmp;
            }
            else if (sys == IPS_SYSGAL)
            {
                dst->ngnss[4]++;
                for (int f = 0; f < NFREQ; f++)
                    std::memcpy(gs_strFrq_tmp[f], gnss_common::gs_strGALFrq[f].c_str(), gnss_common::gs_strGALFrq[f].size() + 1);
                frq = gs_strFrq_tmp;
            }
            else if (sys == IPS_SYSQZS)
            {
                dst->ngnss[4]++;
                for (int f = 0; f < NFREQ; f++)
                    std::memcpy(gs_strFrq_tmp[f], gnss_common::gs_strQZSFrq[f].c_str(), gnss_common::gs_strQZSFrq[f].size() + 1);
                frq = gs_strFrq_tmp;
            }
            else
                continue;

            for (int f = 0; f < NFREQ; f++)
            {
                index = gnss_common::FindFrqIndex(sys, frq + f, src[i]);
                if (index < 0)
                    continue;
                iobs.P[f] = src[i].P[index];
                iobs.L[f] = src[i].L[index];
                iobs.D[f] = src[i].D[index];
                iobs.LLI[f] = src[i].LLI[index] & (LLI_SLIP | LLI_HALFC | LLI_BOCTRK);
                iobs.S[f] = (float)(src[i].SNR[index] * SNR_UNIT);
            }

            dst->obs.push_back(iobs);
            dst->nsat++;
        }
        gnss_common::SortGNSSObs_IPSStruct(dst);
        frq = NULL;

        return;
    }

    /**
     * @brief       Convert the gnss observation data from IPS struct to RobotGVINS struct
     * @note        1. It is used to process observation data in one epoch
     *
     * @param[in]   IPS_OBSDATA *           ipsdata         observation data in IPS struct
     * @param[in]   RobotGVINS_GNSSObs      robotdata       observation data in RobotGVINS format
     *
     * @return
     */
    void Convert_GNSSObsStruct_Onedata_IPS2RobotGVINS(const gnss_common::IPS_OBSDATA *ipsdata, datastreamio::RobotGVINS_GNSSObs &robotdata)
    {
        if (ipsdata == NULL)
            return;

        // timestamp
        robotdata.header.stamp = ros::Time(ipsdata->pubtime);
        // robotdata.header.stamp = ros::Time(ipsdata->gt.GPSWeek * 604800 + ipsdata->gt.secsOfWeek + ipsdata->gt.fracOfSec);

        // observation info
        robotdata.flag = ipsdata->flag;
        robotdata.nsat = ipsdata->nsat;
        for (int i = 0; i < IPS_NSYS; i++)
            robotdata.ngnss.push_back(ipsdata->ngnss[i]);

        // observation data for each satellite
        for (int i = 0; i < ipsdata->obs.size(); i++)
        {
            datastreamio::RobotGVINS_GNSSSat sat_msg;
            sat_msg.prn = ipsdata->obs.at(i).prn;
            for (int f = 0; f < NFREQ; f++)
            {
                sat_msg.cp_meas.push_back(ipsdata->obs.at(i).L[f]);
                sat_msg.pr_meas.push_back(ipsdata->obs.at(i).P[f]);
                sat_msg.do_meas.push_back(ipsdata->obs.at(i).D[f]);
                sat_msg.sig_cno.push_back(ipsdata->obs.at(i).S[f]);
                sat_msg.code.push_back(ipsdata->obs.at(i).code[f]);
                sat_msg.SNR.push_back(ipsdata->obs.at(i).SNR[f]);
                sat_msg.LLI.push_back(ipsdata->obs.at(i).LLI[f]);
                sat_msg.cs.push_back(ipsdata->obs.at(i).cs[f]);
                sat_msg.P_TGD.push_back(ipsdata->obs.at(i).P_TGD[f]);
                sat_msg.SMP.push_back(ipsdata->obs.at(i).SMP[f]);
            }
            robotdata.obsdata.push_back(sat_msg);
        }
    }

    /**
     * @brief       Convert all gnss observation data from IPS struct to RobotGVINS format
     * @note
     *
     * @param[in]   list      ipsdata        gnss observation data in IPS format
     * @param[out]  list      robotdata      gnss observation data in RobotGVINS format
     *
     * @return
     */
    extern void Convert_GNSSObsStruct_Alldata_IPS2RobotGVINS(const std::list<gnss_common::IPS_OBSDATA> &ipsdata, std::list<datastreamio::RobotGVINS_GNSSObs> &robotdata)
    {
        if (ipsdata.size() <= 0)
            return;

        for (auto iter : ipsdata)
        {
            datastreamio::RobotGVINS_GNSSObs one_msg;
            Convert_GNSSObsStruct_Onedata_IPS2RobotGVINS(&iter, one_msg);
            robotdata.push_back(one_msg);
        }
    }

    /**
     * @brief       Convert rtklib nav data to IPS eph data
     * @note        1. It is used to process observation data in one epoch
     *              2. GLONASS eph is not considered
     *
     * @param[in]   nav_t*           src     rtklib eph data
     * @param[out]  IPS_GPSEPH*      n       IPS eph data
     *
     * @return
     */
    void Convert_GNSSNavStruct_RTKLIB2IPS(const nav_t *src, gnss_common::IPS_GPSEPH *dst)
    {
        if (dst == NULL)
            return;

        for (int i = 0; i < (src->n - src->ng); i++)
        {
            int prn = gnss_common::ConvertPrn(src->eph[i].sat);
            if (prn < 1 || prn > IPS_NSATMAX)
                continue;

            gnss_common::IPS_GPSTIME gt;
            gnss_common::ConvertTime(src->eph[i].toe, &gt);
            double dt = MinusGPSTIME(gt, dst[prn - 1].toe);
            if (dt <= 0.0)
                continue;

            Convert_GNSSEphStruct_RTKLIB2IPS(&src->eph[i], &dst[prn - 1]);
        }
    }

    /**
     * @brief       Convert rtklib eph data to IPS eph data
     * @note        1. It is used to process observation data in one epoch
     *              2. GLONASS eph is not considered
     *
     * @param[in]   eph_t*           src     rtklib eph data
     * @param[out]  IPS_GPSEPH*      n       IPS eph data
     *
     * @return
     */
    void Convert_GNSSEphStruct_RTKLIB2IPS(const eph_t *src, gnss_common::IPS_GPSEPH *dst)
    {
        gnss_common::ConvertTime(src->toe, &dst->toe);
        gnss_common::ConvertTime(src->toc, &dst->toc);

        dst->prn = gnss_common::ConvertPrn(src->sat);
        dst->iode = src->iode;
        dst->iodc = src->iodc;
        dst->sva = src->sva;
        dst->svh = src->svh;
        dst->week = src->week;
        dst->code = src->code;
        dst->A = src->A;
        dst->e = src->e;
        dst->i0 = src->i0;
        dst->OMG0 = src->OMG0;
        dst->omg = src->omg;
        dst->M0 = src->M0;
        dst->deln = src->deln;
        dst->OMGd = src->OMGd;
        dst->idot = src->idot;
        dst->crc = src->crc;
        dst->crs = src->crs;
        dst->cuc = src->cuc;
        dst->cus = src->cus;
        dst->cic = src->cic;
        dst->cis = src->cis;
        dst->toes = src->toes;
        dst->f0 = src->f0;
        dst->f1 = src->f1;
        dst->f2 = src->f2;

        for (int i = 0; i < 4; i++)
        {
            dst->tgd[i] = src->tgd[i];
        }
    }

    /**
     * @brief       Convert the gnss ephemeris data from IPS struct to RobotGVINS struct
     * @note        1. It is used to process ephemeris data for only one satellite
     *
     * @param[in]   IPS_GPSEPH *            ipsdata         ephemeris data in IPS struct
     * @param[in]   RobotGVINS_GNSSEph      robotdata       ephemeris data in RobotGVINS format
     *
     * @return
     */
    void Convert_GNSSEphStruct_Onedata_IPS2RobotGVINS(const gnss_common::IPS_GPSEPH *ipsdata, datastreamio::RobotGVINS_GNSSEph &robotdata)
    {
        if (ipsdata == NULL)
            return;

        // get the ephemeris system
        int sys = IPS_SYSNON;
        int prn = ipsdata->prn;                               // GNSS prn in IPS program
        int sat = gnss_common::satprn2no(prn, &sys);          // GNSS prn for each system
        robotdata.header.stamp = ros::Time(ipsdata->pubtime); // the timestamp to publish ros message

        // get the data body
        robotdata.prn = ipsdata->prn;
        robotdata.iode = ipsdata->iode;
        robotdata.iodc = ipsdata->iodc;
        robotdata.sva = ipsdata->sva;
        robotdata.svh = ipsdata->svh;
        robotdata.week = ipsdata->week;
        robotdata.code = ipsdata->code;
        robotdata.flag = ipsdata->flag;
        robotdata.toe = ipsdata->toe.GPSWeek * 604800 + ipsdata->toe.secsOfWeek + ipsdata->toe.fracOfSec;
        robotdata.toc = ipsdata->toc.GPSWeek * 604800 + ipsdata->toc.secsOfWeek + ipsdata->toc.fracOfSec;
        robotdata.ttr = ipsdata->ttr.GPSWeek * 604800 + ipsdata->ttr.secsOfWeek + ipsdata->ttr.fracOfSec;
        robotdata.eph_A = ipsdata->A;
        robotdata.eph_e = ipsdata->e;
        robotdata.i0 = ipsdata->i0;
        robotdata.OMG0 = ipsdata->OMG0;
        robotdata.omg = ipsdata->omg;
        robotdata.M0 = ipsdata->M0;
        robotdata.deln = ipsdata->deln;
        robotdata.OMGd = ipsdata->OMGd;
        robotdata.idot = ipsdata->idot;
        robotdata.crc = ipsdata->crc;
        robotdata.crs = ipsdata->crs;
        robotdata.cuc = ipsdata->cuc;
        robotdata.cus = ipsdata->cus;
        robotdata.cic = ipsdata->cic;
        robotdata.cis = ipsdata->cis;
        robotdata.toes = ipsdata->toes;
        robotdata.fit = ipsdata->fit;
        robotdata.f0 = ipsdata->f0;
        robotdata.f1 = ipsdata->f1;
        robotdata.f2 = ipsdata->f2;
        for (int i = 0; i < 4; i++)
            robotdata.tgd[i] = ipsdata->tgd[i];
    }

    /**
     * @brief       Convert all gnss ephemeris data from IPS struct to RobotGVINS format
     * @note
     *
     * @param[in]   list      ipsdata        gnss ephemeris data in IPS format
     * @param[out]  list      robotdata      gnss ephemeris data in RobotGVINS format
     *
     * @return
     */
    extern void Convert_GNSSEphStruct_Alldata_IPS2RobotGVINS(const std::list<gnss_common::IPS_GPSEPH> &ipsdata, std::list<datastreamio::RobotGVINS_GNSSEph> &robotdata)
    {
        if (ipsdata.size() <= 0)
            return;

        for (auto iter : ipsdata)
        {
            datastreamio::RobotGVINS_GNSSEph one_msg;
            Convert_GNSSEphStruct_Onedata_IPS2RobotGVINS(&iter, one_msg);
            robotdata.push_back(one_msg);
        }
    }
}