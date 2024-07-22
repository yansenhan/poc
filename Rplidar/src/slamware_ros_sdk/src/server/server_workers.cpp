
#include "server_workers.h"
#include "slamware_ros_sdk_server.h"
#include <rpos/message/base_messages.h>

#include <boost/assert.hpp>
#include <sensor_msgs/MagneticField.h>

namespace slamware_ros_sdk {

    //////////////////////////////////////////////////////////////////////////

    ServerRobotDeviceInfoWorker::ServerRobotDeviceInfoWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : super_t(pRosSdkServer, wkName, triggerInterval)
    {
        auto& nhRos = rosNodeHandle();
        pubRobotDeviceInfo_ = nhRos.advertise<RobotDeviceInfo>("robot_device_info", 1, true);
    }

    ServerRobotDeviceInfoWorker::~ServerRobotDeviceInfoWorker()
    {
        //
    }

    bool ServerRobotDeviceInfoWorker::reinitWorkLoop(slamware_platform_t& pltfm)
    {
        if (!this->super_t::reinitWorkLoop(pltfm))
            return false;
        isWorkLoopInitOk_ = false;

        auto wkDat = mutableWorkData();
        auto& msgRobotDeviceInfo = wkDat->robotDeviceInfo;

        const auto devInfo = pltfm.getDeviceInfo();
        msgRobotDeviceInfo.device_id = devInfo.deviceID();
        msgRobotDeviceInfo.model_id = devInfo.modelID();
        msgRobotDeviceInfo.model_name = devInfo.modelName();
        msgRobotDeviceInfo.manufacturer_id = devInfo.manufacturerID();
        msgRobotDeviceInfo.manufacturer_name = devInfo.manufacturerName();
        msgRobotDeviceInfo.hardware_version = devInfo.hardwareVersion();
        msgRobotDeviceInfo.software_version = devInfo.softwareVersion();

        msgRobotDeviceInfo.sdp_version = pltfm.getSDPVersion();
        msgRobotDeviceInfo.sdk_version = pltfm.getSDKVersion();

        pubRobotDeviceInfo_.publish(msgRobotDeviceInfo);
        ROS_INFO("device_id: %s, hardware_version: %s, software_version: %s, sdp_version: %s, sdk_version: %s."
            , msgRobotDeviceInfo.device_id.c_str()
            , msgRobotDeviceInfo.hardware_version.c_str()
            , msgRobotDeviceInfo.software_version.c_str()
            , msgRobotDeviceInfo.sdp_version.c_str()
            , msgRobotDeviceInfo.sdk_version.c_str()
            );
        isWorkLoopInitOk_ = true;
        return isWorkLoopInitOk_;
    }

    void ServerRobotDeviceInfoWorker::doPerform(slamware_platform_t& /*pltfm*/)
    {
        // do nothing
    }

    //////////////////////////////////////////////////////////////////////////

    ServerOdometryWorker::ServerOdometryWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
        , isOdoPoseFeatureSupported_(false)
        , isSpeedFeatureSupported_(false)
    {
        const auto& srvParams = serverParams();
        auto& nhRos = rosNodeHandle();
        pubOdometry_ = nhRos.advertise<nav_msgs::Odometry>(srvParams.odom_topic, 10);
    }

    ServerOdometryWorker::~ServerOdometryWorker()
    {
        //
    }

    bool ServerOdometryWorker::reinitWorkLoop(slamware_platform_t& pltfm)
    {
        if (!this->ServerWorkerBase::reinitWorkLoop(pltfm))
            return false;

        const auto& srvParams = serverParams();
        if (srvParams.pub_accumulate_odometry)
        {
            try
            {
                rpos::core::Pose odometryPose = pltfm.getOdoPose();
                isOdoPoseFeatureSupported_ = true;
            }
            catch (const rpos::robot_platforms::UnsupportedCommandException& excp)
            {
                isOdoPoseFeatureSupported_ = false;
                ROS_WARN("worker: %s, reinitWorkLoop(), exception: %s.", getWorkerName().c_str(), excp.what());
            }
        }

        try
        {
            rpos::message::base::MotionRequest robotSpeed = pltfm.getSpeed();
            isSpeedFeatureSupported_ = true;
        }
        catch (const rpos::robot_platforms::UnsupportedCommandException& excp)
        {
            isSpeedFeatureSupported_ = false;
            ROS_WARN("worker: %s, reinitWorkLoop(), exception: %s.", getWorkerName().c_str(), excp.what());
        }

        isWorkLoopInitOk_ = true;
        return isWorkLoopInitOk_;
    }

    void ServerOdometryWorker::doPerform(slamware_platform_t& pltfm)
    {
        const auto& srvParams = serverParams();
        auto& tfBrdcst = tfBroadcaster();
        auto wkDat = mutableWorkData();

        rpos::core::Pose odometryPose;
        if (srvParams.pub_accumulate_odometry && isOdoPoseFeatureSupported_)
        {
            odometryPose = pltfm.getOdoPose();
        }
        else
        {
            // send TF transform
            if (srvParams.fixed_odom_map_tf) // only for debug rosrun
            {
                tf::Transform tfIdenty;
                tfIdenty.setOrigin(tf::Vector3 (0.0, 0.0, 0.0));
                tfIdenty.setRotation(tf::Quaternion(0, 0, 0, 1));

                tfBrdcst.sendTransform(tf::StampedTransform(tfIdenty, ros::Time::now(), srvParams.map_frame, srvParams.odom_frame));
                //tfBrdcst.sendTransform(tf::StampedTransform(tfIdenty, ros::Time::now(), srvParams.robot_frame, srvParams.laser_frame));
            }
            odometryPose = pltfm.getPose();
            // publish odom transform
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(odometryPose.x(), odometryPose.y(), 0.0));
            tf::Quaternion q = tf::createQuaternionFromYaw(odometryPose.yaw());
            transform.setRotation(q);
            tfBrdcst.sendTransform(tf::StampedTransform(transform, ros::Time::now(), srvParams.odom_frame, srvParams.robot_frame));
        }

        nav_msgs::Odometry msgOdometry;
        msgOdometry.header.frame_id = srvParams.odom_frame;
        msgOdometry.header.stamp = ros::Time::now();
        sltcToRosMsg(odometryPose, msgOdometry.pose.pose);
        if (isSpeedFeatureSupported_)
        {
            const rpos::message::base::MotionRequest robotSpeed = pltfm.getSpeed();
            msgOdometry.twist.twist.linear.x = robotSpeed.vx();
            msgOdometry.twist.twist.linear.y = robotSpeed.vy();
            msgOdometry.twist.twist.angular.z = robotSpeed.omega();
        }
        pubOdometry_.publish(msgOdometry);
    }

    //////////////////////////////////////////////////////////////////////////

    ServerRobotPoseWorker::ServerRobotPoseWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
    {
        const auto& srvParams = serverParams();
        auto& nhRos = rosNodeHandle();
        pubRobotPose_ = nhRos.advertise<geometry_msgs::PoseStamped>(srvParams.robot_pose_topic, 10);
    }

    ServerRobotPoseWorker::~ServerRobotPoseWorker()
    {
        //
    }

    void ServerRobotPoseWorker::doPerform(slamware_platform_t& pltfm)
    {
        const auto& srvParams = serverParams();
        auto& tfBrdcst = tfBroadcaster();
        auto wkDat = mutableWorkData();

        // send TF transform
        if (srvParams.fixed_odom_map_tf) // only for debug rosrun
        {
            tf::Transform tfIdenty;
            tfIdenty.setOrigin(tf::Vector3 (0.0, 0.0, 0.0));
            tfIdenty.setRotation(tf::Quaternion(0, 0, 0, 1));

            tfBrdcst.sendTransform(tf::StampedTransform(tfIdenty, ros::Time::now(), srvParams.map_frame, srvParams.robot_pose_frame));
            //tfBrdcst.sendTransform(tf::StampedTransform(tfIdenty, ros::Time::now(), srvParams.robot_frame, srvParams.laser_frame));
        }

        // check power
        //int battPercentage = pltfm.getBatteryPercentage();
        //if (battPercentage < 10)
        //    std::cout << "lower power!! Battery: " << battPercentage << "%." << std::endl;

        //const rpos::core::Location location = pltfm.getLocation();
        const rpos::core::Pose robotPose = pltfm.getPose();
        wkDat->robotPose = robotPose;

        // publish robot_pose transform
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(robotPose.x(), robotPose.y(), 0.0));
        tf::Quaternion q = tf::createQuaternionFromYaw(robotPose.yaw());
        transform.setRotation(q);
        tfBrdcst.sendTransform(tf::StampedTransform(transform, ros::Time::now(), srvParams.robot_pose_frame, srvParams.robot_frame));

        geometry_msgs::PoseStamped msgRobotPose;
        msgRobotPose.header.frame_id = srvParams.robot_pose_frame;
        msgRobotPose.header.stamp = ros::Time::now();
        sltcToRosMsg(robotPose, msgRobotPose.pose);
        pubRobotPose_.publish(msgRobotPose);
    }

    //////////////////////////////////////////////////////////////////////////

    ServerExploreMapUpdateWorker::ServerExploreMapUpdateWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : super_t(pRosSdkServer, wkName, triggerInterval)
        , shouldReinitMap_(true)
        , enableUpdateMap_(true)
    {
    }

    ServerExploreMapUpdateWorker::~ServerExploreMapUpdateWorker()
    {
        //
    }

    bool ServerExploreMapUpdateWorker::reinitWorkLoop(slamware_platform_t& pltfm)
    {
        if (!this->super_t::reinitWorkLoop(pltfm))
            return false;
        isWorkLoopInitOk_ = false;

        requestReinitMap_();

        isWorkLoopInitOk_ = true;
        return isWorkLoopInitOk_;
    }

    void ServerExploreMapUpdateWorker::doPerform(slamware_platform_t& pltfm)
    {
        const auto& srvParams = serverParams();
        auto wkDat = mutableWorkData();

        if (!checkToReinitMap_(pltfm, wkDat))
            return;

        if (wkDat->syncMapRequested.load())
        {
            ROS_INFO("try to sync whole explore map.");
            if (syncWholeMap_(srvParams, pltfm, wkDat))
            {
                wkDat->syncMapRequested.store(false);
                ROS_INFO("whole explore map synchronized.");
            }
            else
            {
                ROS_WARN("failed to sync whole explore map.");
            }
            lastMapUpdateTime_ = clock_t::now();
            return;
        }

        updateMapNearRobot_(srvParams, pltfm, wkDat);
    }

    rpos::features::location_provider::Map ServerExploreMapUpdateWorker::getMapByPltfm_(slamware_platform_t& pltfm, const rpos::core::RectangleF& area) const
    {
        try
        {
            return pltfm.getMap(rpos::features::location_provider::MapTypeBitmap8Bit, area, rpos::features::location_provider::EXPLORERMAP);
        }
        catch (const rpos::robot_platforms::OperationFailException&)
        {
            //
        }
        return rpos::features::location_provider::Map();
    }

    void ServerExploreMapUpdateWorker::requestReinitMap_()
    {
        shouldReinitMap_ = true;
    }

    bool ServerExploreMapUpdateWorker::checkToReinitMap_(slamware_platform_t& pltfm, const ServerWorkData_Ptr& wkDat)
    {
        if (!shouldReinitMap_)
            return true;

        ROS_INFO("try to reinit explore map.");
        wkDat->syncMapRequested.store(true);

        const float fHalfWH = 0.5f;
        const float fWH = fHalfWH + fHalfWH;
        const auto tArea = rpos::core::RectangleF(-fHalfWH, -fHalfWH, fWH, fWH);
        auto hMap = getMapByPltfm_(pltfm, tArea);
        if (!hMap)
        {
            ROS_WARN("failed get map when init explore map.");
            return false;
        }
        const auto& mapResolution = hMap.getMapResolution();
        wkDat->exploreMapHolder.reinit(mapResolution.x());

        ROS_INFO("explore map initialized, resolution: %f, moreCellCntToExtend: %d."
            , mapResolution.x()
            , wkDat->exploreMapHolder.getMoreCellCountToExtend()
            );
        shouldReinitMap_ = false;
        enableUpdateMap_ = true;
        lastMapUpdateTime_ = clock_t::now();
        return true;
    }

    bool ServerExploreMapUpdateWorker::checkRecvResolution_(float recvResolution, const ServerWorkData_Ptr& wkDat)
    {
        const auto fResolution = wkDat->exploreMapHolder.resolution();
        if (rpos::system::types::fequal(fResolution, recvResolution))
            return true;

        ROS_ERROR("local resolution: %f, received resolution: %f, request reinit.", fResolution, recvResolution);
        requestReinitMap_();
        return false;
    }

    bool ServerExploreMapUpdateWorker::updateMapInCellIdxRect_(slamware_platform_t& pltfm
            , const rpos::core::RectangleI& reqIdxRect
            , const ServerWorkData_Ptr& wkDat
            )
    {
        const auto reqArea = wkDat->exploreMapHolder.calcAreaByCellIdxRect(reqIdxRect);
        auto hMap = getMapByPltfm_(pltfm, reqArea);
        if (!hMap)
            return false;

        const auto& mapResolution = hMap.getMapResolution();
        if (!checkRecvResolution_(mapResolution.x(), wkDat))
            return false;

        wkDat->exploreMapHolder.setMapData(hMap);
        return true;
    }

    bool ServerExploreMapUpdateWorker::syncWholeMap_(const ServerParams& srvParams
        , slamware_platform_t& pltfm
        , const ServerWorkData_Ptr& wkDat
        )
    {
        const float fOnceMaxWH = std::max<float>(16.0f, srvParams.map_sync_once_get_max_wh);
        const auto fResolution = wkDat->exploreMapHolder.resolution();
        const int iOnceMaxCellCntWH = static_cast<int>(std::round(fOnceMaxWH / fResolution));
        BOOST_ASSERT(0 < iOnceMaxCellCntWH);

        const auto knownArea = pltfm.getKnownArea(rpos::features::location_provider::MapTypeBitmap8Bit, rpos::features::location_provider::EXPLORERMAP);
        const auto knownCellIdxRect = wkDat->exploreMapHolder.calcRoundedCellIdxRect(knownArea);
        ROS_INFO("known area: ((%f, %f), (%f, %f)), cell rect: ((%d, %d), (%d, %d)), iOnceMaxCellCntWH: %d."
            , knownArea.x(), knownArea.y(), knownArea.width(), knownArea.height()
            , knownCellIdxRect.x(), knownCellIdxRect.y(), knownCellIdxRect.width(), knownCellIdxRect.height()
            , iOnceMaxCellCntWH
            );
        if (ServerMapHolder::sfIsCellIdxRectEmpty(knownCellIdxRect))
        {
            ROS_ERROR("sync map, knownCellIdxRect is empty.");
            return false;
        }

        wkDat->exploreMapHolder.clear();
        wkDat->exploreMapHolder.reserveByCellIdxRect(knownCellIdxRect);

        const int cellIdxXEnd = knownCellIdxRect.x() + knownCellIdxRect.width();
        const int cellIdxYEnd = knownCellIdxRect.y() + knownCellIdxRect.height();
        int cellIdxY = knownCellIdxRect.y();
        while (cellIdxY < cellIdxYEnd)
        {
            const int tmpRemY = cellIdxYEnd - cellIdxY;
            const int reqSizeY = std::min<int>(tmpRemY, iOnceMaxCellCntWH);
            //
            int cellIdxX = knownCellIdxRect.x();
            while (cellIdxX < cellIdxXEnd)
            {
                const int tmpRemX = cellIdxXEnd - cellIdxX;
                const int reqSizeX = std::min<int>(tmpRemX, iOnceMaxCellCntWH);
                //
                const auto reqIdxRect = rpos::core::RectangleI(cellIdxX, cellIdxY, reqSizeX, reqSizeY);
                if (!updateMapInCellIdxRect_(pltfm, reqIdxRect, wkDat))
                {
                    return false;
                }
                //
                cellIdxX += reqSizeX;
            }
            //
            cellIdxY += reqSizeY;
        }
        return true;
    }

    bool ServerExploreMapUpdateWorker::updateMapNearRobot_(const ServerParams& srvParams
            , slamware_platform_t& pltfm
            , const ServerWorkData_Ptr& wkDat
            )
    {
        if(wkDat->robotState.is_map_building_enabled)
        {
            enableUpdateMap_ = true;
            lastMapUpdateTime_ = clock_t::now();
        }
        else
        { 
            if (!enableUpdateMap_)
            {
                return true;
            }
            auto duration = clock_t::now() - lastMapUpdateTime_;
            if(boost::chrono::duration_cast<boost::chrono::milliseconds>(duration).count() > 5000)
            {
                enableUpdateMap_ = false;
                wkDat->exploreMapHolder.clear();
                ROS_INFO("in localization mode for 5 seconds, stop publishing map");
                return true;
            }
        }
        
        const float fHalfWH = std::max<float>(2.0f, srvParams.map_update_near_robot_half_wh);
        const float fWH = fHalfWH + fHalfWH;
        const auto nearRobotArea = rpos::core::RectangleF(static_cast<float>(wkDat->robotPose.x() - fHalfWH)
            , static_cast<float>(wkDat->robotPose.y() - fHalfWH)
            , fWH
            , fWH
            );
        const auto nearRobotIdxRect = wkDat->exploreMapHolder.calcMinBoundingCellIdxRect(nearRobotArea);

        const auto knownArea = pltfm.getKnownArea(rpos::features::location_provider::MapTypeBitmap8Bit, rpos::features::location_provider::EXPLORERMAP);
        const auto knownCellIdxRect = wkDat->exploreMapHolder.calcRoundedCellIdxRect(knownArea);
    #if 0
        ROS_INFO("known area: ((%f, %f), (%f, %f)), cell rect: ((%d, %d), (%d, %d))."
            , knownArea.x(), knownArea.y(), knownArea.width(), knownArea.height()
            , knownCellIdxRect.x(), knownCellIdxRect.y(), knownCellIdxRect.width(), knownCellIdxRect.height()
            );
    #endif
        if (ServerMapHolder::sfIsCellIdxRectEmpty(knownCellIdxRect))
        {
            ROS_ERROR("update map, knownCellIdxRect is empty, request sync map.");
            rosSdkServer()->requestSyncMap();
            return false;
        }

        const auto reqIdxRect = ServerMapHolder::sfIntersectionOfCellIdxRect(nearRobotIdxRect, knownCellIdxRect);
        if (ServerMapHolder::sfIsCellIdxRectEmpty(reqIdxRect))
        {
            ROS_WARN("knownCellIdxRect: ((%d, %d), (%d, %d)), nearRobotIdxRect: ((%d, %d), (%d, %d)), intersection is empty."
                , knownCellIdxRect.x(), knownCellIdxRect.y(), knownCellIdxRect.width(), knownCellIdxRect.height()
                , nearRobotIdxRect.x(), nearRobotIdxRect.y(), nearRobotIdxRect.width(), nearRobotIdxRect.height()
                );
            return false;
        }
        return updateMapInCellIdxRect_(pltfm, reqIdxRect, wkDat);
    }

    //////////////////////////////////////////////////////////////////////////

    ServerExploreMapPublishWorker::ServerExploreMapPublishWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
    {
        const auto& srvParams = serverParams();
        auto& nhRos = rosNodeHandle();
        pubMapDat_ = nhRos.advertise<nav_msgs::OccupancyGrid>(srvParams.map_topic, 1, true);
        pubMapInfo_ = nhRos.advertise<nav_msgs::MapMetaData>(srvParams.map_info_topic, 1, true);
    }

    ServerExploreMapPublishWorker::~ServerExploreMapPublishWorker()
    {
        //
    }

    void ServerExploreMapPublishWorker::doPerform(slamware_platform_t& /*pltfm*/)
    {
        const auto& srvParams = serverParams();
        auto wkDat = workData();

        if (wkDat->exploreMapHolder.isMapDataEmpty())
        {
            //ROS_WARN("current explore map data is empty.");
            return;
        }

        nav_msgs::GetMap::Response msgMap;
        wkDat->exploreMapHolder.fillRosMapMsg(msgMap);

        // Set the header information on the map
        msgMap.map.header.stamp = ros::Time::now();
        msgMap.map.header.frame_id = srvParams.map_frame;

        pubMapDat_.publish(msgMap.map);
        pubMapInfo_.publish(msgMap.map.info);
    }

    //////////////////////////////////////////////////////////////////////////

    ServerLaserScanWorker::ServerLaserScanWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
        , compensatedAngleCnt_(360u)
        , absAngleIncrement_(C_FLT_2PI / compensatedAngleCnt_)
        , latestLidarStartTimestamp_(0)
    {
        const auto& srvParams = serverParams();
        auto& nhRos = rosNodeHandle();
        pubLaserScan_ = nhRos.advertise<sensor_msgs::LaserScan>(srvParams.scan_topic, 10);
    }

    ServerLaserScanWorker::~ServerLaserScanWorker()
    {
        //
    }

    void ServerLaserScanWorker::doPerform(slamware_platform_t& pltfm)
    {
        const auto& srvParams = serverParams();
        auto& tfBrdcst = tfBroadcaster();
        auto wkDat = workData();

        ros::Time startScanTime(0.0), endScanTime(0.0);
        rpos::features::system_resource::LaserScan tLs;
        if (!srvParams.raw_ladar_data)
        {
            startScanTime = ros::Time::now();
            tLs = pltfm.getLaserScan();
            endScanTime = ros::Time::now();
        }
        else
        {
            tLs = pltfm.getRawLaserScan();
            std::uint64_t lidarStartTimestamp = tLs.getStartTimestamp();
            if (latestLidarStartTimestamp_ == lidarStartTimestamp)
            {
                // do not send same lidar date
                return;
            }
            else
            {
                latestLidarStartTimestamp_ = lidarStartTimestamp;
            }
            startScanTime = ros::Time(lidarStartTimestamp / 1000000.0);
            endScanTime = ros::Time(tLs.getEndTimestamp() / 1000000.0);
        }
        double dblScanDur = (endScanTime - startScanTime).toSec();

        const auto& points = tLs.getLaserPoints();
        if (points.size() < 2)
        {
            ROS_ERROR("laser points count: %u, too small, skip publish.", (unsigned int)points.size());
            return;
        }

        const auto laserPose = (tLs.getHasPose() ? tLs.getLaserPointsPose() : wkDat->robotPose);
        //ROS_INFO("has laser pose: %s, robotPose: ((%f, %f), (%f)), laserPose: ((%f, %f), (%f))."
        //    , (tLs.getHasPose() ? "true" : "false")
        //    , wkDat->robotPose.x(), wkDat->robotPose.y(), wkDat->robotPose.yaw()
        //    , laserPose.x(), laserPose.y(), laserPose.yaw()
        //    );

        sensor_msgs::LaserScan msgScan;
        msgScan.header.stamp = startScanTime;
        msgScan.header.frame_id = srvParams.laser_frame;
        fillRangeMinMaxInMsg_(points, msgScan);

        bool isLaserDataReverse = false;
        if ((points.back().angle() < points.front().angle() && !srvParams.ladar_data_clockwise) ||
            (points.back().angle() > points.front().angle() && srvParams.ladar_data_clockwise))
        {
            isLaserDataReverse = true;
        }

        if (srvParams.angle_compensate)
        {
            compensateAndfillRangesInMsg_(points, srvParams.ladar_data_clockwise, isLaserDataReverse, msgScan);
        }
        else
        {
            fillOriginalRangesInMsg_(points, isLaserDataReverse, msgScan);
        }
        BOOST_ASSERT(2 <= msgScan.ranges.size());
        msgScan.scan_time = dblScanDur;
        msgScan.time_increment = dblScanDur / (double)(msgScan.ranges.size() - 1);

        {
            tf::Transform laserTrans;
            laserTrans.setOrigin(tf::Vector3(laserPose.x(), laserPose.y(), 0.0));
            tf::Quaternion qLaserTrans = tf::createQuaternionFromYaw(laserPose.yaw());
            laserTrans.setRotation(qLaserTrans);
            tfBrdcst.sendTransform(tf::StampedTransform(laserTrans, startScanTime, srvParams.map_frame, srvParams.laser_frame));
        }
        pubLaserScan_.publish(msgScan);
    }

    void ServerLaserScanWorker::fillRangeMinMaxInMsg_(const std::vector<rpos::core::LaserPoint>& laserPoints
            , sensor_msgs::LaserScan& msgScan
            ) const
    {
        msgScan.range_min = std::numeric_limits<float>::infinity();
        msgScan.range_max = 0.0f;
        for (auto cit = laserPoints.cbegin(), citEnd = laserPoints.cend(); citEnd != cit; ++cit)
        {
            if (cit->valid())
            {
                const float tmpDist = cit->distance();
                
                if (tmpDist < msgScan.range_min)
                {
                    msgScan.range_min = std::max<float>(0.0f, tmpDist);
                }

                if (msgScan.range_max < tmpDist)
                {
                    msgScan.range_max = tmpDist;
                }
            }
        }
    }

    float ServerLaserScanWorker::calcAngleInNegativePiToPi_(float angle) const
    {
        float fRes = std::fmod(angle + C_FLT_PI, C_FLT_2PI);
        if (fRes < 0.0f)
            fRes += C_FLT_2PI;
        fRes -= C_FLT_PI;

        if (fRes < -C_FLT_PI)
            fRes = -C_FLT_PI;
        else if (C_FLT_PI <= fRes)
            fRes = -C_FLT_PI;
        return fRes;
    }

    std::uint32_t ServerLaserScanWorker::calcCompensateDestIndexBySrcAngle_(float srcAngle
        , bool isAnglesReverse
        ) const
    {
        BOOST_ASSERT(-C_FLT_PI <= srcAngle && srcAngle < C_FLT_PI);
        
        float fDiff = (isAnglesReverse ? (C_FLT_PI - srcAngle) : (srcAngle + C_FLT_PI));
        fDiff = std::max<float>(0.0f, fDiff);
        fDiff = std::min<float>(fDiff, C_FLT_2PI);

        std::uint32_t destIdx = static_cast<std::uint32_t>(std::round(fDiff / absAngleIncrement_));
        if (compensatedAngleCnt_ <= destIdx)
            destIdx = 0;
        return destIdx;
    }

    bool ServerLaserScanWorker::isSrcAngleMoreCloseThanOldSrcAngle_(float srcAngle, float destAngle, float oldSrcAngle) const
    {
        BOOST_ASSERT(-C_FLT_PI <= srcAngle && srcAngle < C_FLT_PI);
        BOOST_ASSERT(-C_FLT_PI <= destAngle && destAngle < C_FLT_PI);
        BOOST_ASSERT(-C_FLT_PI <= oldSrcAngle && oldSrcAngle < C_FLT_PI);

        float newDiff = std::abs(destAngle - srcAngle);
        if (C_FLT_2PI <= newDiff)
            newDiff = 0.0f;
        else if (C_FLT_PI < newDiff)
            newDiff = C_FLT_2PI - newDiff;

        float oldDiff = std::abs(destAngle - oldSrcAngle);
        if (C_FLT_2PI <= oldDiff)
            oldDiff = 0.0f;
        else if (C_FLT_PI < oldDiff)
            oldDiff = C_FLT_2PI - oldDiff;

        return (newDiff < oldDiff);
    }

    void ServerLaserScanWorker::compensateAndfillRangesInMsg_(const std::vector<rpos::core::LaserPoint>& laserPoints
            , bool isClockwise
            , bool isLaserDataReverse
            , sensor_msgs::LaserScan& msgScan
            ) const
    {
        BOOST_ASSERT(2 <= laserPoints.size());

        msgScan.ranges.clear();
        msgScan.intensities.clear();
        msgScan.ranges.resize(compensatedAngleCnt_, std::numeric_limits<float>::infinity());
        msgScan.intensities.resize(compensatedAngleCnt_, 0.0);

        if (!isClockwise)
        {
            msgScan.angle_min = -C_FLT_PI;
            msgScan.angle_max = C_FLT_PI - absAngleIncrement_;
            msgScan.angle_increment = absAngleIncrement_;
        }
        else
        {
            msgScan.angle_min = C_FLT_PI;
            msgScan.angle_max = (-C_FLT_PI + absAngleIncrement_); 
            msgScan.angle_increment = -absAngleIncrement_;
        }

        std::vector<float> tmpSrcAngles(compensatedAngleCnt_);
        if (!isLaserDataReverse)
        {
            for (int i = 0; i < laserPoints.size(); ++i)
            {
                compensateAndfillRangeInMsg_(laserPoints[i], isClockwise, msgScan, tmpSrcAngles);
            }
        }
        else
        {
            for (int i = laserPoints.size() - 1; i >= 0; --i)
            {
                compensateAndfillRangeInMsg_(laserPoints[i], isClockwise, msgScan, tmpSrcAngles);
            }
        }
    }

    void ServerLaserScanWorker::compensateAndfillRangeInMsg_(const rpos::core::LaserPoint& laserPoint
            , bool isClockwise
            , sensor_msgs::LaserScan& msgScan
            , std::vector<float>& tmpSrcAngles
            ) const
    {
        if (laserPoint.valid())
        {
            const float srcAngle = calcAngleInNegativePiToPi_(laserPoint.angle());
            const std::uint32_t destIdx = calcCompensateDestIndexBySrcAngle_(srcAngle, isClockwise);
            BOOST_ASSERT(destIdx < compensatedAngleCnt_);
            const float destAngle = calcAngleInNegativePiToPi_(msgScan.angle_min + msgScan.angle_increment * destIdx);

            const bool shouldWrite = (std::isinf(msgScan.ranges[destIdx])
                || isSrcAngleMoreCloseThanOldSrcAngle_(srcAngle, destAngle, tmpSrcAngles[destIdx])
                );
            if (shouldWrite)
            {
                msgScan.ranges[destIdx] = laserPoint.distance();
                msgScan.intensities[destIdx] = laserPoint.quality();
                tmpSrcAngles[destIdx] = srcAngle;
            }
        }
    }

    void ServerLaserScanWorker::fillOriginalRangesInMsg_(const std::vector<rpos::core::LaserPoint>& laserPoints
            , bool isLaserDataReverse
            , sensor_msgs::LaserScan& msgScan
            ) const
    {
        msgScan.intensities.resize(laserPoints.size());
        msgScan.ranges.resize(laserPoints.size());

        int index = 0;
        if (!isLaserDataReverse)
        {
            for (int i = 0; i < laserPoints.size(); ++i)
            {
                fillOriginalRangeInMsg_(laserPoints[i], index++, msgScan);
            }
            msgScan.angle_min =  laserPoints.front().angle();
            msgScan.angle_max =  laserPoints.back().angle();
            msgScan.angle_increment = (msgScan.angle_max - msgScan.angle_min) / (double)(msgScan.ranges.size() - 1);
        }
        else
        {
            for (int i = laserPoints.size() - 1; i >= 0; --i)
            {
                fillOriginalRangeInMsg_(laserPoints[i], index++, msgScan);
            }
            msgScan.angle_min =  laserPoints.back().angle();
            msgScan.angle_max =  laserPoints.front().angle();
            msgScan.angle_increment = (msgScan.angle_max - msgScan.angle_min) / (double)(msgScan.ranges.size() - 1);
        }
    }

    void ServerLaserScanWorker::fillOriginalRangeInMsg_(const rpos::core::LaserPoint& laserPoint
            , int index
            , sensor_msgs::LaserScan& msgScan
            ) const
    {
        if (!laserPoint.valid())
        {
            msgScan.ranges[index] = std::numeric_limits<float>::infinity();
            msgScan.intensities[index] = 0.0;
        }
        else
        {
            msgScan.ranges[index] = laserPoint.distance();
            msgScan.intensities[index] = laserPoint.quality();
        }
    }

    //////////////////////////////////////////////////////////////////////////

    ServerBasicSensorsInfoWorker::ServerBasicSensorsInfoWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
    {
        const auto& srvParams = serverParams();
        auto& nhRos = rosNodeHandle();
        pubSensorsInfo_ = nhRos.advertise<slamware_ros_sdk::BasicSensorInfoArray>(srvParams.basic_sensors_info_topic, 1, true);
    }

    ServerBasicSensorsInfoWorker::~ServerBasicSensorsInfoWorker()
    {
        //
    }

    void ServerBasicSensorsInfoWorker::doPerform(slamware_platform_t& pltfm)
    {
        auto wkDat = mutableWorkData();
        
        {
            sensors_info_map_t sensorsInfo;
            if (!getSensorsInfo_(pltfm, sensorsInfo))
            {
                ROS_ERROR("failed to get sensors info from slamware platform.");
                return;
            }
            if (isSensorsInfoAsTheSame_(wkDat->sensorsInfo, sensorsInfo))
                return;

            wkDat->sensorsInfo.swap(sensorsInfo);
            sltcToRosMsg(wkDat->sensorsInfo, wkDat->rosBasicSensorsInfo);
        }

        slamware_ros_sdk::BasicSensorInfoArray msgSensorsInfo;
        const size_t sensorsCnt = wkDat->rosBasicSensorsInfo.size();
        msgSensorsInfo.sensors_info.resize(sensorsCnt);
        size_t t = 0;
        for (auto cit = wkDat->rosBasicSensorsInfo.cbegin(), citEnd = wkDat->rosBasicSensorsInfo.cend(); citEnd != cit; ++cit, ++t)
        {
            const auto& rcRosInfo = cit->second;
            BOOST_ASSERT(rcRosInfo.id == cit->first);
            msgSensorsInfo.sensors_info[t] = rcRosInfo;
        }
        pubSensorsInfo_.publish(msgSensorsInfo);
        ROS_INFO("new sensors info published, sensorsCnt: %u.", (unsigned int)sensorsCnt);
    }

    bool ServerBasicSensorsInfoWorker::getSensorsInfo_(slamware_platform_t& pltfm, sensors_info_map_t& sensorsInfo) const
    {
        std::vector<rpos::features::impact_sensor::ImpactSensorInfo> vSensorsInfo;
        if (!pltfm.getSensors(vSensorsInfo))
            return false;

        sensorsInfo.clear();
        for (auto cit = vSensorsInfo.cbegin(), citEnd = vSensorsInfo.cend(); citEnd != cit; ++cit)
        {
            sensorsInfo.insert(sensors_info_map_t::value_type(cit->id, *cit));
        }
        return true;
    }

    bool ServerBasicSensorsInfoWorker::isSensorsInfoAsTheSame_(const sensors_info_map_t& sensorsInfoA, const sensors_info_map_t& sensorsInfoB) const
    {
        const size_t sensorsCnt = sensorsInfoA.size();
        if (sensorsInfoB.size() != sensorsCnt)
        {
            ROS_INFO("sensorsCnt: %u --> %u.", (unsigned int)sensorsCnt, (unsigned int)sensorsInfoB.size());
            return false;
        }

        for (auto citA = sensorsInfoA.cbegin(), citAEnd = sensorsInfoA.cend(); citAEnd != citA; ++citA)
        {
            const auto& rcInfoA = citA->second;
            BOOST_ASSERT(rcInfoA.id == citA->first);

            auto citB = sensorsInfoB.find(rcInfoA.id);
            if (sensorsInfoB.cend() == citB)
            {
                ROS_INFO("sensor id %d has gone away.", rcInfoA.id);
                return false;
            }

            const auto& rcInfoB = citB->second;
            BOOST_ASSERT(rcInfoB.id == citB->first);

            if (rcInfoA.coreSensorType != rcInfoB.coreSensorType)
            {
                ROS_INFO("sensor id %d, core type: %d --> %d.", rcInfoA.id, (int)rcInfoA.coreSensorType, (int)rcInfoB.coreSensorType);
                return false;
            }
            if (rcInfoA.type != rcInfoB.type)
            {
                ROS_INFO("sensor id %d, impact type: %d --> %d.", rcInfoA.id, (int)rcInfoA.type, (int)rcInfoB.type);
                return false;
            }
            if (!(rcInfoA.pose == rcInfoB.pose))
            {
                ROS_INFO("sensor id %d, pose changed.", rcInfoA.id);
                return false;
            }
        }
        return true;
    }

    //////////////////////////////////////////////////////////////////////////

    ServerBasicSensorsValuesWorker::ServerBasicSensorsValuesWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
    {
        const auto& srvParams = serverParams();
        auto& nhRos = rosNodeHandle();
        pubSensorsValues_ = nhRos.advertise<slamware_ros_sdk::BasicSensorValueDataArray>(srvParams.basic_sensors_values_topic, 5);
    }

    ServerBasicSensorsValuesWorker::~ServerBasicSensorsValuesWorker()
    {
        //
    }

    void ServerBasicSensorsValuesWorker::doPerform(slamware_platform_t& pltfm)
    {
        auto wkDat = mutableWorkData();
        
        sensors_values_map_t sensorsValues;
        if (!getSensorsValues_(pltfm, sensorsValues))
        {
            ROS_ERROR("failed to get sensors values from slamware platform.");
            return;
        }

        slamware_ros_sdk::BasicSensorValueDataArray msgSensorsValues;
        const size_t sensorsCnt = wkDat->rosBasicSensorsInfo.size();
        msgSensorsValues.values_data.resize(sensorsCnt);
        size_t t = 0;
        for (auto citInfo = wkDat->rosBasicSensorsInfo.cbegin(), citInfoEnd = wkDat->rosBasicSensorsInfo.cend(); citInfoEnd != citInfo; ++citInfo, ++t)
        {
            const auto& rcInfo = citInfo->second;
            BOOST_ASSERT(rcInfo.id == citInfo->first);

            auto& destValDat = msgSensorsValues.values_data[t];
            destValDat.info = rcInfo;
            auto& destVal = destValDat.value;
            
            sensors_values_map_t::const_iterator citSrcVal = sensorsValues.find(rcInfo.id);
            if (sensorsValues.cend() == citSrcVal)
                continue;

            const auto& rcSrcVal = citSrcVal->second;
            destVal.value = rcSrcVal.value;
            destVal.is_in_impact = isSensorValueImpact_(rcInfo, rcSrcVal);
        }
        pubSensorsValues_.publish(msgSensorsValues);
    }

    bool ServerBasicSensorsValuesWorker::getSensorsValues_(slamware_platform_t& pltfm, sensors_values_map_t& sensorsValues) const
    {
        sensorsValues.clear();
        return pltfm.getSensorValues(sensorsValues);
    }

    bool ServerBasicSensorsValuesWorker::isSensorValueImpact_(const BasicSensorInfo& basicInfo, const sensor_value_t& sensorVal) const
    {
        switch (basicInfo.impact_type.type)
        {
        case ImpactType::DIGITAL:
            return ServerWorkData::sfIsDigitalSensorValueImpact(sensorVal.value);
        case ImpactType::ANALOG:
            {
                switch (basicInfo.sensor_type.type)
                {
                case SensorType::SONAR:
                    return true;
                default:
                    break;
                }
            }
            return false;
        default:
            break;
        }
        return false;
    }

    //////////////////////////////////////////////////////////////////////////

    ServerPlanPathWorker::ServerPlanPathWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
    {
        const auto& srvParams = serverParams();
        auto& nhRos = rosNodeHandle();
        pubPlanPath_ = nhRos.advertise<nav_msgs::Path>(srvParams.path_topic, 10);
    }

    ServerPlanPathWorker::~ServerPlanPathWorker()
    {
        //
    }

    void ServerPlanPathWorker::doPerform(slamware_platform_t& pltfm)
    {
        const auto& srvParams = serverParams();
        
        nav_msgs::Path msgPath;
        msgPath.poses.resize(0);
        msgPath.header.frame_id = srvParams.map_frame;

        rpos::actions::MoveAction actMove = pltfm.getCurrentAction();
        if (!actMove)
        {
            msgPath.header.stamp = ros::Time::now();
            pubPlanPath_.publish(msgPath);
            return;
        }
        rpos::features::motion_planner::Path remPath = actMove.getRemainingPath();
        if (!remPath)
        {
            msgPath.header.stamp = ros::Time::now();
            pubPlanPath_.publish(msgPath);
            return;
        }

        const auto& remPathPoints = remPath.getPoints();
        msgPath.poses.resize(remPathPoints.size());
        msgPath.header.stamp = ros::Time::now();
        for (size_t i = 0; i < remPathPoints.size(); ++i)
        {
            geometry_msgs::PoseStamped tPoseStamp;
            tPoseStamp.header.frame_id = srvParams.map_frame;
            tPoseStamp.header.stamp = ros::Time::now();
            sltcToRosMsg(remPathPoints[i], tPoseStamp.pose.position);
            tPoseStamp.pose.orientation.x = 0;
            tPoseStamp.pose.orientation.y = 0;
            tPoseStamp.pose.orientation.z = 0;
            tPoseStamp.pose.orientation.w = 1;
            msgPath.poses[i] = tPoseStamp;
        }
        pubPlanPath_.publish(msgPath);
    }

    //////////////////////////////////////////////////////////////////////////

    ServerRobotBasicStateWorker::ServerRobotBasicStateWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
    {
        auto& nhRos = rosNodeHandle();
        pubRobotBasicState_ = nhRos.advertise<RobotBasicState>("robot_basic_state", 1, true);
    }

    ServerRobotBasicStateWorker::~ServerRobotBasicStateWorker()
    {
        //
    }

    void ServerRobotBasicStateWorker::doPerform(slamware_platform_t& pltfm)
    {
        auto wkDat = mutableWorkData();
        
        wkDat->robotState.is_map_building_enabled = pltfm.getMapUpdate(rpos::features::location_provider::EXPLORERMAP);
        wkDat->robotState.is_localization_enabled = pltfm.getMapLocalization();
        
        wkDat->robotState.localization_quality = pltfm.getLocalizationQuality();

        wkDat->robotState.board_temperature = pltfm.getBoardTemperature();

        const auto pwrStatus = pltfm.getPowerStatus();
        wkDat->robotState.battery_percentage = pwrStatus.batteryPercentage;
        wkDat->robotState.is_dc_in = pwrStatus.isDCConnected;
        wkDat->robotState.is_charging = pwrStatus.isCharging;

        pubRobotBasicState_.publish(wkDat->robotState);
    }

    //////////////////////////////////////////////////////////////////////////

    ServerArtifactLinesWorker::ServerArtifactLinesWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        , const params_t& rcParams
        )
        : super_t(pRosSdkServer, wkName, triggerInterval)
        , params_(rcParams)
        , sltcUsage_(rpos::features::artifact_provider::ArtifactUsageVirtualWall)
        , isFeatureSupported_(false)
    {
        rosMsgToSltc(params_.usage, sltcUsage_);
    }

    ServerArtifactLinesWorker::~ServerArtifactLinesWorker()
    {
        //
    }

    void ServerArtifactLinesWorker::resetOnWorkLoopBegin()
    {
        pubArtifactLines_= ros::Publisher();
        isFeatureSupported_ = false;

        this->super_t::resetOnWorkLoopBegin();
    }

    bool ServerArtifactLinesWorker::reinitWorkLoop(slamware_platform_t& pltfm)
    {
        if (!this->super_t::reinitWorkLoop(pltfm))
            return false;
        isWorkLoopInitOk_ = false;

        std::vector<rpos::core::Line> vLines;
        try
        {
            vLines = pltfm.getLines(sltcUsage_);
            isFeatureSupported_ = true;
        }
        catch (const rpos::robot_platforms::UnsupportedCommandException& excp)
        {
            isFeatureSupported_ = false;
            ROS_WARN("worker: %s, reinitWorkLoop(), usage: %d, exception: %s.", getWorkerName().c_str(), (int)params_.usage.usage, excp.what());
        }

        if (isFeatureSupported_)
        {
            Line2DFlt32Array msgLines;
            sltcToRosMsg(vLines, msgLines.lines);

            auto& nhRos = rosNodeHandle();
            pubArtifactLines_ = nhRos.advertise<Line2DFlt32Array>(params_.topic, params_.queueSize, params_.latch);
            pubArtifactLines_.publish(msgLines);
        }

        isWorkLoopInitOk_ = true;
        return isWorkLoopInitOk_;
    }

    void ServerArtifactLinesWorker::doPerform(slamware_platform_t& pltfm)
    {
        if (!isFeatureSupported_)
            return;

        const auto vLines = pltfm.getLines(sltcUsage_);
        
        Line2DFlt32Array msgLines;
        sltcToRosMsg(vLines, msgLines.lines);

        pubArtifactLines_.publish(msgLines);
    }

    //////////////////////////////////////////////////////////////////////////

    ServerReadEventsWorker::ServerReadEventsWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
    {
       
    }

    ServerReadEventsWorker::~ServerReadEventsWorker()
    {
        //
    }

    bool ServerReadEventsWorker::reinitWorkLoop(slamware_platform_t& pltfm)
    {
        if (!this->super_t::reinitWorkLoop(pltfm))
            return false;
        isWorkLoopInitOk_ = false;

        systemEventProvider_ = pltfm.createSystemEventProvider();
        if (!systemEventProvider_)
        {
            ROS_WARN("read events get event provider failed.");
            return isWorkLoopInitOk_;
        }

        isWorkLoopInitOk_ = true;
        return isWorkLoopInitOk_;
    }

    void ServerReadEventsWorker::doPerform(slamware_platform_t& pltfm)
    {
        std::vector<rpos::core::DiagnosisInfoInternalSystemEvent> events;
        systemEventProvider_->readEvents(events);

        for (size_t i = 0; i < events.size(); ++i)
        {
            if (rpos::core::InternalSystemEvent::InternalSystemEventMapLoopClosure == events[i].internalSystemEvent)
            {
                rosSdkServer()->requestSyncMap();
                ROS_INFO("receive event: loop closure, sync map.");
                return;
            }
            else if (rpos::core::InternalSystemEvent::InternalSystemEventSetMapDone == events[i].internalSystemEvent)
            {
                rosSdkServer()->requestSyncMap();
                ROS_INFO("receive event: set map done, sync map.");
                return;
            }
        }
    }

    //////////////////////////////////////////////////////////////////////////
    
    ServerImuRawDataWorker::ServerImuRawDataWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
        , isFeatureSupported_(false)
    {
        const auto& srvParams = serverParams();
        auto& nhRos = rosNodeHandle();
        pubImuRawData_ = nhRos.advertise<sensor_msgs::Imu>(srvParams.imu_raw_data_topic, 10);
        pubImuRawMagData_ = nhRos.advertise<sensor_msgs::MagneticField>(srvParams.imu_raw_mag_data_topic, 10);
    }

    ServerImuRawDataWorker::~ServerImuRawDataWorker()
    {
        //
    }

    bool ServerImuRawDataWorker::reinitWorkLoop(slamware_platform_t& pltfm)
    {
        if (!this->super_t::reinitWorkLoop(pltfm))
            return false;

        try
        {
            rpos::core::IMURawStandardUnitData imuRawData = pltfm.getImuRawData();
            isFeatureSupported_ = true;
        }
        catch (const rpos::robot_platforms::UnsupportedCommandException& excp)
        {
            isFeatureSupported_ = false;
            ROS_WARN("worker: %s, reinitWorkLoop(), exception: %s.", getWorkerName().c_str(), excp.what());
        }

        isWorkLoopInitOk_ = true;
        return isWorkLoopInitOk_;
    }

    void ServerImuRawDataWorker::doPerform(slamware_platform_t& pltfm)
    {
        if (!isFeatureSupported_)
            return;
        
        rpos::core::IMURawStandardUnitData imuRawData = pltfm.getImuRawData();

        sensor_msgs::Imu imuRawDataRos;
        imuRawDataRos.linear_acceleration.x = imuRawData.acc_x;
        imuRawDataRos.linear_acceleration.y = imuRawData.acc_y;
        imuRawDataRos.linear_acceleration.z = imuRawData.acc_z;
        imuRawDataRos.angular_velocity.x = imuRawData.gyro_x;
        imuRawDataRos.angular_velocity.y = imuRawData.gyro_y;
        imuRawDataRos.angular_velocity.z = imuRawData.gyro_z;
        imuRawDataRos.header.stamp = ros::Time::now();
        pubImuRawData_.publish(imuRawDataRos);

        if (std::fabs(imuRawData.comp_x) > 1e-6 || std::fabs(imuRawData.comp_y) > 1e-6 || std::fabs(imuRawData.comp_z) > 1e-6)
        {
            sensor_msgs::MagneticField imuRawMagDataRos;
            imuRawMagDataRos.magnetic_field.x = imuRawData.comp_x;
            imuRawMagDataRos.magnetic_field.y = imuRawData.comp_y;
            imuRawMagDataRos.magnetic_field.z = imuRawData.comp_z;
            imuRawMagDataRos.header.stamp = ros::Time::now();
            pubImuRawMagData_.publish(imuRawMagDataRos);
        }
    }

    //////////////////////////////////////////////////////////////////////////
    
    RosConnectWorker::RosConnectWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
    {
        auto& nhRos = rosNodeHandle();
        pubRosConnect_ = nhRos.advertise<std_msgs::String>("state", 1); 
    }

    RosConnectWorker::~RosConnectWorker()
    {
        //
    }

    bool RosConnectWorker::reinitWorkLoop(slamware_platform_t& pltfm)
    {
        if (!this->super_t::reinitWorkLoop(pltfm))
            return false;
        
        isWorkLoopInitOk_ = true;
        return isWorkLoopInitOk_;
    }

    void RosConnectWorker::doPerform(slamware_platform_t& pltfm)
    {
        auto connectStatus = std_msgs::String();
        connectStatus.data = "connected";

        if (!pltfm)
        {
            connectStatus.data = "disconnected";
        }
        
        pubRosConnect_.publish(connectStatus);
        
    }

    //////////////////////////////////////////////////////////////////////////
    
}
