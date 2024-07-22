
#pragma once

#include <ros/ros.h>

#include <slamware_ros_sdk/utils.h>

#include <boost/shared_ptr.hpp>

namespace slamware_ros_sdk {

    extern const float C_FLT_PI;
    extern const float C_FLT_2PI;

    struct ServerParams
    {
        std::string ip_address;
        int robot_port;
        int reconn_wait_ms;

        bool angle_compensate;
        bool fixed_odom_map_tf;

        std::string robot_frame;
        std::string laser_frame;
        std::string map_frame;
        std::string odom_frame;
        std::string robot_pose_frame;

        float odometry_pub_period;

        float robot_pose_pub_period;
        float scan_pub_period;
        float map_update_period;
        float map_pub_period;
        float basic_sensors_info_update_period;
        float basic_sensors_values_pub_period;
        float path_pub_period;
        float robot_basic_state_pub_period;
        float virtual_walls_pub_period;
        float virtual_tracks_pub_period;

        float map_sync_once_get_max_wh;
        float map_update_near_robot_half_wh;

        float imu_raw_data_period;
        float event_period;

        std::string scan_topic;
        std::string odom_topic;
        std::string robot_pose_topic;
        std::string map_topic;
        std::string map_info_topic;
        std::string basic_sensors_info_topic;
        std::string basic_sensors_values_topic;
        std::string path_topic;

        std::string vel_control_topic;
        std::string goal_topic;

        std::string imu_raw_data_topic;
        std::string imu_raw_mag_data_topic;

        bool raw_ladar_data;
        bool ladar_data_clockwise;

        bool pub_accumulate_odometry;

        ServerParams();

        void resetToDefault();
        void setBy(const ros::NodeHandle& nhRos);
    };
    
}
