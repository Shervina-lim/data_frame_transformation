#pragma once
#ifndef TRANSFORM_HPP
#define TRANSFORM_HPP

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
// #include <tf2/convert.h>

class TransformData{


public:
    TransformData();
    ~TransformData();

private:
    void transformToBaseLink(const sensor_msgs::Imu::ConstPtr &imu_msg);
    void loadParam();

    ros::NodeHandle nh;
    ros::Subscriber sub_imu;
    ros::Publisher pub_imu;

    // use tf listener
    // tf::TransformListener listener;
    std::string imu_topic, new_topic, new_frame_id;
    std::vector<double> translation_xyz, rotation_rpy;
    Eigen::Vector3d trans, rot;
    bool unit_in_g;
};

#endif TRANSFORM_HPP