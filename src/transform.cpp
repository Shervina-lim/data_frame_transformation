#include "transform_imu_data/transform.hpp"

TransformData::TransformData(){
    ROS_INFO_STREAM("Loading params from yaml file");
    loadParam();
    // init sub and publisher
    pub_imu = nh.advertise<sensor_msgs::Imu>(new_topic, 1); 
    sub_imu = nh.subscribe<sensor_msgs::Imu>(imu_topic, 1, [this](const sensor_msgs::Imu::ConstPtr &imu_msg) {
        // ROS_INFO_STREAM("in callback");
        transformToBaseLink(imu_msg);        
    });
}
TransformData::~TransformData(){}

void TransformData::loadParam(){
 
    nh.getParam("imu_topic", imu_topic);
    ROS_INFO_STREAM("Loaded imu topic: " << imu_topic); 

    nh.getParam("new_topic", new_topic);
    ROS_INFO_STREAM("Loaded new topic: " << new_topic); 

    nh.getParam("new_frame_id", new_frame_id);
    ROS_INFO_STREAM("Loaded new frame id: " << new_frame_id); 

    nh.getParam("unit_in_g", unit_in_g);
    ROS_INFO_STREAM("Loaded unit in g (bool): " << unit_in_g); 
    
    nh.getParam("extrinsic_T", translation_xyz);
    ROS_INFO_STREAM("Loaded extrinsic translation: [" << translation_xyz[0] << ", " << translation_xyz[1] << ", " << translation_xyz[2] << "]");
    
    nh.getParam("extrinsic_R", rotation_rpy);
    ROS_INFO_STREAM("Loaded extrinsic translation: [" << rotation_rpy[0] << ", " << rotation_rpy[1] << ", " << rotation_rpy[2] << "]");
    // default val
    // imu_topic = "/livox/imu_192_168_1_122";
    // new_topic = "imu";
    // new_frame_id = "imu_link";
    // unit_in_g = true;
    // translation_xyz = {0,0,0};
    // rotation_rpy = {0,90,0};
    trans << translation_xyz[0], translation_xyz[1], translation_xyz[2];
    rot << rotation_rpy[0], rotation_rpy[1], rotation_rpy[2];
    rot = rot * M_PI / 180;

}
void TransformData::transformToBaseLink(const sensor_msgs::Imu::ConstPtr &imu_msg){
    // get rot mat from rpy
    Eigen::Quaterniond q_rot = Eigen::AngleAxisd(rot[2], Eigen::Vector3d::UnitZ()) *
                                Eigen::AngleAxisd(rot[1], Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(rot[0], Eigen::Vector3d::UnitX());

    Eigen::Matrix3d rotation_matrix = q_rot.toRotationMatrix();

    Eigen::Vector3d angular_velocity, linear_acceleration;
    double gravity = -9.81;
    if (unit_in_g)
    {
        linear_acceleration = Eigen::Vector3d(imu_msg->linear_acceleration.x * gravity, imu_msg->linear_acceleration.y * gravity, imu_msg->linear_acceleration.z * gravity);
        angular_velocity = Eigen::Vector3d(imu_msg->angular_velocity.x * gravity, imu_msg->angular_velocity.y * gravity, imu_msg->angular_velocity.z * gravity);
    }
    else
    {
        linear_acceleration = Eigen::Vector3d(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
        angular_velocity = Eigen::Vector3d(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
    }
    // Apply rotation to linear acceleration
    linear_acceleration = rotation_matrix * linear_acceleration;
    // Apply rotation to angular velocity
    angular_velocity = rotation_matrix * angular_velocity;

    // Apply rotation to orientation
    Eigen::Quaterniond orientation;
    // tf2::fromMsg(imu_msg->orientation, orientation);
    orientation.x() = imu_msg->orientation.x;
    orientation.y() = imu_msg->orientation.y;
    orientation.z() = imu_msg->orientation.z;
    orientation.w() = imu_msg->orientation.w;
    orientation = orientation * Eigen::AngleAxisd(rot[2], Eigen::Vector3d::UnitZ()) *
                                Eigen::AngleAxisd(rot[1], Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(rot[0], Eigen::Vector3d::UnitX());
    orientation.normalize();

    // pub msg
    sensor_msgs::Imu transformed_imu_msg;
    transformed_imu_msg.header.frame_id = new_frame_id;
    transformed_imu_msg.header.seq = imu_msg->header.seq;
    transformed_imu_msg.header.stamp = ros::Time::now();
    transformed_imu_msg.linear_acceleration.x = linear_acceleration.x();
    transformed_imu_msg.linear_acceleration.y = linear_acceleration.y();
    transformed_imu_msg.linear_acceleration.z = linear_acceleration.z();
    transformed_imu_msg.angular_velocity.x = angular_velocity.x();
    transformed_imu_msg.angular_velocity.y = angular_velocity.y();
    transformed_imu_msg.angular_velocity.z = angular_velocity.z();
    // transformed_imu_msg.orientation = tf2::toMsg(orientation);
    transformed_imu_msg.orientation.x = orientation.x();
    transformed_imu_msg.orientation.y = orientation.y();
    transformed_imu_msg.orientation.z = orientation.z();
    transformed_imu_msg.orientation.w = orientation.w();
    // Publish transformed IMU data
    pub_imu.publish(transformed_imu_msg);
}

int main(int argc, char ** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "imu_transform_node");
    // 
    TransformData node;  
    ros::spin();
    return 0;
}