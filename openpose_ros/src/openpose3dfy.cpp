#include <iostream>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "openpose_ros_msgs/OpenPoseHumanList.h"
#include "openpose_ros_msgs/OpenPoseHumanList3d.h"
#include "openpose3dficator.h"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, openpose_ros_msgs::OpenPoseHumanList> NoSyncPolicy;

int main(int argc, char** argv)
{
    // Initialise ROS
    ros::init(argc, argv, "openpose3d");
    ros::NodeHandle nh;
    ROS_INFO("Wait to receive camera info.");

    // Fill K from camera info topic into a new vector
    sensor_msgs::CameraInfoConstPtr camera_info(new sensor_msgs::CameraInfo);
    camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/kinect2/qhd/camera_info");
    ROS_INFO("Camera info topic received.");
    std::vector<float> camera_params(4);
    camera_params[0] = camera_info->K[0];
    camera_params[1] = camera_info->K[2];
    camera_params[2] = camera_info->K[4];
    camera_params[3] = camera_info->K[5];

    ros::Publisher keypoints3d_publisher = nh.advertise<openpose_ros_msgs::OpenPoseHumanList3d>("openpose_ros/human_list3d", 1, true);
    openpose_ros::OpenPose3dficator to_3d(keypoints3d_publisher, camera_params);

    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/kinect2/qhd/image_depth_rect", 1);
    message_filters::Subscriber<openpose_ros_msgs::OpenPoseHumanList> keypoints_sub(nh, "/openpose_ros/human_list", 1);
    message_filters::Synchronizer<NoSyncPolicy> sync(NoSyncPolicy(10), image_sub, keypoints_sub);
    sync.registerCallback(boost::bind(&openpose_ros::OpenPose3dficator::CallBack, &to_3d, _1, _2));
    ros::spin();
    return 0;
}
