#include <iostream>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "openpose_ros_msgs/OpenPoseHumanList.h"
#include "openpose_ros_msgs/OpenPoseHumanList3d.h"
#include "openpose3dficator.h"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, openpose_ros_msgs::OpenPoseHumanList> NoSyncPolicy;

int main(int argc, char** argv)
{
    // Initialise ROS
    ros::init(argc, argv, "openpose3d");
    ros::NodeHandle nh;
    ROS_INFO("Wait to receive camera info.");

    ros::Publisher keypoints3d_publisher = nh.advertise<openpose_ros_msgs::OpenPoseHumanList3d>("openpose_ros/human_list3d", 1, true);
    openpose_ros::OpenPose3dficator to_3d(keypoints3d_publisher);

    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/kinect2/qhd/points", 1);
    message_filters::Subscriber<openpose_ros_msgs::OpenPoseHumanList> keypoints_sub(nh, "/openpose_ros/human_list", 1);
    message_filters::Synchronizer<NoSyncPolicy> sync(NoSyncPolicy(10), cloud_sub, keypoints_sub);
    sync.registerCallback(boost::bind(&openpose_ros::OpenPose3dficator::CallBack, &to_3d, _1, _2));
    ros::spin();
    return 0;
}
