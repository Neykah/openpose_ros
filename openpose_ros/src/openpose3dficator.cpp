#include "openpose3dficator.h"
#include <iostream>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"

#include "openpose_ros_msgs/PointWithProb.h"
#include "openpose_ros_msgs/PointWithProb3d.h"
#include "openpose_ros_msgs/OpenPoseHuman.h"
#include "openpose_ros_msgs/OpenPoseHuman3d.h"
#include "openpose_ros_msgs/OpenPoseHumanList.h"
#include "openpose_ros_msgs/OpenPoseHumanList3d.h"


namespace openpose_ros
{
OpenPose3dficator::OpenPose3dficator(const ros::Publisher& pub, const std::vector<float>& camera_params)
    : pub_(pub), camera_params_(camera_params)
{}

void OpenPose3dficator::MakePoint3d(const openpose_ros_msgs::PointWithProb& point2d, openpose_ros_msgs::PointWithProb3d point3d)
{
    float fx, cx, fy, cy;
    fx = camera_params_[0];
    cx = camera_params_[1];
    fy = camera_params_[2];
    cy = camera_params_[3];
}

void OpenPose3dficator::MakeHuman3d(const openpose_ros_msgs::OpenPoseHuman& human2d, openpose_ros_msgs::OpenPoseHuman3d& human3d)
{
    human3d.num_body_key_points_with_non_zero_prob = human2d.num_body_key_points_with_non_zero_prob;

    for (size_t i = 0 ; i < human2d.body_key_points_with_prob.size() ; i++)
    {
        openpose_ros_msgs::PointWithProb3d point3d;
        MakePoint3d(human2d.body_key_points_with_prob[i], point3d);
        human3d.body_key_points_with_prob[i] = point3d;
    }
}

void OpenPose3dficator::CallBack(const sensor_msgs::ImageConstPtr& depth_image, const openpose_ros_msgs::OpenPoseHumanListConstPtr& input_human_list)
{
    ROS_INFO("Beginning of the callback function.");
    // Create the output message and fill the headers
    openpose_ros_msgs::OpenPoseHumanList3d output_human_list;
    output_human_list.header = input_human_list->header;
    output_human_list.image_header = input_human_list->image_header;
    output_human_list.num_humans = input_human_list->num_humans;

    for (size_t i = 0 ; i < input_human_list->human_list.size() ; i++)
    {
        openpose_ros_msgs::OpenPoseHuman3d human3d;
        MakeHuman3d(input_human_list->human_list[i], human3d);
        output_human_list.human_list.push_back(human3d);
    }
    pub_.publish(output_human_list);
}
} // namespace openpose_ros