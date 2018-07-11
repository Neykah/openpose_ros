#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"

#include "openpose_ros_msgs/OpenPoseHumanList.h"
#include "openpose_ros_msgs/OpenPoseHuman.h"
#include "openpose_ros_msgs/OpenPoseHuman3d.h"
#include "openpose_ros_msgs/PointWithProb.h"
#include "openpose_ros_msgs/PointWithProb3d.h"



namespace openpose_ros
{    


class OpenPose3dficator
{
    public:
        OpenPose3dficator(const ros::Publisher&, const std::vector<float>&);
        void MakePoint3d(const openpose_ros_msgs::PointWithProb&, 
                         const sensor_msgs::ImageConstPtr& depth_image, 
                         openpose_ros_msgs::PointWithProb3d&);

        void MakeHuman3d(const openpose_ros_msgs::OpenPoseHuman&, 
                         const sensor_msgs::ImageConstPtr& depth_image, 
                         openpose_ros_msgs::OpenPoseHuman3d&);

        void CallBack(const sensor_msgs::ImageConstPtr&, const openpose_ros_msgs::OpenPoseHumanListConstPtr&);

    private:
        ros::Publisher pub_;
        std::vector<float> camera_params_;

};
} // namespace openpose_ros