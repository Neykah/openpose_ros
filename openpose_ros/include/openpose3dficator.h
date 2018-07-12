#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "openpose_ros_msgs/OpenPoseHumanList.h"
#include "openpose_ros_msgs/OpenPoseHuman.h"
#include "openpose_ros_msgs/OpenPoseHuman3d.h"
#include "openpose_ros_msgs/PointWithProb.h"
#include "openpose_ros_msgs/PointWithProb3d.h"


namespace openpose_ros
{
void notDetectedBodyPart(int);

double Average(std::vector<double>);

void MakePoint3d(const openpose_ros_msgs::PointWithProb&, int,
                 const pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
                 const double minx, const double maxx, 
                 const double miny, const double maxy, 
                 const double minz, const double maxz,  
                 openpose_ros_msgs::PointWithProb3d&);

void MakeHuman3d(const openpose_ros_msgs::OpenPoseHuman&, 
                 const pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
                 const double minx, const double maxx, 
                 const double miny, const double maxy, 
                 const double minz, const double maxz,  
                 openpose_ros_msgs::OpenPoseHuman3d&);
class OpenPose3dficator
{
    public:
        OpenPose3dficator(const ros::Publisher&);
        void CallBack(const sensor_msgs::PointCloud2ConstPtr&, const openpose_ros_msgs::OpenPoseHumanListConstPtr&);

    private:
        ros::Publisher pub_;

};
} // namespace openpose_ros