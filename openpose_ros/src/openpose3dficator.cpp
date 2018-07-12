#include "openpose3dficator.h"
#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl/common/common.h"
#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

#include "openpose_ros_msgs/PointWithProb.h"
#include "openpose_ros_msgs/PointWithProb3d.h"
#include "openpose_ros_msgs/OpenPoseHuman.h"
#include "openpose_ros_msgs/OpenPoseHuman3d.h"
#include "openpose_ros_msgs/OpenPoseHumanList.h"
#include "openpose_ros_msgs/OpenPoseHumanList3d.h"

// Set resolution (qhd)
#define width 960
#define height 540

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC; 

namespace openpose_ros
{

// Function for when a bodypart is not detected
void notDetectedBodyPart(int bodypart_index)
{
    std::cerr << "Body part " << bodypart_index << " not detected!" << std::endl;
    std::cerr << "Body part " << bodypart_index << " pixel coordinates (x,y,z): " << std::endl;
    std::cerr << "(nan, nan, nan)" << std::endl;

    std::cerr << "Body part " << bodypart_index << " real world coordinates (x,y,z): " << std::endl;
    std::cerr << "(nan, nan, nan)" << std::endl;
}

double Average(std::vector<double> v)
{
    double total(0.0);
    double size(0.0);
    for (int n = 0 ; n < v.size() ; n++)
        total += v[n];
    
    return total / v.size();
}

void MakePoint3d(const openpose_ros_msgs::PointWithProb& point2d,
                 int bodypart_index,
                 const PointCloudC::Ptr cloud,
                 const double minx, const double maxx, 
                 const double miny, const double maxy, 
                 const double minz, const double maxz,  
                 openpose_ros_msgs::PointWithProb3d& point3d)
{
    // Include 2d confidence in 3d message
    point3d.prob = point2d.prob;

    // If not detected bodypart
    if (std::isnan(point3d.prob) || point3d.prob == 0.0 || (point2d.x == 0 && point2d.y == 0) || 
        point2d.x > width || point2d.y > height)
    {
        notDetectedBodyPart(bodypart_index);
        point3d.x = NAN;
        point3d.y = NAN;
        point3d.z = NAN;
    }

    // If detected bodypart
    else
    {
        // Get keypoint pixel coordinates
        unsigned int x_pixel = point2d.x;
        unsigned int y_pixel = point2d.y;

        // Vector for storing the keypoint index and the surrounding indices ones
        std::vector<unsigned int> indices;
        int index = 0;

        // Number of rows and columns of indices surrounding keypoint to get (both must be even)
        int rows = 3;
        int columns = 3;

        // Store in the vector the indices surrounding the keypoint
        for (int i = -(rows - 1) / 2 ; i <= (rows - 1) / 2 ; i++)
        {
            for (int j = -(columns - 1) / 2 ; j <= (columns - 1) / 2 ; j++)
            {
                index = width * (y_pixel + i) + x_pixel + j + 1;
                indices.push_back(index);
            }
        }

        // Vector for storing possible world coordinates of indices in the cluster
        std::vector<double> possible_x;
        std::vector<double> possible_y;
        std::vector<double> possible_z;

        // Get coordinates if they are valid
        for (size_t i = 0 ; i < indices.size() ; i++)
        {
            if (not std::isnan(cloud->points[indices[i]].x) && 
                not std::isnan(cloud->points[indices[i]].y) && 
                not std::isnan(cloud->points[indices[i]].z))
            {
                if (cloud->points[indices[i]].x >= minx && cloud->points[indices[i]].x <= maxx)
                {
                    if (cloud->points[indices[i]].y >= miny && cloud->points[indices[i]].y <= maxy)
                    {
                        if (cloud->points[indices[i]].z >= minz && cloud->points[indices[i]].z <= maxz)
                        {
                            possible_x.push_back(cloud->points[indices[i]].x);
                            possible_y.push_back(cloud->points[indices[i]].y);
                            possible_z.push_back(cloud->points[indices[i]].z);
                        }
                    }
                }
            }
        }

        // Check if vectors are empty
        if(possible_x.size() == 0 || possible_y.size() == 0 || possible_z.size() == 0)
        {
            notDetectedBodyPart(bodypart_index);
            point3d.x = NAN;
            point3d.y = NAN;
            point3d.z = NAN;
        }
        else
        {
            // Compute the mean for each coordinate
            point3d.x = Average(possible_x);
            point3d.y = Average(possible_y);
            point3d.z = Average(possible_z);

            // Print out the result coordinates
            std::cout << "Body part " << bodypart_index << " pixel coordinates (x, y, z): " << std::endl;
            std::cout << "( " << point2d.x << ", " << point2d.y << ", " << point3d.z << ")" << std::endl;

            std::cout << "Body part " << bodypart_index << " real world coordinates (x, y, z): " << std::endl;
            std::cout << "( " << point3d.x << ", " << point3d.y << ", " << point3d.z << ")" << std::endl; 
        }
    }
}

void MakeHuman3d(const openpose_ros_msgs::OpenPoseHuman& human2d, 
                 const PointCloudC::Ptr cloud,
                 const double minx, const double maxx, 
                 const double miny, const double maxy, 
                 const double minz, const double maxz,     
                 openpose_ros_msgs::OpenPoseHuman3d& human3d)
{
    human3d.num_body_key_points_with_non_zero_prob = human2d.num_body_key_points_with_non_zero_prob;

    for (size_t i = 0 ; i < human2d.body_key_points_with_prob.size() ; i++)
    {
        openpose_ros_msgs::PointWithProb3d point3d;
        MakePoint3d(human2d.body_key_points_with_prob[i], i, cloud, minx, maxx, miny, maxy, minz, maxz, point3d);
        human3d.body_key_points_with_prob[i] = point3d;
    }
}


OpenPose3dficator::OpenPose3dficator(const ros::Publisher& pub)
    : pub_(pub)
{}

void OpenPose3dficator::CallBack(const sensor_msgs::PointCloud2ConstPtr& cloud, const openpose_ros_msgs::OpenPoseHumanListConstPtr& input_human_list)
{
    ROS_INFO("Beginning of the callback function.");
    PointCloudC::Ptr pcl_cloud(new PointCloudC);
    pcl::fromROSMsg(*cloud, *pcl_cloud);
    PointC min;
    PointC max;

    // Get min and max coordinates
    pcl::getMinMax3D<PointC>(*pcl_cloud, min, max);

    // Print minimum x, y, z 
    std::cout << "Minimum pointcloud (x, y, z) coordinates: " << std::endl;
    std::cout << "  (" << min.x << ", " << min.y << ", " << min.z << ")" << std::endl;

    // Print maximum x, y ,z
    std::cout << "Maximum pointcloud (x, y, z) coordinates: " << std::endl;
    std::cout << "  (" << max.x << ", " << max.y << ", " << max.z << ")" << std::endl;


    // Create the output message and fill the headers
    openpose_ros_msgs::OpenPoseHumanList3d output_human_list;
    output_human_list.header = input_human_list->header;
    output_human_list.image_header = input_human_list->image_header;
    output_human_list.num_humans = input_human_list->num_humans;

    for (size_t i = 0 ; i < input_human_list->human_list.size() ; i++)
    {
        openpose_ros_msgs::OpenPoseHuman3d human3d;
        MakeHuman3d(input_human_list->human_list[i], pcl_cloud, min.x, max.x, min.y, max.y, min.z, max.z, human3d);
        output_human_list.human_list.push_back(human3d);
    }
    pub_.publish(output_human_list);
}
} // namespace openpose_ros