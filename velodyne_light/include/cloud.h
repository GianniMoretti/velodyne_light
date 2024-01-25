#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <algorithm>
#include <cstdarg>

class Cloud{

    public:
        Cloud(int init_width)
        {
            //int fields = 6;
            cloud.fields.clear();
            cloud.fields.reserve(6);
            int offset = 0;
            offset = addPointField(cloud, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
            offset = addPointField(cloud, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
            offset = addPointField(cloud, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
            offset = addPointField(cloud, "intensity", 1, sensor_msgs::PointField::FLOAT32, offset);
            offset = addPointField(cloud, "ring", 1, sensor_msgs::PointField::UINT16, offset);
            offset = addPointField(cloud, "time", 1, sensor_msgs::PointField::FLOAT32, offset);
            cloud.point_step = offset;
            cloud.row_step = init_width * cloud.point_step;
        }

        sensor_msgs::PointCloud2 cloud;

};