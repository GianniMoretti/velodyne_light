/** \file
 *
 *  ROS driver node for the Velodyne 3D LIDARs.
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "velodyne_msgs/VelodyneScan.h"
#include "driver.h"
#include "rawdata.h"
#include "calibration.h"
#include "datacontainerbase.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velodyne_light_node");
    ros::NodeHandle node;
    const int NUM_THREADS = 8;
    // raw packet output topic
    //ros::Publisher output= node.advertise<velodyne_msgs::VelodyneScan>("velodyne_packets", 10);
    ros::Publisher output = node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);

    //Prendere i parametri dal nodo
    velodyne_driver::DriverParam config;
    node.param("/velodyne_light/device_ip", config.ipAddr, std::string("192.168.1.201"));
    node.param("/velodyne_light/frame_id", config.frame_id, std::string("velodyne"));
    node.param("/velodyne_light/model", config.model, std::string("VLP16"));
    node.param("/velodyne_light/port", config.udp_port, 2368);
    node.param("/velodyne_light/rpm", config.rpm, 600.0);
    node.param("/velodyne_light/gps_time", config.gpsTime, false);
    node.param("/velodyne_light/beforeZeroAng", config.beforeZeroAng, -1);
    node.param("/velodyne_light/afterZeroAng", config.afterZeroAng, -1);
    node.param("/velodyne_light/timestamp_first_packet", config.timestamp_first_packet, false);
    std::string targetFrame = "";
    std::string fixedFrame = "";
    config.time_offset = 0.0;   //volendo da aggiungere come parametro

    std::string calibFile = "";
    node.param("/velodyne_light/calibration", calibFile, std::string(""));
    ROS_INFO_STREAM("Calib file: " << calibFile);
    std::string tf_prefix = tf::getPrefixParam(node);
    ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
    config.frame_id = tf::resolve(tf_prefix, config.frame_id);

    // start the driver
    velodyne_driver::VelodyneDriver dvr(config);

    // Create a rawdata
    velodyne_rawdata::RawData* data = new velodyne_rawdata::RawData();

    //Calibration 
    velodyne_pointcloud::Calibration* calibration = data->setup(calibFile, config.model);

    int num_lasers = 16;
    if(calibration != nullptr)
    {
        ROS_DEBUG_STREAM("Calibration file loaded.");
        num_lasers = static_cast<int>(calibration->num_lasers);
    }
    else
    {
        ROS_ERROR_STREAM("Could not load calibration file!");
    }

    
    velodyne_rawdata::DataContainerBase* container_ptr = new velodyne_rawdata::DataContainerBase(targetFrame, fixedFrame, num_lasers, 0, false, data->scansPerPacket());
    container_ptr->configure(fixedFrame, targetFrame);

    // loop until shut down or end of file  
    while(ros::ok())
    {
        velodyne_msgs::VelodyneScan* scan_ptr = new velodyne_msgs::VelodyneScan();
        // poll device until end of file
        bool polled_ = dvr.poll(scan_ptr);
        if (!polled_){
            ROS_ERROR_THROTTLE(1.0, "Velodyne - Failed to poll device.");
            continue;
        }
        
        container_ptr->setup(scan_ptr);

        if(!container_ptr->computeTransformToTarget(scan_ptr->header.stamp))
        {
            // target frame not available
        }

        // process each packet provided by the driver
        //#pragma omp parallel for schedule(static)  num_threads(NUM_THREADS)
        for (size_t i = 0; i < scan_ptr->packets.size(); ++i)
        {
            // calculate individual transform for each packet to account for ego
            // during one rotation of the velodyne sensor
            if(!container_ptr->computeTransformToFixed(scan_ptr->packets[i].stamp))
            {
                // fixed frame not available
            }
            data->unpack(scan_ptr->packets[i], *container_ptr, i, scan_ptr->header.stamp);
        }
        // publish the accumulated cloud message
       // finishCloud = container_ptr->finishCloud();
        output.publish(container_ptr->finishCloud());
    }
    return 0;
}