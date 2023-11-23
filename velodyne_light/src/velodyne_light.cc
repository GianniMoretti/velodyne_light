/** \file
 *
 *  ROS driver node for the Velodyne 3D LIDARs.
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <velodyne_msgs/VelodyneScan.h>
#include <velodyne_light/driver.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velodyne_light_node");
    ros::NodeHandle node;

    // raw packet output topic
    ros::Publisher output= node.advertise<velodyne_msgs::VelodyneScan>("velodyne_packets", 10);

    //Prendere i parametri dal nodo
    velodyne_driver::DriverParam config;
    node.param("device_ip", config.ipAddr, std::string("192.168.1.201"));
    node.param("frame_id", config.frame_id, std::string("velodyne"));
    node.param("model", config.model, std::string("VLP16"));
    node.param("pcap", config.dump_file, std::string(""));
    node.param("port", config.udp_port, 2368);
    node.param("rpm", config.rpm, 600.0);
    node.param("gps_time", config.gpsTime, false);
    node.param("beforeZeroAng", config.beforeZeroAng, -1);
    node.param("afterZeroAng", config.afterZeroAng, -1);
    node.param("timestamp_first_packet", config.timestamp_first_packet, false);
    config.time_offset = 0.0;   //volendo da aggiungere come parametro

    std::string tf_prefix = tf::getPrefixParam(node);
    ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
    config.frame_id = tf::resolve(tf_prefix, config.frame_id);

    // start the driver
    velodyne_driver::VelodyneDriver dvr(config);

    // loop until shut down or end of file  
    while(ros::ok())
    {
        velodyne_msgs::VelodyneScan scan;
        // poll device until end of file
        bool polled_ = dvr.poll(&scan);
        if (!polled_)
            ROS_ERROR_THROTTLE(1.0, "Velodyne - Failed to poll device.");
            continue;

        //Qui si aggiunge la trasformazione

        //Pubblicare il messaggio
        output.publish(scan);
    }

    return 0;
}