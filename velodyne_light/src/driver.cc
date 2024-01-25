// Copyright (C) 2007, 2009-2012 Austin Robot Technology, Patrick Beeson, Jack O'Quin
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/** \file
 *
 *  ROS driver implementation for the Velodyne 3D LIDARs
 */

#include <string>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "velodyne_msgs/VelodyneScan.h"
#include "driver.h"

namespace velodyne_driver
{
  VelodyneDriver::VelodyneDriver(DriverParam config):config_(config)
  { 
    //validate string, determine packet rate
    double packet_rate;                   // packet frequency (Hz)
    std::string model_full_name;
    if (config_.model == "VLS128")
    { 
      //  3 firing cycles in a data packet. 3 x 53.3 μs = 0.1599 ms is the accumulation delay per packet. 
      //1 packet/0.1599 ms = 6253.9 packets/second
      packet_rate = 6253.9;     
      model_full_name = config_.model;
    }
    else if ((config_.model == "64E_S2") || (config_.model == "64E_S2.1"))    // generates 1333312 points per second
    {                                   // 1 packet holds 384 points
      packet_rate = 3472.17;            // 1333312 / 384
      model_full_name = std::string("HDL-") + config_.model;
    }
    else if (config_.model == "64E")
    {
      packet_rate = 2600.0;
      model_full_name = std::string("HDL-") + config_.model;
    }
    else if (config_.model == "64E_S3") // generates 2222220 points per second (half for strongest and half for lastest)
    {                                 // 1 packet holds 384 points
      packet_rate = 5787.03;          // 2222220 / 384
      model_full_name = std::string("HDL-") + config_.model;
    }
    else if (config_.model == "32E")
    {
      packet_rate = 1808.0;
      model_full_name = std::string("HDL-") + config_.model;
    }
      else if (config_.model == "32C")
    {
      packet_rate = 1507.0;
      model_full_name = std::string("VLP-") + config_.model;
    }
    else if (config_.model == "VLP16")
    {
      packet_rate = 754;             // 754 Packets/Second for Last or Strongest mode 1508 for dual (VLP-16 User Manual)
      model_full_name = "VLP-16";
    }
    else
    {
      ROS_ERROR_STREAM("unknown Velodyne LIDAR model: " << config_.model);
      packet_rate = 2600.0;
    }
    std::string deviceName(std::string("Velodyne ") + model_full_name);

    ROS_INFO_STREAM(deviceName << " rotating at " << config_.rpm << " RPM");
    double frequency = (config_.rpm / 60.0);     // expected Hz rate

    // default number of packets for each scan in a single revolution
    // (fractions rounded up)
    config_.npackets = (int) ceil(packet_rate / frequency);

    // if we are timestamping based on the first or last packet in the scan
    if (config_.timestamp_first_packet)
      ROS_INFO("Setting velodyne scan start time to timestamp of first packet");

    //Setting up the FOV
    if (config_.beforeZeroAng < 0 && config_.afterZeroAng < 0)
    {
      ROS_INFO_STREAM("FOV restriction deactivated.");
    }
    else if (config_.beforeZeroAng >= 180 && config_.beforeZeroAng <= 360 && config_.afterZeroAng <= 180 && config_.afterZeroAng >= 0)  //TODO: questo if in realtà non funziona!!
    {
        ROS_INFO_STREAM("FOV restriction activated from" << config_.beforeZeroAng << " and " << config_.afterZeroAng);
        // Convert cut_angle from radian to one-hundredth degree,
        // which is used in velodyne packets
        config_.beforeZeroAng = config_.beforeZeroAng * 100;
        config_.afterZeroAng = config_.afterZeroAng * 100;
    }
    else
    {
      ROS_ERROR_STREAM("FOV restriction deactivated. Wrong parameter settings.");
      config_.beforeZeroAng = -1;
      config_.afterZeroAng = -1;
    }

    //TODO: Da togliere?
    const double scan_freq = packet_rate/config_.npackets;
    ROS_INFO("expected frequency: %.3f (Hz)", scan_freq);

    config_.enabled = true;
    input_.reset(new velodyne_driver::InputSocket(config_.ipAddr, config_.gpsTime, config_.udp_port));
  }

  /** poll the device
   *
   *  @returns true unless end of file reached
   */
  bool VelodyneDriver::poll(velodyne_msgs::VelodyneScan *scanPkg)
  {
    if (!config_.enabled) {
      // If we are not enabled exit once a second to let the caller handle
      // anything it might need to, such as if it needs to exit.
      ros::Duration(1).sleep();
      return true;
    }

    // Allocate a new shared pointer for zero-copy sharing with other nodelets.
    //velodyne_msgs::VelodyneScanPtr scan(new velodyne_msgs::VelodyneScan);            //Va combiato con il pointer della funzione
    if (config_.beforeZeroAng >= 0 && config_.afterZeroAng >= 0) //FOV restiriction activated
    {

      scanPkg->packets.resize(config_.npackets);     //TODO: Controllare se va bene la freccia
      velodyne_msgs::VelodynePacket tmp_packet;

      for (int i = 0; i < config_.npackets; ++i)
      {
        while (true)
        {
          // keep reading until full packet received
          int rc = input_->getPacket(&tmp_packet, config_.time_offset);
          if (rc == 1) break;       // got a full packet?
          if (rc < 0) return false; // end of file reached?
          if (rc == 0) continue;    //timeout?
        }
        //Extract base rotation of first block in packet
        std::size_t azimuth_data_pos = 100*0+2;
        int azimuth = *( (u_int16_t*) (&tmp_packet.data[azimuth_data_pos]));

        if (azimuth <= config_.afterZeroAng || azimuth >= config_.beforeZeroAng){
          scanPkg->packets[i] = tmp_packet;
        }
      }
    }
    else // standard behaviour
    {
      // Since the velodyne delivers data at a very high rate, keep
      // reading and publishing scans as fast as possible.
      scanPkg->packets.resize(config_.npackets);
      for (int i = 0; i < config_.npackets; ++i)
      {
        while (true)
        {
          // keep reading until full packet received
          int rc = input_->getPacket(&scanPkg->packets[i], config_.time_offset);
          if (rc == 1) break;       // got a full packet?
          if (rc < 0) return false; // end of file reached?
          if (rc == 0) continue;    //timeout?
        }
      }
    }
    
    // publish message using time of last packet read

    if (config_.timestamp_first_packet){

      scanPkg->header.stamp = scanPkg->packets.front().stamp;
    }
    else{

      scanPkg->header.stamp = scanPkg->packets.back().stamp;
    }
    scanPkg->header.frame_id = config_.frame_id;

    return true;
  }
} // namespace velodyne_driver
