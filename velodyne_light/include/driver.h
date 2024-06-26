// Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
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

#ifndef VELODYNE_DRIVER_DRIVER_H
#define VELODYNE_DRIVER_DRIVER_H

#include <string>
#include <ros/ros.h>

#include "velodyne_msgs/VelodyneScan.h"
#include "input.h"

namespace velodyne_driver
{
  struct DriverParam
  {
    std::string frame_id;            // tf frame ID
    std::string model;               // device model name
    std::string ipAddr;              // Address of velodyne lidar
    bool gpsTime;                    // Bool to enamble gps
    int npackets;                 // number of packets to collect
    double rpm;                      // device rotation rate (RPMs)
    double time_offset;              // time in seconds added to each velodyne time stamp
    bool enabled;                    // polling is enabled
    bool timestamp_first_packet;
    int udp_port;
    int afterZeroAng;
    int beforeZeroAng;
  };

  class VelodyneDriver
  {
    public:
      VelodyneDriver(DriverParam config);
      ~VelodyneDriver() {}
      bool poll(velodyne_msgs::VelodyneScan *scanPkg);

    private:
      DriverParam config_;
      boost::shared_ptr<Input> input_;   //Deve rimanere shared?
  };
}  // namespace velodyne_driver

#endif  // VELODYNE_DRIVER_DRIVER_H
