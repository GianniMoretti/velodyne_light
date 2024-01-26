#ifndef VELODYNE_POINTCLOUD_DATACONTAINERBASE_H
#define VELODYNE_POINTCLOUD_DATACONTAINERBASE_H
// Copyright (C) 2012, 2019 Austin Robot Technology, Jack O'Quin, Joshua Whitley, Sebastian Pütz
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

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <algorithm>
#include <cstdarg>

#include "cloud.h"

namespace velodyne_rawdata
{
  struct Config
  {
    std::string target_frame;  ///< output frame of final point cloud
    std::string fixed_frame;   ///< world fixed frame for ego motion compenstation
    unsigned int init_width;
    unsigned int init_height;
    bool is_dense;
    unsigned int scans_per_packet;

    Config(std::string target_frame, std::string fixed_frame,
          unsigned int init_width, unsigned int init_height, bool is_dense, unsigned int scans_per_packet)
      : target_frame(target_frame)
      , fixed_frame(fixed_frame)
      , init_width(init_width)
      , init_height(init_height)
      , is_dense(is_dense)
      , scans_per_packet(scans_per_packet)
    {
      ROS_INFO_STREAM("Initialized container with "
                      << "target_frame: " << target_frame << ", fixed_frame: " << fixed_frame
                      << ", init_width: " << init_width << ", init_height: " << init_height
                      << ", is_dense: " << is_dense << ", scans_per_packet: " << scans_per_packet);
    }
  };

  class DataContainerBase
  {
    public:
    //Il problema è qui
      DataContainerBase(const std::string& target_frame, const std::string& fixed_frame, const unsigned int init_width, const unsigned int init_height, const bool is_dense, const unsigned int scans_per_packet, ...)
      : config_(target_frame, fixed_frame, init_width, init_height, is_dense, scans_per_packet), cloud_obj(init_width), 
      iter_x(cloud_obj.cloud, "x"), iter_y(cloud_obj.cloud, "y"), iter_z(cloud_obj.cloud, "z"),
      iter_ring(cloud_obj.cloud, "ring"), iter_intensity(cloud_obj.cloud, "intensity"), iter_time(cloud_obj.cloud, "time")
      {
        cloud = cloud_obj.cloud;
      }

      void setup(const velodyne_msgs::VelodyneScan* scan_msg)
      {
        sensor_frame = scan_msg->header.frame_id;
        manage_tf_buffer();
        cloud.header.stamp = scan_msg->header.stamp;
        cloud.data.resize(scan_msg->packets.size() * config_.scans_per_packet * cloud.point_step);
        cloud.width = config_.init_width;
        cloud.height = config_.init_height;
        cloud.is_dense = static_cast<uint8_t>(config_.is_dense);
        iter_x = sensor_msgs::PointCloud2Iterator<float>(cloud, "x");
        iter_y = sensor_msgs::PointCloud2Iterator<float>(cloud, "y");
        iter_z = sensor_msgs::PointCloud2Iterator<float>(cloud, "z");
        iter_intensity = sensor_msgs::PointCloud2Iterator<float>(cloud, "intensity");
        iter_ring = sensor_msgs::PointCloud2Iterator<uint16_t >(cloud, "ring");
        iter_time = sensor_msgs::PointCloud2Iterator<float >(cloud, "time");
      }

      void addPoint(int nfirings, float x, float y, float z, const uint16_t ring, const uint16_t /*azimuth*/, const float distance, const float intensity, const float time)
      {
        //transformPoint(x, y, z);
        int offset = config_.init_width * nfirings;
        *(iter_x + offset + ring) = x;
        *(iter_y + offset + ring) = y;
        *(iter_z + offset + ring) = z;
        *(iter_intensity + offset + ring) = intensity;
        *(iter_ring + offset + ring) = ring;
        *(iter_time + offset + ring) = time;
        if (ring == 15){
          cloud.height++;
        }
      }

      const sensor_msgs::PointCloud2& finishCloud()
      {
        cloud.data.resize(cloud.point_step * cloud.width * cloud.height);
        cloud.row_step = cloud.point_step * cloud.width;

        if (!config_.target_frame.empty()){
          cloud.header.frame_id = config_.target_frame;
        }
        else if (!config_.fixed_frame.empty()){
          cloud.header.frame_id = config_.fixed_frame;
        }
        else{
          cloud.header.frame_id = sensor_frame;
        }

        //ROS_INFO_STREAM("Prepared cloud width " << cloud.height * cloud.width << " Velodyne points, time: " << cloud.header.stamp);
        return cloud;
      }

      void manage_tf_buffer()
      {
        // check if sensor frame is already known, if not don't prepare tf buffer until setup was called
        if (sensor_frame.empty()){
          return;
        }

        // avoid doing transformation when sensor_frame equals target frame and no ego motion compensation is perfomed
        if (config_.fixed_frame.empty() && sensor_frame == config_.target_frame){
          // when the string is empty the points will not be transformed later on
          config_.target_frame = "";
          return;
        }

        // only use somewhat resource intensive tf listener when transformations are necessary
        if (!config_.fixed_frame.empty() || !config_.target_frame.empty()){
          if (!tf_buffer){
            tf_buffer = std::make_shared<tf2_ros::Buffer>();
            tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
          }
        }
        else{
          tf_listener.reset();
          tf_buffer.reset();
        }
      }

      void configure(const std::string fixed_frame, const std::string target_frame){
        config_.fixed_frame = fixed_frame;
        config_.target_frame = target_frame;
        manage_tf_buffer();
      }

      sensor_msgs::PointCloud2 cloud;
      Cloud cloud_obj;

      inline bool calculateTransformMatrix(Eigen::Affine3f& matrix, const std::string& target_frame, const std::string& source_frame, const ros::Time& time)
      {
        if (!tf_buffer){
          ROS_ERROR("tf buffer was not initialized yet");
          return false;
        }

        geometry_msgs::TransformStamped msg;
        try{
          msg = tf_buffer->lookupTransform(target_frame, source_frame, time, ros::Duration(0.2));
        }
        catch (tf2::LookupException& e){
          ROS_ERROR("%s", e.what());
          return false;
        }
        catch (tf2::ExtrapolationException& e){
          ROS_ERROR("%s", e.what());
          return false;
        }

        const geometry_msgs::Quaternion& quaternion = msg.transform.rotation;
        Eigen::Quaternionf rotation(quaternion.w, quaternion.x, quaternion.y, quaternion.z);

        const geometry_msgs::Vector3& origin = msg.transform.translation;
        Eigen::Translation3f translation(origin.x, origin.y, origin.z);

        matrix = translation * rotation;
        return true;
      }

      inline bool computeTransformToTarget(const ros::Time &scan_time)
      {
        if (config_.target_frame.empty())
        {
          // no need to calculate transform -> success
          return true;
        }
        std::string& source_frame = config_.fixed_frame.empty() ? sensor_frame : config_.fixed_frame;
        return calculateTransformMatrix(tf_matrix_to_target, config_.target_frame, source_frame, scan_time);
      }

      inline bool computeTransformToFixed(const ros::Time &packet_time)
      {
        if (config_.fixed_frame.empty())
        {
          // no need to calculate transform -> success
          return true;
        }
        std::string &source_frame = sensor_frame;
        return calculateTransformMatrix(tf_matrix_to_fixed, config_.fixed_frame, source_frame, packet_time);
      }

      inline void transformPoint(float& x, float& y, float& z)
      {
        Eigen::Vector3f p = Eigen::Vector3f(x, y, z);
        if (!config_.fixed_frame.empty())
        {
          p = tf_matrix_to_fixed * p;
        }
        if (!config_.target_frame.empty())
        {
          p = tf_matrix_to_target * p;
        }
        x = p.x();
        y = p.y();
        z = p.z();
      }

    private:
      sensor_msgs::PointCloud2Iterator<float> iter_x, iter_y, iter_z, iter_intensity, iter_time;
      sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring;


    protected:
      Config config_;
      std::shared_ptr<tf2_ros::TransformListener> tf_listener;
      std::shared_ptr<tf2_ros::Buffer> tf_buffer;
      Eigen::Affine3f tf_matrix_to_fixed;
      Eigen::Affine3f tf_matrix_to_target;
      std::string sensor_frame;
  };

} /* namespace velodyne_rawdata */
#endif  // VELODYNE_POINTCLOUD_DATACONTAINERBASE_H
