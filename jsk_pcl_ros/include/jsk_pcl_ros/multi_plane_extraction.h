// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda and JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#ifndef JSK_PCL_ROS_MULTI_PLANE_EXTRACTION_H_
#define JSK_PCL_ROS_MULTI_PLANE_EXTRACTION_H_

#include <ros/ros.h>
#include <ros/names.h>
#include <pcl_ros/pcl_nodelet.h>

#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>

#include "jsk_pcl_ros/ClusterPointIndices.h"
#include "sensor_msgs/PointCloud2.h"
#include "jsk_pcl_ros/ModelCoefficientsArray.h"
#include "jsk_pcl_ros/PolygonArray.h"

namespace jsk_pcl_ros
{
  class MultiPlaneExtraction: public pcl_ros::PCLNodelet
  {
  public:
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, jsk_pcl_ros::ClusterPointIndices, jsk_pcl_ros::ModelCoefficientsArray, jsk_pcl_ros::PolygonArray> SyncPolicy;
  protected:
    int maximum_queue_size_;
    ros::Publisher pub_;
    
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<jsk_pcl_ros::ModelCoefficientsArray> sub_coefficients_;
    message_filters::Subscriber<jsk_pcl_ros::PolygonArray> sub_polygons_;
    message_filters::Subscriber<jsk_pcl_ros::ClusterPointIndices> sub_indices_;
    
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    
    virtual void extract(const sensor_msgs::PointCloud2::ConstPtr& input,
                         const jsk_pcl_ros::ClusterPointIndices::ConstPtr& indices,
                         const jsk_pcl_ros::ModelCoefficientsArray::ConstPtr& coefficients,
                         const jsk_pcl_ros::PolygonArray::ConstPtr& polygons);
    
  private:
    virtual void onInit();
    
  };
}

#endif
