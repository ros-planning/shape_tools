/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *     disclaimer in the documentation and/or other materials provided
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

#include "shape_conversions/shape_to_marker.h"
#include <ros/console.h>

bool shape_conversions::constructMarkerFromShape(const shape_msgs::Shape &shape_msg, visualization_msgs::Marker &mk, bool use_mesh_triangle_list)
{
  switch (shape_msg.type)
  {
  case shape_msgs::Shape::SPHERE:
    mk.type = visualization_msgs::Marker::SPHERE;
    if (shape_msg.dimensions.size() != 1)
    {
      ROS_ERROR("Unexpected number of dimensions in sphere definition");
      return false;
    }
    else
      mk.scale.x = mk.scale.y = mk.scale.z = shape_msg.dimensions[0] * 2.0;
    break;
  case shape_msgs::Shape::BOX:
    mk.type = visualization_msgs::Marker::CUBE;
    if (shape_msg.dimensions.size() != 3)
    {
      ROS_ERROR("Unexpected number of dimensions in box definition");
      return false;
    }
    else
    {
      mk.scale.x = shape_msg.dimensions[0];
      mk.scale.y = shape_msg.dimensions[1];
      mk.scale.z = shape_msg.dimensions[2];
    }
    break;
  case shape_msgs::Shape::CYLINDER:
    mk.type = visualization_msgs::Marker::CYLINDER;
    if (shape_msg.dimensions.size() != 2)
    {
      ROS_ERROR("Unexpected number of dimensions in cylinder definition");
      return false;
    }
    else
    {
      mk.scale.x = shape_msg.dimensions[0] * 2.0;
      mk.scale.y = shape_msg.dimensions[0] * 2.0;
      mk.scale.z = shape_msg.dimensions[1];
    }
    break;
    
  case shape_msgs::Shape::MESH:
    if (shape_msg.dimensions.size() != 0) {
      ROS_ERROR("Unexpected number of dimensions in mesh definition");
      return false;
    } 
    if (shape_msg.triangles.size() % 3 != 0) {
      ROS_ERROR("Number of triangle indices is not divisible by 3");
      return false;
    }
    if (shape_msg.triangles.empty() || shape_msg.vertices.empty()) {
      ROS_ERROR("Mesh definition is empty");
      return false;
    }
    if(!use_mesh_triangle_list) {
      mk.type = visualization_msgs::Marker::LINE_LIST;
      mk.scale.x = mk.scale.y = mk.scale.z = 0.01;
      std::size_t nt = shape_msg.triangles.size() / 3;
      for (std::size_t i = 0 ; i < nt ; ++i)
      {
        mk.points.push_back(shape_msg.vertices[shape_msg.triangles[3*i]]);
        mk.points.push_back(shape_msg.vertices[shape_msg.triangles[3*i+1]]);
        mk.points.push_back(shape_msg.vertices[shape_msg.triangles[3*i]]);
        mk.points.push_back(shape_msg.vertices[shape_msg.triangles[3*i+2]]);
        mk.points.push_back(shape_msg.vertices[shape_msg.triangles[3*i+1]]);
        mk.points.push_back(shape_msg.vertices[shape_msg.triangles[3*i+2]]);
      }
    } else {
      mk.type = visualization_msgs::Marker::TRIANGLE_LIST;
      mk.scale.x = mk.scale.y = mk.scale.z = 1.0;
      for (std::size_t i = 0 ; i < shape_msg.triangles.size(); i+=3)
      {
        mk.points.push_back(shape_msg.vertices[shape_msg.triangles[i]]);
        mk.points.push_back(shape_msg.vertices[shape_msg.triangles[i+1]]);
        mk.points.push_back(shape_msg.vertices[shape_msg.triangles[i+2]]);
      }
    }
    return true;
  default:
    ROS_ERROR("Unknown shape type: %d", (int)shape_msg.type);
    return false;
  }
  return true;
}
