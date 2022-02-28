/*
 * rrtstar_visualization.cpp
 *
 * ---------------------------------------------------------------------
 * Copyright (C) 2022 Matthew (matthewoots at gmail.com)
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * ---------------------------------------------------------------------
 *
 * 
 * 
 */

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

#include <string>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>

#include <geometry_msgs/Point.h>

#include "rrtstar_ros/point_array.h"
#include "rrtstar_ros/start_end_markers.h"
#include "rrtstar_ros/bounding_box.h"
#include "rrtstar_ros/bounding_box_array.h"

using namespace std;
using namespace Eigen;

ros::Publisher rrt_marker_pub, start_end_marker_pub, bs_marker_pub;
ros::Publisher ellipsoid_marker_pub, constrains_marker_pub;
ros::Subscriber bs_message, rrt_message, start_end_message, bounding_box_message;

double _obs_threshold, _scale_z;

void rrt_callback(const rrtstar_ros::point_array::ConstPtr &msg)
{
  rrtstar_ros::point_array rrt = *msg;

  visualization_msgs::Marker rrt_points, line_strip, ellipsoid;
  rrt_points.header.frame_id = line_strip.header.frame_id = 
    ellipsoid.header.frame_id = "/map";
  rrt_points.header.stamp = line_strip.header.stamp = 
    ellipsoid.header.stamp = ros::Time::now();
  rrt_points.ns = line_strip.ns = ellipsoid.ns = "rrt_visualization_points";
  rrt_points.action = line_strip.action = ellipsoid.action = visualization_msgs::Marker::ADD;
  rrt_points.pose.orientation.w = line_strip.pose.orientation.w =
    ellipsoid.pose.orientation.w = 1.0;

  rrt_points.id = 0;
  line_strip.id = 1;

  rrt_points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  ellipsoid.type = visualization_msgs::Marker::SPHERE;

  // POINTS markers use x and y scale for width/height respectively
  rrt_points.scale.x = 0.2;
  rrt_points.scale.y = 0.2;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.1;

  // Points color
  rrt_points.color.g = 1.0f;
  rrt_points.color.b = 1.0f;
  rrt_points.color.a = 1.0f;

  // Line strip color
  line_strip.color.r = 1.0f;
  line_strip.color.b = 1.0f;
  line_strip.color.a = 1.0f;

  int rrt_size = rrt.array.size();
  // Create the vertices for the points and lines
  for (int i = 0; i < rrt_size; i++)
  {
    geometry_msgs::Point p;
    p.x = rrt.array[i].x;
    p.y = rrt.array[i].y;
    p.z = rrt.array[i].z;

    rrt_points.points.push_back(p);
    line_strip.points.push_back(p);

    ellipsoid.id = i;
    ellipsoid.scale.x = _obs_threshold;
    ellipsoid.scale.y = _obs_threshold;
    ellipsoid.scale.z = (1 /_scale_z) * _obs_threshold;
    // Ellipsoid color
    ellipsoid.color.b = 1.0f;
    ellipsoid.color.a = 0.5f;
    ellipsoid.pose.position = p;

    ellipsoid_marker_pub.publish(ellipsoid);
  }

  rrt_marker_pub.publish(rrt_points);
  rrt_marker_pub.publish(line_strip);
}

void constrain_callback(const rrtstar_ros::bounding_box_array::ConstPtr &msg)
{
  rrtstar_ros::bounding_box_array bb_array = *msg;

  visualization_msgs::Marker bb_points;
  bb_points.header.frame_id = "/map";
  bb_points.header.stamp = ros::Time::now();
  bb_points.ns = "constrain_visualization_points";
  bb_points.action = visualization_msgs::Marker::ADD;

  bb_points.type = visualization_msgs::Marker::CUBE;

  int bb_size = bb_array.array.size();
  // Create the vertices for the points and lines
  for (int i = 0; i < bb_size; i++)
  {
    bb_points.id = i;

    bb_points.color.r = 1.0f;
    bb_points.color.g = 0.0f;
    bb_points.color.b = 0.0f;
    bb_points.color.a = 0.3f;

    Vector3d box_difference;
    box_difference.x() = bb_array.array[i].max.x - bb_array.array[i].min.x;
    box_difference.y() = bb_array.array[i].max.y - bb_array.array[i].min.y;
    box_difference.z() = bb_array.array[i].max.z - bb_array.array[i].min.z;

    Vector3d box_center = box_difference/2;

    printf("%s[rrtstar_visualization.cpp] [%lf %lf %lf] Box Seperation [%lf %lf %lf] \n", 
      KYEL, bb_array.array[i].center.x, bb_array.array[i].center.y, 
      bb_array.array[i].center.z, box_difference.x(), box_difference.y(), box_difference.z());

    bb_points.pose.position.x = bb_array.array[i].center.x;
    bb_points.pose.position.y = bb_array.array[i].center.y;
    bb_points.pose.position.z = bb_array.array[i].center.z;
    
    bb_points.pose.orientation = bb_array.array[i].orientation;

    // CUBE markers use x and y scale for width/height respectively
    bb_points.scale.x = abs(box_difference.x());
    bb_points.scale.y = abs(box_difference.y());
    bb_points.scale.z = abs(box_difference.z());

    constrains_marker_pub.publish(bb_points);
  }

}

void bs_callback(const rrtstar_ros::point_array::ConstPtr &msg)
{
  rrtstar_ros::point_array rrt = *msg;

  visualization_msgs::Marker cp_points, line_strip;
  cp_points.header.frame_id = line_strip.header.frame_id = "/map";
  cp_points.header.stamp = line_strip.header.stamp = ros::Time::now();
  cp_points.ns = line_strip.ns = "bspline_visualization_points";
  cp_points.action = line_strip.action = visualization_msgs::Marker::ADD;
  cp_points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

  cp_points.id = 0;
  line_strip.id = 1;

  cp_points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;


  // POINTS markers use x and y scale for width/height respectively
  cp_points.scale.x = 0.2;
  cp_points.scale.y = 0.2;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.1;

  // Points color
  cp_points.color.r = 1.0f;
  cp_points.color.g = 1.0f;
  cp_points.color.a = 1.0f;

  // Line strip color
  line_strip.color.b = 1.0f;
  line_strip.color.g = 1.0f;
  line_strip.color.a = 1.0f;

  int rrt_size = rrt.array.size();
  // Create the vertices for the points and lines
  for (int i = 0; i < rrt_size; i++)
  {
    geometry_msgs::Point p;
    p.x = rrt.array[i].x;
    p.y = rrt.array[i].y;
    p.z = rrt.array[i].z;

    cp_points.points.push_back(p);
    line_strip.points.push_back(p);
  }

  bs_marker_pub.publish(cp_points);
  bs_marker_pub.publish(line_strip);
}

void start_end_callback(const rrtstar_ros::start_end_markers::ConstPtr &msg)
{
  rrtstar_ros::start_end_markers _msg = *msg;
  int size_markers = _msg.start.size();

  visualization_msgs::Marker start;
  visualization_msgs::Marker end;
  start.header.frame_id = end.header.frame_id = "/map";
  start.header.stamp = end.header.stamp = ros::Time::now();
  start.ns = end.ns = "rrtstar_visualization";
  start.action = end.action = visualization_msgs::Marker::ADD;
  start.pose.orientation.w = end.pose.orientation.w = 1.0;

  start.id = 0;
  end.id = 1;

  start.type = visualization_msgs::Marker::POINTS;
  end.type = visualization_msgs::Marker::POINTS;


  // POINTS markers use x and y scale for width/height respectively
  start.scale.x = 0.4;
  start.scale.y = 0.4;

  // POINTS markers use x and y scale for width/height respectively
  end.scale.x = 0.4;
  end.scale.y = 0.4;

  // // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  // line_strip.scale.x = 0.03;

  // Points are green
  start.color.g = 1.0;
  start.color.a = 1.0;

  // Points are red
  end.color.r = 1.0;
  end.color.a = 1.0;

  // Create the vertices for the points and lines
  for (int i = 0; i < size_markers; i++)
  {
    geometry_msgs::Point s = _msg.start[i];
    geometry_msgs::Point e = _msg.end[i];

    start.points.push_back(s);
    end.points.push_back(e);
  }

  start_end_marker_pub.publish(start);
  start_end_marker_pub.publish(end);
}

int main( int argc, char** argv )
{
  double rate = 1.0;
  
  ros::init(argc, argv, "rrtstar_visualization");
  ros::NodeHandle n("~");
  
  n.param<double>("obs_threshold", _obs_threshold, 1.0);
  n.param<double>("z_scale", _scale_z, 1.0);

  bs_marker_pub = n.advertise<visualization_msgs::Marker>(
        "/bs_visualization_marker", 10);
  rrt_marker_pub = n.advertise<visualization_msgs::Marker>(
        "/rrt_visualization_marker", 10);
  start_end_marker_pub = n.advertise<visualization_msgs::Marker>(
        "/start_end_visualization_marker", 10);
  constrains_marker_pub = n.advertise<visualization_msgs::Marker>(
        "/constrains_visualization_marker", 10);
  ellipsoid_marker_pub = n.advertise<visualization_msgs::Marker>(
        "/ellipsoid_visualization_marker", 10);

  bs_message = n.subscribe<rrtstar_ros::point_array>(
        "/bs", 10, &bs_callback);      
  rrt_message = n.subscribe<rrtstar_ros::point_array>(
        "/rrt", 10, &rrt_callback);
  bounding_box_message = n.subscribe<rrtstar_ros::bounding_box_array>(
        "/bb", 10, &constrain_callback);
  start_end_message = n.subscribe<rrtstar_ros::start_end_markers>(
        "/start_end_markers", 10, &start_end_callback);

  ros::Rate r(rate);

  ros::spin();

  return 0;
}


  



