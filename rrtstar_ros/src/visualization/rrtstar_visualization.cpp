/*
 * rrtstar_visualization.cpp
 *
 * ---------------------------------------------------------------------
 * Copyright (C) 2021 Matthew (matthewoots at gmail.com)
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

#include "rrtstar_ros/rrt_array.h"
#include "rrtstar_ros/rrt_data.h"
#include "rrtstar_ros/start_end_markers.h"

using namespace std;
using namespace Eigen;

ros::Publisher rrt_marker_pub;
ros::Publisher start_end_marker_pub;
ros::Subscriber rrt_message;
ros::Subscriber start_end_message;

void rrt_callback(const rrtstar_ros::rrt_array::ConstPtr &msg)
{
  rrtstar_ros::rrt_array rrt = *msg;

  visualization_msgs::Marker rrt_points, line_strip;
  rrt_points.header.frame_id = line_strip.header.frame_id = "/map";
  rrt_points.header.stamp = line_strip.header.stamp = ros::Time::now();
  rrt_points.ns = line_strip.ns = "bspline_visualization_points";
  rrt_points.action = line_strip.action = visualization_msgs::Marker::ADD;
  rrt_points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

  rrt_points.id = 0;
  line_strip.id = 1;

  rrt_points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;


  // POINTS markers use x and y scale for width/height respectively
  rrt_points.scale.x = 0.1;
  rrt_points.scale.y = 0.1;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.08;

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
  }

  rrt_marker_pub.publish(rrt_points);
  rrt_marker_pub.publish(line_strip);
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
  int _size;  
  ros::init(argc, argv, "rrtstar_visualization");
  ros::NodeHandle n("~");
  
  n.param<int>("marker_size", _size, 4);

  rrt_marker_pub = n.advertise<visualization_msgs::Marker>(
        "/rrt_visualization_marker", 10);
  start_end_marker_pub = n.advertise<visualization_msgs::Marker>(
        "/start_end_visualization_marker", 10);
  rrt_message = n.subscribe<rrtstar_ros::rrt_array>(
        "/rrt", 10, &rrt_callback);
  start_end_message = n.subscribe<rrtstar_ros::start_end_markers>(
        "/start_end_markers", 10, &start_end_callback);

  ros::Rate r(rate);

  ros::spin();

  return 0;
}


  



