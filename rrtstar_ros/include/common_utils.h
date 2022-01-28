/*
 * common_utils.h
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

#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <vector>

#include "csv.h"

#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

using namespace Eigen;
using namespace std;

class common_utility
{
public:

    /* 
    * @brief Convert csv start and end waypoints to Vector3d form
    */
    bool UnpackWaypoint(std::vector<Vector3d> *_start, std::vector<Vector3d> *_end, std::string _file_location)
    {
        printf("%s[common_utils.h] Trying to open %s \n", KYEL, _file_location.c_str());
        ifstream file(_file_location);
        
        if (!file)
        {
            printf("%s[common_utils.h] File not present! \n", KRED);
            return false;
        }
        printf("%s[common_utils.h] Success, found %s \n", KGRN, _file_location.c_str());
        io::CSVReader<6> in(_file_location);

        in.read_header(io::ignore_extra_column, "x_start", "y_start", "z_start",
            "x_end", "y_end", "z_end");
        double xs, ys, zs, xe, ye, ze;
        std::vector<Vector3d> start, end;
        // First pass is to get number of rows
        while (in.read_row(xs, ys, zs, xe, ye, ze))
        {
            start.push_back(Vector3d(xs, ys, zs));
            end.push_back(Vector3d(xe, ye, ze));
        }

        *_start = start; *_end = end; 
        printf("%s[common_utils.h] Completed %s \n", KGRN, KNRM);
        return true;
    }

    /* 
    * @brief Convert point cloud from ROS sensor message to 
    * pcl point ptr
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr 
        pcl2_converter(sensor_msgs::PointCloud2 _pc)
    {
        pcl::PCLPointCloud2 pcl_pc2;
        printf("%s[rrtstar.h] ros_pcl2 to pcl! \n", KBLU);
        pcl_conversions::toPCL(_pc, pcl_pc2);

        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        printf("%s[rrtstar.h] fromPCLPointCloud2! \n", KBLU);
        pcl::fromPCLPointCloud2(pcl_pc2, *tmp_cloud);
        
        printf("%s[rrtstar.h] return fromPCLPointCloud2! \n", KBLU);
        return tmp_cloud;
    }

    /* 
    * @brief Filter point cloud with the dimensions given
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr 
        pcl2_filter(sensor_msgs::PointCloud2 _pc, 
        Vector3d centroid, Vector3d dimension)
    {
        pcl::PCLPointCloud2 pcl_pc2;
        printf("%s[rrtstar.h] ros_pcl2 to pcl! \n", KBLU);
        pcl_conversions::toPCL(_pc, pcl_pc2);

        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        
        printf("%s[rrtstar.h] fromPCLPointCloud2! \n", KBLU);
        pcl::fromPCLPointCloud2(pcl_pc2, *tmp_cloud);
        
        float minX = centroid.x() - dimension.x()/2;
        float maxX = centroid.x() + dimension.x()/2;

        float minY = centroid.y() - dimension.y()/2;
        float maxY = centroid.y() + dimension.y()/2;

        float minZ = centroid.z() - dimension.z()/2;
        float maxZ = centroid.z() + dimension.z()/2;

        pcl::CropBox<pcl::PointXYZ> box_filter;
        box_filter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
        box_filter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));

        box_filter.setInputCloud(tmp_cloud);
        box_filter.filter(*output);

        printf("%s[rrtstar.h] return fromPCLPointCloud2! \n", KBLU);
        return output;
    }

    /* 
    * @brief Find Centroid of a line
    */
    Vector3d findCentroid(Vector3d p, Vector3d q)
    {
        Vector3d centroid;
        centroid.x() = (p.x() + q.x()) / 2;
        centroid.y() = (p.y() + q.y()) / 2;
        centroid.z() = (p.z() + q.z()) / 2;
        return centroid;
    }

    /* 
    * @brief Find dimensions of a line with ability to add buffer
    */
    Vector3d findBoundary(Vector3d p, Vector3d q, double xy_buffer, double z_buffer)
    {
        Vector3d boundary;
        boundary.x() = abs(p.x() - q.x()) + xy_buffer;
        boundary.y() = abs(p.y() - q.y()) + xy_buffer;
        boundary.z() = abs(p.z() - q.z()) + z_buffer;
        
        return boundary;
    }
    

};