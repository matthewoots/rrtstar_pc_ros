/*
 * main.cpp
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

#include "rrtstar.h"

#include <vector>
#include <ros/ros.h>
#include <Eigen/Dense>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

#include "rrtstar_ros/rrt_array.h"
#include "rrtstar_ros/rrt_data.h"
#include "rrtstar_ros/start_end_markers.h"

using namespace std;

rrtstar rrt;

class initiator
{
private:
    ros::NodeHandle _nh;
    std::vector<Vector3d> rrt_list;
    ros::Publisher rrt_pub;
    ros::Publisher start_end_pub;
    ros::Publisher bs_pub;
    ros::Publisher altered_pcl_pub;
    ros::Subscriber pcl_sub;

public:
    sensor_msgs::PointCloud2 pcl_pc2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc;
    pcl::PointCloud<pcl::PointXYZ>::Ptr actual_pc;

    initiator(ros::NodeHandle &nodeHandle)
    {
        /** 
        * @brief Publisher of rrt points
        */
        bs_pub = _nh.advertise<rrtstar_ros::rrt_array>("/bs", 10);
        rrt_pub = _nh.advertise<rrtstar_ros::rrt_array>("/rrt", 10);
        start_end_pub = _nh.advertise<rrtstar_ros::start_end_markers>("/start_end_markers", 10);
        altered_pcl_pub = _nh.advertise<sensor_msgs::PointCloud2>("/new_pcl", 10);

        pcl_sub = _nh.subscribe("/pcl", 1, &initiator::pcl2_callback, this);
        // pcl_sub = _nh.subscribe("/pcl", 1, initiator::pcl2_callback);
        printf("%s[main.cpp] Constructor Setup Ready! \n", KGRN);
    }
    ~initiator(){};

    void set_rrt_array(std::vector<Vector3d> _rrt_list)
    {
        rrt_list.clear();
        rrt_list = _rrt_list;
        return;
    }

    void path_message_wrapper_publisher(Vector3d s)
    {
        rrtstar_ros::rrt_array msg; rrtstar_ros::rrt_data data;

        int v_size = rrt_list.size();
        for (int i = 0; i < v_size; i++)
        {
            data.x = rrt_list[i].x(); data.y = rrt_list[i].y(); data.z = rrt_list[i].z();
            msg.array.push_back(data);
        }

        // Somehow the algorithm will miss the start data
        data.x = s.x(); data.y = s.y(); data.z = s.z();
        msg.array.push_back(data);

        rrt_pub.publish(msg);
    }

    void bspline_message_wrapper_publisher(int _order, Vector3d e)
    {
        rrtstar_ros::rrt_array msg; rrtstar_ros::rrt_data data;

        int v_size = rrt_list.size();
        MatrixXd wp = MatrixXd::Zero(3,v_size-1);
        
        for (int i = 0; i < v_size-1; i++)
        {
            wp(0,i) = rrt_list[i+1].x(); 
            wp(1,i) = rrt_list[i+1].y(); 
            wp(2,i) = rrt_list[i+1].z();
        }

        // Somehow the algorithm will miss the start data
        // Since the nodes are counting backwards, hence the start point is the end point
        Vector3d end_pose = e;

        MatrixXd global_cp = _common_utils.setClampedPath(wp, 
        4, 1, _order, end_pose);
        VectorXd knots = _common_utils.setKnotsPath(global_cp, 1, _order);
        std::vector<Vector3d> bs = _common_utils.updateFullPath(global_cp, 
            1, _order, knots);

        for (int i = 0; i < bs.size(); i++)
        {
            data.x = bs[i].x(); data.y = bs[i].y(); data.z = bs[i].z();
            msg.array.push_back(data);
        }

        bs_pub.publish(msg);
    }

    void new_pcl_publisher()
    {
        // ROS point cloud
        sensor_msgs::PointCloud2 msg;

        // PCL first generation point cloud-> ROS point cloud
        pcl::toROSMsg(*pc, msg);

        altered_pcl_pub.publish(msg);
    }

    void points_message_wrapper_publisher(
            std::vector<Vector3d> _start,
            std::vector<Vector3d> _end)
    {
        rrtstar_ros::start_end_markers msg;

        int v_size = _start.size();
        for (int i = 0; i < v_size; i++)
        {
            geometry_msgs::Point s, e; 
            s.x = _start[i].x();
            s.y = _start[i].y();
            s.z = _start[i].z();

            e.x = _end[i].x();
            e.y = _end[i].y();
            e.z = _end[i].z(); 

            msg.start.push_back(s);
            msg.end.push_back(e);
        }

        start_end_pub.publish(msg);
    }

    void pcl2_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        pcl_pc2 = *msg;
    }
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "rrtstar_node");
    ros::NodeHandle _nh("~"); 

    initiator initiator(_nh);
    
    // ROS Params
    std::string _file_location;
    double _step_size;
    double _obs_threshold;
    double _random_multiplier;
    int _line_search_division;
    double _xybuffer, _zbuffer;
    double _start_delay;
    double _min_height, _max_height;
    double _bs_order;

    _nh.param<std::string>("file_location", _file_location, "/home");
    _nh.param<double>("step_size", _step_size, 1.0);
    _nh.param<double>("obs_threshold", _obs_threshold, 1.0);
    _nh.param<double>("random_multiplier", _random_multiplier, 1.0);
    _nh.param<int>("line_search_division", _line_search_division, 1);
    _nh.param<double>("xybuffer", _xybuffer, 1.0);
    _nh.param<double>("zbuffer", _zbuffer, 1.0);
    _nh.param<double>("start_delay", _start_delay, 1.0);

    _nh.param<double>("min_height", _min_height, 1.0);
    _nh.param<double>("max_height", _max_height, 1.0);

    _nh.param<double>("bs_order", _bs_order, 1.0);
    
    // Sleep for 3 second to let mockamap initialise the map
    ros::Duration(_start_delay).sleep();

    std::vector<Vector3d> start;
    std::vector<Vector3d> end; 
    
    ros::spinOnce();

    printf("%s[main.cpp] Unpacking waypoint! \n", KBLU);
    // Input to the system will be given from csv file
    if (!_common_utils.UnpackWaypoint(&start, &end, _file_location))
    {
        ROS_ERROR("Not able to find file location %s", _file_location.c_str());
        return 0;
    }

    // Total number of paths we have to find
    int total = start.size();
    for (int i = 0; i < total; i++) 
    {
        printf("%s[main.cpp] Initial Start (%lf %lf %lf) End (%lf %lf %lf) \n", 
                KBLU, start[i].x(), start[i].y(), start[i].z(),
                end[i].x(), end[i].y(), end[i].z());
    }

    Vector3d _map_size;
    Vector3d _origin;
    // Check points to be obstacle-free
    for (int i = 0; i < total; i++) 
    {
        printf("%s[main.cpp] Waypoints Initialize Round %d! \n", KBLU, i);
        // kdtree_pcl return true if there is a point nearby
        // Check whether start point is in or near an obstacle
        while (rrt.kdtree_pcl(start[i], initiator.pcl_pc2, _obs_threshold))
        {
            start[i].x() = start[i].x() + ((double)(rand() % 100) / 100.0 - 0.5) * _random_multiplier;
            start[i].y() = start[i].y() + ((double)(rand() % 100) / 100.0 - 0.5) * _random_multiplier;
            start[i].z() = start[i].z() + ((double)(rand() % 100) / 100.0 - 0.5) * _random_multiplier;
            start[i].z() = max(start[i].z(), _min_height);
            start[i].z() = min(start[i].z(), _max_height);
        }

        // kdtree_pcl return true if there is a point nearby
        // Check whether end point is in or near an obstacle
        while (rrt.kdtree_pcl(end[i], initiator.pcl_pc2, _obs_threshold))
        {
            end[i].x() = end[i].x() + ((double)(rand() % 100) / 100.0 - 0.5) * _random_multiplier;
            end[i].y() = end[i].y() + ((double)(rand() % 100) / 100.0 - 0.5) * _random_multiplier;
            end[i].z() = end[i].z() + ((double)(rand() % 100) / 100.0 - 0.5) * _random_multiplier;
            end[i].z() = max(end[i].z(), _min_height);
            end[i].z() = min(end[i].z(), _max_height);
        }

        printf("%s[main.cpp] Start (%lf %lf %lf) End (%lf %lf %lf) \n", 
            KBLU, start[i].x(), start[i].y(), start[i].z(),
            end[i].x(), end[i].y(), end[i].z());

        // Map size and origin should be determined and isolated 
        // Find a way to rotate the boundary so that we can minimize the space
        _map_size = _common_utils.findBoundary(start[i], end[i], _xybuffer, _zbuffer);
        _origin = _common_utils.findCentroid(start[i], end[i]);

        double crop_timer = ros::Time::now().toSec();
        // We can crop the pointcloud to the dimensions that we are using
        initiator.pc = _common_utils.pcl2_filter(initiator.pcl_pc2, _origin, _map_size);
        initiator.actual_pc = _common_utils.pcl2_converter(initiator.pcl_pc2);
        printf("%s[main.cpp] Time taken to crop obstacle %lf! \n", KGRN, ros::Time::now().toSec() - crop_timer);
    }
    // We can advertise the start and end as markers
    initiator.points_message_wrapper_publisher(start, end);
    initiator.new_pcl_publisher();

    for (int i = 0; i < total; i++) 
    {
        size_t num_points = initiator.actual_pc->size();
        int total = static_cast<int>(num_points);
        printf("%s[main.cpp] Round %d! \n", KBLU, i);
        printf("%s[main.cpp] Actual obstacle size %d! \n", KGRN, total);
        rrt.initialize(start[i], end[i], initiator.pc,
            _map_size, _origin,
            _step_size, _obs_threshold,
            _min_height, _max_height,
            _line_search_division);

        rrt.run();

        if (rrt.process_status())
        {
            printf("%s[main.cpp] Will not continue bspline and publishing process! %s\n", KRED, KNRM);
            return 0;
        }

        // Extract path
        std::vector<Vector3d> path = rrt.path_extraction();

        initiator.set_rrt_array(path);

        initiator.path_message_wrapper_publisher(start[i]);
        initiator.bspline_message_wrapper_publisher(_bs_order, end[i]);

        ros::spin();
        //ros::Duration(20.0).sleep();
    }

    return 0;
}