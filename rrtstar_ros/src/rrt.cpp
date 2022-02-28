/*
 * rrt.cpp
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
#include "rrt_standalone.h"

#include <vector>
#include <ros/ros.h>
#include <Eigen/Dense>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include "rrtstar_ros/point_array.h"

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

using namespace std;
// using namespace rrt_helper;

std::vector<Vector3d> rrt_path;
double _step_size;
double _obs_threshold;

double _xybuffer, _zbuffer;

double _min_height, _max_height;
double _passage_size;

int _max_tries;
double _timeout;
double _scale_z;

bool pcl_received;

double yaw;
Vector3d translation;
pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pc1(new pcl::PointCloud<pcl::PointXYZ>);;

std::vector<Vector3d> bspline;

class simple_node
{
private:

    ros::NodeHandle _nh;
    ros::Publisher rrt_pub;
    ros::Subscriber pcl_sub;

    // For debug
    ros::Publisher altered_pcl_pub, bs_pub;

public:
    sensor_msgs::PointCloud2 pcl_pc2;


    simple_node(ros::NodeHandle &nodeHandle)
    {
        /** 
        * @brief Publisher of rrt points
        */
        rrt_pub = _nh.advertise<rrtstar_ros::point_array>("/rrt", 10);

        pcl_sub = _nh.subscribe("/pcl", 1, &simple_node::pcl2_callback, this);
        // pcl_sub = _nh.subscribe("/pcl", 1, simple_node::pcl2_callback);
        printf("%s[rrt.cpp] Constructor Setup Ready! \n", KGRN);

        // For debug
        altered_pcl_pub = _nh.advertise<sensor_msgs::PointCloud2>("/query_pcl", 10);
        bs_pub = _nh.advertise<rrtstar_ros::point_array>("/bs", 10);
    }
    ~simple_node(){};

    void path_message_wrapper_publisher()
    {
        rrtstar_ros::point_array msg; geometry_msgs::Point data;

        int v_size = rrt_path.size();
        for (int i = 0; i < v_size; i++)
        {
            data.x = rrt_path[i].x(); data.y = rrt_path[i].y(); data.z = rrt_path[i].z();
            msg.array.push_back(data);
        }

        rrt_pub.publish(msg);
    }

    void pcl2_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        pcl_pc2 = *msg;
        pcl_received = true;
    }

    void query_pcl_publisher(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pc,
        double _yaw, Vector3d _translation)
    {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*transformed_pc, msg);
        sensor_msgs::PointCloud2 tmp_pc = transform_sensor_cloud(msg,
            - Vector3d(0, 0, - _yaw), Vector3d(0, 0, 0));
        msg = transform_sensor_cloud(tmp_pc,
            - Vector3d(0, 0, 0), _translation);

        altered_pcl_pub.publish(msg);
    }

    void bspline_publisher()
    {
        rrtstar_ros::point_array msg; 

        for (int i = 0; i < bspline.size(); i++)
        {
            geometry_msgs::Point data;
            data.x = bspline[i].x(); data.y = bspline[i].y(); data.z = bspline[i].z();
            msg.array.push_back(data);
        }

        printf("%s[main.cpp] Bspline message converted %s\n", KGRN, KNRM);

        bs_pub.publish(msg);
    }
};



bool RRT(sensor_msgs::PointCloud2 pcl_pc, 
    Vector3d start, Vector3d end, vector<VectorXd> no_fly_zone)
{
    rrt_path.clear();

    pcl::PointCloud<pcl::PointXYZ>::Ptr original_pcl_pc = 
        pcl2_converter(pcl_pc);

    printf("%s[rrt.cpp] Start (%lf %lf %lf) End (%lf %lf %lf) \n", 
        KBLU, start.x(), start.y(), start.z(),
        end.x(), end.y(), end.z());

    // Find the origin of the transformed frame
    Vector3d _origin;
    _origin.x() = (start.x() + end.x()) / 2;
    _origin.y() = (start.y() + end.y()) / 2;
    _origin.z() = (start.z() + end.z()) / 2;
    
    // Do the preparation for transformation
    // Find the translation vector and the yaw angle
    Vector3d tmp_vect = end - start;
    yaw = atan2(tmp_vect.y(), tmp_vect.x()) / 3.1415926535 * 180;
    Vector3d rotation = Vector3d(0,0,yaw);

    translation = Vector3d(_origin.x(), _origin.y(), 0);       
    
    printf("%s[rrt.cpp] translation vector [%lf %lf %lf] yaw %lf\n", KBLU, 
        translation.x(), translation.y(), translation.z(), yaw);

    Vector3d transformed_translation = rotate_vector(rotation, translation);

    // We align everthing to the rotated x axis
    // So now we are playing in X and Z axis
    // Translate then rotate to temporary frame for start and end points
    // geometry_msgs::Point transformed_start = transform_point(
    //     vector_to_point(start), -rotation, transformed_translation);
    // geometry_msgs::Point transformed_end = transform_point(
    //     vector_to_point(end), -rotation, transformed_translation);

    geometry_msgs::Point transformed_start = forward_transform_point(
        vector_to_point(start), rotation, translation);
    geometry_msgs::Point transformed_end = forward_transform_point(
        vector_to_point(end), rotation, translation);

    Vector3d t_start = point_to_vector(transformed_start);
    Vector3d t_end = point_to_vector(transformed_end);

    printf("%s[rrt.cpp] transformed start (%lf %lf %lf) transformed end (%lf %lf %lf) \n", 
        KBLU, t_start.x(), t_start.y(), t_start.z(),
        t_end.x(), t_end.y(), t_end.z());

    // We find the original pcl in the transformed frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pcl_pc =
        base_to_transform_pcl(original_pcl_pc, 
        Vector3d(0, 0, yaw), translation);

    // Map size and origin should be determined and isolated 
    // Find a way to rotate the boundary so that we can minimize the space
    Vector3d _map_size;
    _map_size.x() = abs(t_start.x() - t_end.x()) + _xybuffer;
    _map_size.y() = abs(t_start.y() - t_end.y()) + _xybuffer + _passage_size;
    _map_size.z() = abs(t_start.z() - t_end.z()) + _zbuffer;

    printf("%s[rrt.cpp] map_size start (%lf %lf %lf)\n", 
        KBLU, _map_size.x(), _map_size.y(), _map_size.z());

    // We can crop the pointcloud to the dimensions that we are using
    // Origin will already to (0,0,0)
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cropped_pc1 = pcl2_filter(
        transformed_pcl_pc, Vector3d(0,0,_origin.z()), _map_size);

    // *** For DEBUG ***
    transformed_pc1->points.clear();
    transformed_pc1 = transformed_cropped_pc1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cropped_pc = 
        pcl_z_scale(transformed_cropped_pc1, _scale_z);

    // map size is affected too
    // _min_height and _max_height is different too

    // *** Added in transform ***
    Vector3d map_size = Vector3d(_map_size.x(), _map_size.y(), _map_size.z() * _scale_z);
    
    double min_height = _min_height * _scale_z;
    double max_height = _max_height * _scale_z;

    double t_z = _origin.z() * _scale_z;

    double step_size = _step_size + _scale_z / 2.0;

    Vector3d t_t_start = Vector3d(t_start.x(), t_start.y(), t_start.z() *_scale_z);
    Vector3d t_t_end = Vector3d(t_end.x(), t_end.y(), t_end.z() *_scale_z);
    // *** End of adding in transform ***
    
    // Set line division to floor of step_size
    // Size must be 3 or more for linspace to work
    int line_search_division = max(4,(int)floor(step_size)); 

    size_t num_points = original_pcl_pc->size();
    int total = static_cast<int>(num_points);
    printf("%s[rrt.cpp] Actual obstacle size %d! \n", KGRN, total);

    // Run RRT 
    rrt_node rrt;
    rrt.error = true;
    int rrt_tries = 0;
    while (rrt_tries < _max_tries && rrt.process_status())
    {
        // Use for transformed start and end in transformed frame
        rrt.initialize(t_t_start, t_t_end, transformed_cropped_pc,
            map_size, Vector3d(0,0,t_z),
            step_size, _obs_threshold,
            min_height, max_height,
            line_search_division, _timeout, no_fly_zone,
            rotation, translation);
        rrt.run();
        rrt_tries++;
    }

    if (rrt_tries > _max_tries)
    {
        printf("%s[rrt.cpp] Will not continue bspline and publishing process! %s\n", KRED, KNRM);
        return false;
    }

    if (rrt.process_status())
    {
        printf("%s[rrt.cpp] Will not continue bspline and publishing process! %s\n", KRED, KNRM);
        return false;
    }

    printf("%s[rrt.cpp] rrt_size %d! %s\n", KGRN, (rrt.path_extraction()).size(), KNRM);


    // Extract path from transformed frame into normal frame
    // Remember to factor in z scale back
    std::vector<Vector3d> transformed_path1 = rrt.path_extraction();
    std::vector<Vector3d> transformed_path;

    // Transformed path needs to scale back the z
    for (int j = 0; j < transformed_path1.size(); j++)
    {
        Vector3d tmp_path = Vector3d(transformed_path1[j].x(), 
            transformed_path1[j].y(), transformed_path1[j].z() / _scale_z);

        transformed_path.push_back(tmp_path);
    }

    std::vector<Vector3d> path;

    // Vector3d transformed_translation_to_original = rotate_vector(
    //     -rotation, -translation);
    for (int j = 0; j < transformed_path.size(); j++)
    {
        Vector3d tmp_path = transformed_path[j];
        
        // geometry_msgs::Point n_tmp_path = transform_point(
        //     vector_to_point(tmp_path),
        //     -rotation, transformed_translation_to_original);
        geometry_msgs::Point n_tmp_path = backward_transform_point(
            vector_to_point(tmp_path), rotation, translation);


        path.push_back(point_to_vector(n_tmp_path));
    }

    // Somehow the algorithm will miss the start data
    path.push_back(start);

    // Now the whole path is flipped, hence we need to flip it back in bspline

    rrt_path = path;
    
    // create_bspline(_bs_order, end, start);
    return true;
}

void rrt_bspline(int _order)
{
    bspline.clear();

    int v_size = rrt_path.size();
    MatrixXd wp = MatrixXd::Zero(3,v_size);
    
    // We need to flip it back since RRT path is inverted 
    for (int i = rrt_path.size()-1; i >= 0; i--)
    {
        wp(0,rrt_path.size()-1 - i) = rrt_path[i].x(); 
        wp(1,rrt_path.size()-1 - i) = rrt_path[i].y(); 
        wp(2,rrt_path.size()-1 - i) = rrt_path[i].z();
    }

    // Somehow the algorithm will miss the start data
    // wp(0,v_size-1) = s.x(); 
    // wp(1,v_size-1) = s.y(); 
    // wp(2,v_size-1) = s.z();

    // // Somehow the algorithm will miss the start data
    // // Since the nodes are counting backwards, hence the start point is the end point
    // Vector3d end_pose = e;

    // Start position will be at rrt_path.size()-1
    MatrixXd global_cp = setClampedPath(wp, 
    2, 4, _order, rrt_path[rrt_path.size()-1]);
    VectorXd knots = setKnotsPath(global_cp, 1, _order);
    std::vector<Vector3d> bs_tmp = updateFullPath(global_cp, 
        1, _order, knots);

    // Since the nodes are flipped we have to flip them back first
    // Change back the order
    for (int i = 0; i < bs_tmp.size(); i++)
        bspline.push_back(bs_tmp[i]);


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rrt_simple_node");
    ros::NodeHandle _nh("~"); 

    simple_node simple_node(_nh);
    
    // ROS Params
    _nh.param<double>("step_size", _step_size, 1.0);
    _nh.param<double>("obs_threshold", _obs_threshold, 1.0);
    _nh.param<double>("xybuffer", _xybuffer, 1.0);
    _nh.param<double>("zbuffer", _zbuffer, 1.0);
    _nh.param<double>("passage_size", _passage_size, 1.0);

    _nh.param<double>("min_height", _min_height, 1.0);
    _nh.param<double>("max_height", _max_height, 1.0);
    _nh.param<int>("max_tries", _max_tries, 1.0);
    _nh.param<double>("timeout", _timeout, 0.1);
    _nh.param<double>("z_scale", _scale_z, 1.0);
    
    // *** For DEBUG ***
    


    while (ros::ok())
    {
        if (pcl_received)
        {
            // x_min, x_max, y_min, y_max
            vector<VectorXd> no_fly_zone;

            // Wall
            // Vector3d start = Vector3d(0,0,1.5);
            // Vector3d end = Vector3d(12,0,1.5);

            // Pole
            Vector3d start = Vector3d(13,-2.5,1.5);
            Vector3d end = Vector3d(27.5,-2.5,1.5);

            VectorXd a(4); 
            a(0) = 15.0; a(1) =  25.0; a(2) = -5.0; a(3) = -1.6;
            no_fly_zone.push_back(a);
            a(0) = 15.0; a(1) =  25.0; a(2) = 1.6; a(3) = 5.0;
            no_fly_zone.push_back(a);


            // Square Hoops
            // Vector3d start = Vector3d(31,0,1.5);
            // Vector3d end = Vector3d(30,-8,1.5);

            // Circle Hoops
            // Vector3d start = Vector3d(22.5,-10,1.5);
            // Vector3d end = Vector3d(17.5,-10,1.5);

            // Triangle Hoops
            // Vector3d start = Vector3d(12.5,-10,1.5);
            // Vector3d end = Vector3d(7.5,-12,1.5);


            if (RRT(simple_node.pcl_pc2, start, end, no_fly_zone))
            {
                rrt_bspline(3);
                printf("%s[rrt.cpp] RRT Succeeded\n", KGRN);

                // For Debug
                simple_node.path_message_wrapper_publisher();
                simple_node.query_pcl_publisher(transformed_pc1, yaw, translation);
                simple_node.bspline_publisher();
            }
            else
                return 0;
        }

        ros::spinOnce();
        ros::Duration(1.5).sleep();
    }

    return 0;
}