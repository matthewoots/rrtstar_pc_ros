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
#include "bspline.h"

#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
    bs::bspline _bsp;

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
        printf("%s[common_utils.h] Completed %s size %d\n", KGRN, KNRM, start.size());
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
    * @brief Filter point cloud (ros) with the dimensions given
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr 
        pcl2_filter(sensor_msgs::PointCloud2 _pc, 
        Vector3d centroid, Vector3d dimension)
    {
        pcl::PCLPointCloud2 pcl_pc2;
        // printf("%s[rrtstar.h] ros_pcl2 to pcl! \n", KBLU);
        pcl_conversions::toPCL(_pc, pcl_pc2);

        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        
        // printf("%s[rrtstar.h] fromPCLPointCloud2! \n", KBLU);
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

        // printf("%s[rrtstar.h] return fromPCLPointCloud2! \n", KBLU);
        return output;
    }

    /* 
    * @brief Filter point cloud (pcl ptr) with the dimensions given
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr 
        pcl_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr _pc, 
        Vector3d centroid, Vector3d dimension)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        
        float minX = centroid.x() - dimension.x()/2;
        float maxX = centroid.x() + dimension.x()/2;

        float minY = centroid.y() - dimension.y()/2;
        float maxY = centroid.y() + dimension.y()/2;

        float minZ = centroid.z() - dimension.z()/2;
        float maxZ = centroid.z() + dimension.z()/2;

        pcl::CropBox<pcl::PointXYZ> box_filter;
        box_filter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
        box_filter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));

        box_filter.setInputCloud(_pc);
        box_filter.filter(*output);

        // printf("%s[rrtstar.h] return fromPCLPointCloud2! \n", KBLU);
        return output;
    }

    /* 
    * @brief Transform sensor cloud according to the translation and rpy given
    */
    sensor_msgs::PointCloud2 transform_sensor_cloud(sensor_msgs::PointCloud2 _pc,
        Vector3d _rpy, Vector3d _translation)
    {
        sensor_msgs::PointCloud2 transformed_cloud;

        geometry_msgs::TransformStamped transform;
        geometry_msgs::Quaternion q; geometry_msgs::Vector3 t;
        tf2::Quaternion quat_tf;

        t.x = _translation.x(); t.y = _translation.y(); t.z = _translation.z(); 

        double deg2rad = 1.0 / 180.0 * 3.1415926535;

        quat_tf.setRPY(_rpy.x() * deg2rad, 
            _rpy.y() * deg2rad, 
            _rpy.z() * deg2rad); // Create this quaternion from roll/pitch/yaw (in radians)
        q = tf2::toMsg(quat_tf);

        transform.transform.translation = t;
        transform.transform.rotation = q;
        transform.child_frame_id = "/base";
        transform.header.frame_id = "/map";

        tf2::doTransform(_pc, transformed_cloud, transform);

        return transformed_cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr base_to_transform_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr _pc, Vector3d rotation, Vector3d translation)
    {
        // https://github.com/felipepolido/EigenExamples
        // for affine3d examples
        geometry_msgs::Quaternion q;
        tf2::Quaternion quat_tf;
        double deg2rad = - 1.0 / 180.0 * 3.1415926535;

        quat_tf.setRPY(rotation.x() * deg2rad, 
            rotation.y() * deg2rad, 
            rotation.z() * deg2rad); // Create this quaternion from roll/pitch/yaw (in radians)
        q = tf2::toMsg(quat_tf);
        
        // w,x,y,z
        Eigen::Quaterniond rot(q.w, q.x, q.y, q.z);
        rot.normalize();

        Eigen::Quaterniond p;
        p.w() = 0;
        p.vec() = - translation;
        Eigen::Quaterniond rotatedP = rot * p * rot.inverse(); 
        Eigen::Vector3d rotatedV = rotatedP.vec();

        Eigen::Affine3d aff_t = Eigen::Affine3d::Identity();
        Eigen::Affine3d aff_r = Eigen::Affine3d::Identity();
        // aff_t.translation() = - translation;
        aff_r.translation() = rotatedV;
        aff_r.linear() = rot.toRotationMatrix();
        // pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pcl_trans(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pcl_rot(new pcl::PointCloud<pcl::PointXYZ>);
        // transformPointCloud(*_pc, *tmp_pcl_trans, aff_t, true);
        transformPointCloud(*_pc, *tmp_pcl_rot, aff_r, true);

        return tmp_pcl_rot;

        // *** Remove multiple conversions that will take up more time ***
        // *** Replace with solely Eigen calculation ***
        // sensor_msgs::PointCloud2 ros_pc;
        // pcl::toROSMsg(*_pc, ros_pc);
        // // Translate then rotate to temporary frame for point clouds
        // sensor_msgs::PointCloud2 tmp_transformed_pc = transform_sensor_cloud(ros_pc, - Vector3d(0, 0, 0), - translation);
        // tmp_transformed_pc = transform_sensor_cloud(tmp_transformed_pc, - rotation, Vector3d(0, 0, 0));
    
        // return pcl2_converter(tmp_transformed_pc);
    }

    geometry_msgs::Point vector_to_point(Vector3d v)
    {
        geometry_msgs::Point tmp;
        tmp.x = v.x(); 
        tmp.y = v.y(); 
        tmp.z = v.z();

        return tmp;
    }

    Vector3d point_to_vector(geometry_msgs::Point p)
    {
        Vector3d tmp;
        tmp.x() = p.x; 
        tmp.y() = p.y; 
        tmp.z() = p.z;

        return tmp;
    }

    geometry_msgs::Quaternion rpy_to_quaternion(Vector3d _rpy)
    {
        geometry_msgs::Quaternion q;
        tf2::Quaternion quat_tf;

        double deg2rad = 1.0 / 180.0 * 3.1415926535;

        quat_tf.setRPY(_rpy.x() * deg2rad, 
            _rpy.y() * deg2rad, 
            _rpy.z() * deg2rad); // Create this quaternion from roll/pitch/yaw (in radians)
        q = tf2::toMsg(quat_tf);
        return q;
    }

    /* 
    * @brief Transform pose according to the translation and rpy given
    */
    geometry_msgs::Point transform_point(geometry_msgs::Point _p,
        Vector3d _rpy, Vector3d _translation)
    {
        geometry_msgs::Point point;

        geometry_msgs::TransformStamped transform;
        geometry_msgs::Quaternion q; geometry_msgs::Vector3 t;
        tf2::Quaternion quat_tf;

        t.x = _translation.x(); t.y = _translation.y(); t.z = _translation.z(); 

        double deg2rad = 1.0 / 180.0 * 3.1415926535;

        quat_tf.setRPY(_rpy.x() * deg2rad, 
            _rpy.y() * deg2rad, 
            _rpy.z() * deg2rad); // Create this quaternion from roll/pitch/yaw (in radians)
        q = tf2::toMsg(quat_tf);

        transform.transform.translation = t;
        transform.transform.rotation = q;
        transform.child_frame_id = "/base";
        transform.header.frame_id = "/map";

        tf2::doTransform(_p, point, transform);

        return point;
    }

    /* 
    * @brief Forward transform pose according to the translation and rpy given
    */
    geometry_msgs::Point forward_transform_point(geometry_msgs::Point _p,
        Vector3d _rpy, Vector3d _translation)
    {
        geometry_msgs::Point tmp = transform_point(_p,
            - Vector3d(0, 0, 0), - _translation);
        tmp = transform_point(tmp,
            - Vector3d(0, 0, _rpy.z()), Vector3d(0, 0, 0));
        return tmp;
    }

    /* 
    * @brief Backward transform pose according to the translation and rpy given
    */
    geometry_msgs::Point backward_transform_point(geometry_msgs::Point _p,
        Vector3d _rpy, Vector3d _translation)
    {
        geometry_msgs::Point tmp = transform_point(_p,
            - Vector3d(0, 0, - _rpy.z()), Vector3d(0, 0, 0));
        tmp = transform_point(tmp, - Vector3d(0, 0, 0), _translation);

        return tmp;
    }

    /* 
    * @brief Filter point cloud with the dimensions given
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr 
        pcl2_filter_ptr(pcl::PointCloud<pcl::PointXYZ>::Ptr _pc, 
        Vector3d centroid, Vector3d dimension)
    {   
        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);

        float minX = centroid.x() - dimension.x()/2;
        float maxX = centroid.x() + dimension.x()/2;

        float minY = centroid.y() - dimension.y()/2;
        float maxY = centroid.y() + dimension.y()/2;

        float minZ = centroid.z() - dimension.z()/2;
        float maxZ = centroid.z() + dimension.z()/2;

        pcl::CropBox<pcl::PointXYZ> box_filter;
        box_filter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
        box_filter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));

        box_filter.setInputCloud(_pc);
        box_filter.filter(*output);

        // printf("%s[rrtstar.h] return fromPCLPointCloud2! \n", KBLU);
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
    Vector3d findBoundary(Vector3d p, Vector3d q, double passage_size, double xy_buffer, double z_buffer)
    {
        Vector3d boundary;
        boundary.x() = abs(p.x() - q.x()) + xy_buffer;
        boundary.y() = abs(p.y() - q.y()) + xy_buffer + passage_size;
        boundary.z() = abs(p.z() - q.z()) + z_buffer;
        
        return boundary;
    }

    VectorXd linspace(double min, double max, double n)
    {
        VectorXd linspaced((int)n);
        double delta = (max - min) / (n - 1.0);
        linspaced(0) = min;
        
        for (int i = 1; i < (int)n; i++)
        {
            linspaced(i) = (linspaced(i-1) + delta);
        }
        return linspaced;
    }
    
    MatrixXd setClampedPath(MatrixXd wp, 
        double max_vel, double _knot_span, int _order, 
        Vector3d start_pose) 
    {
        /* 
        * Uniform Distribution
        */
        MatrixXd cp_raw = MatrixXd::Zero(3,1); VectorXd time_waypoint = VectorXd::Zero(1);

        _bsp.UniformDistribution(start_pose, wp, max_vel, _knot_span, 
            &time_waypoint, &cp_raw);
        std::cout << KYEL << "[common_utils.h] " << "Uniform Distribution Complete" << KNRM << std::endl;

        /* 
        * Clamp the Bspline
        */
        // Update global control points and start and end time 
        MatrixXd _global_cp = _bsp.ClampBspline(_order, cp_raw);
        std::cout << KYEL << "[common_utils.h] " << "Clamping Bspline" << KNRM << std::endl;

        return _global_cp;
    }


    VectorXd setKnotsPath(MatrixXd _global_cp, double   _knot_span, int _order)
    {
        double _start = ros::Time::now().toSec();
        double _end = _start + ((_global_cp.cols() - (_order)) * _knot_span);
        MatrixXd _fixed_knots_tmp = linspace(_start, _end, (double)(_global_cp.cols() - (_order-1)));

        return _fixed_knots_tmp.row(0);
    }


    /** 
    * @brief Update Full Path
    */
    std::vector<Vector3d> updateFullPath(MatrixXd _global_cp, int _knotdiv, int _order, VectorXd _fixed_knots)
    {
        // Reset pos, vel, acc and time
        MatrixXd _pos = MatrixXd::Zero(3,1); 
        MatrixXd _vel = MatrixXd::Zero(3,1); 
        MatrixXd _acc = MatrixXd::Zero(3,1); 
        VectorXd _time = VectorXd::Zero(1);

        // Truncate control points from global to local
        MatrixXd _local_cp = MatrixXd::Zero(3,_global_cp.cols());

        for(int i = 0; i < _local_cp.cols(); i++)
        {
            _local_cp.col(i) = _global_cp.col(i);
        }

        double start = _fixed_knots(0); 
        double end = _fixed_knots(_fixed_knots.size()-1);

        // Bspline Creation using Function
        _bsp.GetBspline3(_order, _local_cp, start, end, _knotdiv, &_pos, &_vel, &_acc, &_time);

        std::cout << KYEL << "[common_utils.h] " << "Full Path Update Complete" << KNRM << std::endl;

        int pos_size = _pos.cols();
        std::vector<Vector3d> tmp;
        for (int i = 0; i < pos_size; i++)
        {
            Vector3d tmp_v;
            tmp_v.x() = _pos.col(i).x();
            tmp_v.y() = _pos.col(i).y();
            tmp_v.z() = _pos.col(i).z();

            tmp.push_back(tmp_v);
        }

        return tmp;
    }

};