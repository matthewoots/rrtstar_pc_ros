/*
 * lbfgs_pcl_constrain.h
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

#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>
#include <Eigen/Dense>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>

using namespace Eigen;
using namespace std;

#define dmax std::numeric_limits<double>::max();

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"



class constrain
{   
    private:
        
        double max_boundaries, corridor_size, safety_radius; 
        int division;
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr query_pcl;
        int flag;
    
    public:

    std::vector<Vector3d> max_constrain, min_constrain;
    std::vector<Vector3d> rotation;
    std::vector<double> time;

    void query_point_contrains(std::vector<Vector3d> point_vector, pcl::PointCloud<pcl::PointXYZ>::Ptr _pc, double _max_boundaries, double _corridor_size, int _division, double _safety_radius)
    {
        rotation.clear(); max_constrain.clear(); 
        min_constrain.clear(); time.clear();
        flag = 0;

        max_boundaries = _max_boundaries;
        input_pcl = _pc;
        division = _division;
        corridor_size = _corridor_size;
        safety_radius = _safety_radius;

        double total_prev = ros::Time::now().toSec();
        // Run check on each query point
        // We don't count the first point
        printf("%s[lbfgs_pcl_constrain.h] Control Point Vector Size %d\n", 
                KGRN, point_vector.size());
        for (int i = 1; i < point_vector.size(); i++)
        {
            double single_prev = ros::Time::now().toSec();
            query_pcl = input_pcl;
            // Find the rotation vector
            Vector3d rot_vec = point_vector[i] - point_vector[i-1];
            double yaw_deg = atan2(rot_vec.y(), rot_vec.x()) / 3.1415926535 * 180;
            double xy = sqrt(pow(rot_vec.x(),2) + pow(rot_vec.y(),2));
            double pitch_deg = atan2(rot_vec.z(), xy) / 3.1415926535 * 180;
            printf("%s[lbfgs_pcl_constrain.h] Pitch %lf\n", 
                KCYN, pitch_deg);
            
            // NWU = RPY
            Vector3d rotation_vector = Vector3d(0, -pitch_deg, yaw_deg);
            rotation.push_back(rotation_vector);
            // Query point cloud transform from original frame
            double t_pcl_prev = ros::Time::now().toSec();
            query_pcl = base_to_transform_pcl(query_pcl, rotation_vector, point_vector[i]);
            printf("%s[lbfgs_pcl_constrain.h] Point %d base_to_transform_pcl, time %lf\n", 
                KCYN, i, ros::Time::now().toSec()-t_pcl_prev);


            double query_prev = ros::Time::now().toSec();
            query_single_point(point_vector[i], point_vector[i-1], rotation_vector);

            
            printf("%s[lbfgs_pcl_constrain.h] Point %d query_single_point, time %lf\n", 
                KCYN, i, ros::Time::now().toSec()-query_prev);
            printf("%s[lbfgs_pcl_constrain.h] Point %d Constrain, time %lf\n", 
                KCYN, i, ros::Time::now().toSec()-single_prev);
        }
        printf("%s[lbfgs_pcl_constrain.h] Total time %lf with flags %d\n", 
                KGRN, ros::Time::now().toSec()-total_prev, flag);
    }

    /* 
    * @brief We will update the bounding area over here, updaring max_constrain and min_constrain
    */
    void query_single_point(Vector3d p1, Vector3d p0, Vector3d rotation_vector)
    {
        double h, k ,m;
        // while (check_points_in_bb(query_pcl))
        // {
        // Let us find the new rotated coordinates centered around p1
        Vector3d np1 = base_to_transform_point(p1, rotation_vector, p1);
        Vector3d np0 = base_to_transform_point(p0, rotation_vector, p1);
        Vector3d new_vec = (np1 - np0);
        printf("%s[lbfgs_pcl_constrain.h] Vect %lf %lf %lf\n", 
                KYEL, new_vec.x(), new_vec.y(), new_vec.z());
        printf("%s[lbfgs_pcl_constrain.h] Vect1 %lf %lf %lf\n", 
                KYEL, np1.x(), np1.y(), np1.z());
        printf("%s[lbfgs_pcl_constrain.h] Vect0 %lf %lf %lf\n", 
                KYEL, np0.x(), np0.y(), np0.z());

        // Since everything is centered on np1, and x-axis aligned
        // We need to linspace to get points to query
        vector<Vector3d> search_vectors = linspace_vector3(np1, np0, division);

        Vector3d tmp_dimension;
        tmp_dimension.x() = abs(new_vec.x()) + 2 * max_boundaries;
        tmp_dimension.y() = abs(new_vec.y()) + 2 * max_boundaries;
        tmp_dimension.z() = abs(new_vec.z()) + 2 * max_boundaries;

        // Center of these 2 points = new_vec / 2;
        double filter1_prev = ros::Time::now().toSec(); 
        query_pcl = pcl_filter(query_pcl, new_vec / 2, tmp_dimension);
        printf("%s[lbfgs_pcl_constrain.h] Total time %lf for pcl filter1\n", 
                KGRN, ros::Time::now().toSec()-filter1_prev);
        
        size_t num_points = query_pcl->size();
        int total = static_cast<int>(num_points);

        double knn_prev = ros::Time::now().toSec();
        vector<double> safe_radius;
        
        // If there are any points inside 
        if (total > 0)
        {
            for (int i = 0; i < division; i++)
            {
                // search_vectors[i]
                double nearest_point = kdtree_pcl_knn(search_vectors[i], query_pcl);
                printf("%s[lbfgs_pcl_constrain.h] nearest_point %lf\n", KCYN, nearest_point);
                // nearest_point cannot be more than max_boundaries and cannot be smaller than safety radius
                double boundary = std::min(std::max(nearest_point - safety_radius, 0.1), corridor_size);
                safe_radius.push_back(boundary);
            }    
        }
        // If there are no points
        else
        {
            for (int i = 0; i < division; i++)
            {
                // nearest_point cannot be more than max_boundaries and also 
                double boundary = corridor_size;
                safe_radius.push_back(boundary);
            }

        }
        printf("%s[lbfgs_pcl_constrain.h] Total time %lf for nearest point query\n", 
                KGRN, ros::Time::now().toSec()-knn_prev);

        // Time to query pairs of circles using circle circle intersection
        // ** There may be an ERROR in the logic since Z is not aligned
        double half_h = corridor_size * 2; // High arbitrary number
        double cc_prev = ros::Time::now().toSec();

        for (int i = 1; i < division; i++)
        {
            double tmp = circle_circle_intersection(search_vectors[i], search_vectors[i-1], safe_radius[i], safe_radius[i-1]);
            printf("%s[lbfgs_pcl_constrain.h] tmp_h value %lf\n", KCYN, tmp);
            if (tmp < half_h)
                half_h = tmp;
        }
        printf("%s[lbfgs_pcl_constrain.h] Total time %lf for circle-circle intersection\n", 
                KGRN, ros::Time::now().toSec()-cc_prev);


        // h is the y axis boundary
        // k is the x axis boundary that would be p0 and p1 + 2*sin(45)*h
        // m is the z axis boundary that would be 2*sin(45)*h

        h = 2 * half_h;
        k = abs(new_vec.x()) + 2 * sin(45.0/180.0 * 3.1415) * half_h;
        m = 2 * sin(45.0/180.0 * 3.1415) * half_h;

        printf("%s[lbfgs_pcl_constrain.h] yh value %lf | %lf\n", KBLU, h, abs(new_vec.y()) + 2 * max_boundaries);
        printf("%s[lbfgs_pcl_constrain.h] xk value %lf | %lf\n", KBLU, k, abs(new_vec.x()) + 2 * max_boundaries);
        printf("%s[lbfgs_pcl_constrain.h] zm value %lf | %lf\n", KBLU, m, abs(new_vec.z()) + 2 * max_boundaries);

        // Center of these 2 points = new_vec / 2; 
        // new dimension are used for cropping 
        
        double filter2_prev = ros::Time::now().toSec();
        query_pcl = pcl_filter(query_pcl, new_vec / 2, Vector3d(k, h, m));
        printf("%s[lbfgs_pcl_constrain.h] Total time %lf for pcl filter2\n", 
                KGRN, ros::Time::now().toSec()-filter2_prev);

        double pcl_query_prev = ros::Time::now().toSec();
        if (check_points_in_bb(query_pcl))
        {
            printf("%s[lbfgs_pcl_constrain.h] Still have collision! \n", KRED);
            size_t num_points = query_pcl->size();
            int total = static_cast<int>(num_points);
            printf("%s[lbfgs_pcl_constrain.h] %d Points inside Bounding Box! \n", KRED, total);
            // return;
        }
        printf("%s[lbfgs_pcl_constrain.h] Total time %lf for pcl query\n", 
                KGRN, ros::Time::now().toSec()-pcl_query_prev);
        // }

        Vector3d tmp_max_constrain = Vector3d(k/2, h/2, m/2);
        Vector3d tmp_min_constrain = Vector3d(-k/2, -h/2, -m/2);
        max_constrain.push_back(tmp_max_constrain);
        min_constrain.push_back(tmp_min_constrain);
    }

    double circle_circle_intersection(Vector3d p1, Vector3d p0, double r1, double r0)
    {
        Vector3d difference = p1 - p0;
        printf("%s[lbfgs_pcl_constrain.h] r1 %lf r0 %lf\n", KBLU, r1, r0);

        double d = sqrt(pow(difference.x(),2) + pow(difference.y(),2));
        printf("%s[lbfgs_pcl_constrain.h] d %lf\n", KBLU, d);

        // Condition when one circle is contained in another
        if (d < abs(r0 - r1))
            return min(r0, r1);

        double a = (pow(r0,2) - pow(r1,2) + pow(d,2)) / (2 * d);
        printf("%s[lbfgs_pcl_constrain.h] a %lf\n", KBLU, a);

        double h = sqrt(pow(r0,2) - pow(a,2));

        if (isnan(abs(h)))
        {
            flag++;
            h = 0.1;
        }

        return h;
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

    Vector3d base_to_transform_point(Vector3d vec, Vector3d rotation, Vector3d translation)
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

        Eigen::Affine3d aff_r = Eigen::Affine3d::Identity();
        // aff_t.translation() = - translation;
        aff_r.translation() = rotatedV;
        aff_r.linear() = rot.toRotationMatrix();

        Vector3d test_value1 = aff_r * vec;

        // *** Remove multiple conversions that will take up more time ***
        // *** Replace with solely Eigen calculation ***
        // geometry_msgs::Point point = vector_to_point(vec);
        // // Translate then rotate to temporary frame for point clouds
        // geometry_msgs::Point tmp_transformed_point = transform_point(point, - Vector3d(0, 0, 0), - translation);
        // tmp_transformed_point = transform_point(tmp_transformed_point, - rotation, Vector3d(0, 0, 0));
    
        // Vector3d test_value2 = point_to_vector(tmp_transformed_point);
        // printf("%s[lbfgs_pcl_constrain.h] test_value1 %lf %lf %lf\n", KYEL, 
        //     test_value1.x(), test_value1.y(), test_value1.z());
        // printf("%s[lbfgs_pcl_constrain.h] test_value2 %lf %lf %lf\n", KYEL, 
        //     test_value2.x(), test_value2.y(), test_value2.z());

        return test_value1;
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

    double kdtree_pcl_knn(Vector3d point, pcl::PointCloud<pcl::PointXYZ>::Ptr _obs)
    {
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(_obs);

        pcl::PointXYZ searchPoint;
        searchPoint.x = point.x();
        searchPoint.y = point.y();
        searchPoint.z = point.z();

        // K nearest neighbor search

        int K = 1;

        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            // for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
            // std::cout << "    "  <<   (*_obs)[ pointIdxNKNSearch[i] ].x 
            //             << " " << (*_obs)[ pointIdxNKNSearch[i] ].y 
            //             << " " << (*_obs)[ pointIdxNKNSearch[i] ].z 
            //             << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
        }

        // We need to square root it
        return sqrt(pointNKNSquaredDistance[0]);
    }

    bool kdtree_pcl_filter(Vector3d point, pcl::PointCloud<pcl::PointXYZ>::Ptr _obs,
        double c)
    {
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

        kdtree.setInputCloud(_obs);

        pcl::PointXYZ searchPoint;
        searchPoint.x = point.x();
        searchPoint.y = point.y();
        searchPoint.z = point.z();

        // We will use seach neighbours within radius, this provides what we need
        // Neighbors within radius search

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        // float radius = 256.0f * rand () / (RAND_MAX + 1.0f);

        float radius = (float)c;

        // std::cout << "Neighbors within radius search at (" << searchPoint.x 
        //             << " " << searchPoint.y 
        //             << " " << searchPoint.z
        //             << ") with radius=" << radius << std::endl;


        if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
        {
            for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
            {
                // std::cout << "    "  <<   (*_obs)[ pointIdxRadiusSearch[i] ].x 
                //             << " " << (*_obs)[ pointIdxRadiusSearch[i] ].y 
                //             << " " << (*_obs)[ pointIdxRadiusSearch[i] ].z 
                //             << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
                return true;
            }
        }

        return false;
    }

    /* 
    * @brief Check if there are any points in the boundary box
    */
    bool check_points_in_bb(pcl::PointCloud<pcl::PointXYZ>::Ptr pc)
    {
        if (pc->points.size())
            true;
        else
            false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr 
        pcl2_converter(sensor_msgs::PointCloud2 _pc)
    {
        pcl::PCLPointCloud2 pcl_pc2;
        // printf("%s[rrtstar.h] ros_pcl2 to pcl! \n", KBLU);
        pcl_conversions::toPCL(_pc, pcl_pc2);

        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        // printf("%s[rrtstar.h] fromPCLPointCloud2! \n", KBLU);
        pcl::fromPCLPointCloud2(pcl_pc2, *tmp_cloud);
        
        // printf("%s[rrtstar.h] return fromPCLPointCloud2! \n", KBLU);
        return tmp_cloud;
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

    vector<Vector3d> linspace_vector3(Vector3d min, Vector3d max, int n)
    {
        vector<double> linspaced[3];
        for (int j = 0; j < 3; j++)
        {
            double delta = (max[j] - min[j]) / (n - 1.0);
            linspaced[j].push_back(min[j]);
            
            for (int i = 1; i < n; i++)
            {
                linspaced[j].push_back(linspaced[j][i-1] + delta);
            }
        }

        vector<Vector3d> output;
        for (int i = 0; i < n; i++)
        {
            Vector3d vec;
            for (int j = 0; j < 3; j++)
            {
                vec[j] = linspaced[j][i];
            }
            output.push_back(vec);
        }
        return output;
    }
};
