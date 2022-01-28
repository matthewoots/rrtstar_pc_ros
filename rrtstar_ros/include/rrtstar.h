/*
 * rrtstar.h
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

/*
* With help from 
* https://github.com/swadhagupta/RRT/blob/master/rrt.cpp
* https://pcl.readthedocs.io/projects/tutorials/en/latest/kdtree_search.html
*/
#include "common_utils.h"

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

common_utility _common_utils;

class rrtstar
{
    private:

    struct Node 
    {
        vector<Node *> children;
        Node *parent;
        Vector3d position;
    };

    Node start_node;
    Node end_node;
    Node* nodes[5000]; // ?? need to clean this up
    pcl::PointCloud<pcl::PointXYZ>::Ptr obs;

    bool reached;
    double obs_threshold;
    int step_size, iter, total_nodes;
    int line_search_division;
    double _min_height, _max_height;

    Vector3d map_size = Vector3d::Zero();
    Vector3d origin = Vector3d::Zero();    

    Vector3d get_pc_pose(int idx)
    {
        Vector3d p = Vector3d(obs->points[idx].x,
            obs->points[idx].y,
            obs->points[idx].z);
        
        return p;
    }

    void rrt()
    {
        double prev =ros::Time::now().toSec();
        int index = 0;

        Node* random_node = new Node;
        Node* step_node = new Node;
    
        (random_node->position).x() = (double)(rand() % 
            (int)round(map_size.x()) * 100) / 
            100.0 - map_size.x()/2 + origin.x() + 1;
        (random_node->position).y() = (double)(rand() % 
            (int)round(map_size.y()) * 100) / 
            100.0 - map_size.y()/2 + origin.y() + 1;

        // Constrain height within the min-max range
        double tmp = (double)(rand() % (int)round(map_size.z()) * 100) / 100.0  - map_size.z()/2 + origin.z() + 1;
        tmp = max(tmp, _min_height);
        tmp = min(tmp, _max_height);
        (random_node->position).z() = tmp;

        index = near_node(*random_node);
        
        if((separation(random_node->position, nodes[index]->position)) < step_size)
            return;
        else
            step_node->position = stepping(nodes[index]->position, random_node->position);
        
        bool flag = check_validity(nodes[index]->position, step_node->position);

        if(flag)
        {
            nodes[total_nodes++] = step_node;
            step_node->parent = nodes[index];
            (nodes[index]->children).push_back(step_node);
            // line(img, Point((stepnode->position).y, (stepnode->position).x), Point(nodes[index]->position.y, nodes[index]->position.x), Scalar(0, 255, 255), 2, 8);
            // for(int i= step_node->position.x() - 2; 
            //     i < step_node->position.x() + 2; i++)
            // {
                // for(int j=step_node->position.y() - 2; j < stepnode->position.y + 2; j++)
                // {
                //     if((i<0) || (i>400) || (j<0) || (j>400))
                //         continue;

                //     img.at<Vec3b>(i, j)[0] = 0;
                //     img.at<Vec3b>(i, j)[1] = 255;
                //     img.at<Vec3b>(i, j)[2] = 0;
                // }
            // }
            if((check_validity(step_node->position, end_node.position)) && 
                (separation(step_node->position,end_node.position) < step_size))
            {
                printf("%sReached path!\n", KGRN);
                reached = true;
                nodes[total_nodes++] = &end_node;
                end_node.parent = step_node;
                (nodes[total_nodes-1]->children).push_back(&end_node);
                // draw_path();
            }
        }
        iter++;

        double final_secs =ros::Time::now().toSec() - prev;
        printf("%squery (%s%.2lf %.2lf %.2lf%s) time-taken (%s%.4lf%s)\n", 
            KBLU, KNRM,
            random_node->position.x(),
            random_node->position.y(),
            random_node->position.z(),
            KBLU, KNRM, final_secs, KBLU);
    }

    // [near_node] is responsible for finding the nearest node in the tree 
    // for a particular random node. 
    int near_node(Node random)
    {
        double min_dist = dmax;
        // We give dist a default value if total node is 1 it will fall back on this
        double dist = separation(start_node.position, random.position);
        
        int linking_node = 0;

        for(int i = 0; i < total_nodes; i++)
        {
            // Other nodes than start node
            dist = separation(nodes[i]->position, random.position);
            // Evaluate distance
            if(dist < min_dist)
            {
                min_dist = dist;
                linking_node = i;
            }
        }
        return linking_node;
    }

    // [separation] function takes two coordinates
    // as its input and returns the distance between them.
    double separation(Vector3d p, Vector3d q)
    {
        Vector3d v;
        v.x() = p.x() - q.x();
        v.y() = p.y() - q.y();
        v.z() = p.z() - q.z();
        return sqrt(pow(v.x(), 2) + pow(v.y(), 2) + pow(v.z(), 2));
    }

    // [stepping] function takes the random node generated and its nearest node in the tree 
    // as its input, and returns the coordinates of the step node. 
    // This function determines the step node by generating a new node at a distance 
    // of step_size from nearest node towards random node
    Vector3d stepping(Vector3d nearest_node, Vector3d random_node)
    {
        Vector3d tmp = Vector3d::Zero();
        Vector3d step = Vector3d::Zero();
        Vector3d norm = Vector3d::Zero();
        double magnitude = 0.0;
        double x, y, z;

        tmp.x() = random_node.x() - nearest_node.x();
        tmp.y() = random_node.y() - nearest_node.y();
        tmp.z() = random_node.z() - nearest_node.z();
        magnitude = sqrt(pow(tmp.x(),2) + 
            pow(tmp.y(),2) +
            pow(tmp.z(),2));

        norm.x() = (tmp.x() / magnitude);
        norm.y() = (tmp.y() / magnitude);
        norm.z() = (tmp.z() / magnitude);

        step.x() = nearest_node.x() + step_size * norm.x();
        step.y() = nearest_node.y() + step_size * norm.y();
        step.z() = nearest_node.z() + step_size * norm.z();

        return step;
    }

    // They check if the step node that has been generated is valid or not
    // It will be invalid if the step node either lies in the proximity of a pointcloud (that is, an obstacle) 
    // or if the straight line path joining nearest_node and step_node goes through an obstacle
    bool check_validity(Vector3d p, Vector3d q)
    {
        Vector3d large, small;
        int i = 0, j1 = 0, j2 = 0;
        int n = line_search_division;
        MatrixXd line_vector;
        line_vector = MatrixXd::Zero(3, n);
        for (int i = 0; i < 3; i++)
        {
            line_vector.row(i) = _common_utils.linspace(p[i], q[i], (double)n);
        }
        int column_size = line_vector.cols();

        for(int i = 0; i < column_size; i++)
        {
            // Check to see whether the point in the line 
            // collides with an obstacle
            // if (obstacle_map_filter(line_vector.col(i)))
            //     return false;
            if (kdtree_pcl(line_vector.col(i), obs, obs_threshold))
                return false;
            // Check to see whether the line is within the boundaries
            // Don't think we need this check
            // if((i<0) || (i>400) || (j1<0) || (j1>400) || (j2<0) || (j2>400))
            //     continue;
        }
        return true;
    }

    // Terrible brute force method that very upsettingly have poor performance
    bool obstacle_map_filter(Vector3d point)
    {
        // Load obstacle map
        size_t num_points = obs->size();
        int total = static_cast<int>(num_points);

        for (int i = 0; i < total; i++)
        {
            double tmp = separation(point, 
                Vector3d(obs->points[i].x, obs->points[i].y, obs->points[i].z));
            if (tmp < obs_threshold)
                return false;
        }
        return true;    
    }

    public:

    // Used for pcl2 and adapted from 
    // https://pcl.readthedocs.io/projects/tutorials/en/latest/kdtree_search.html
    bool kdtree_pcl(Vector3d point, sensor_msgs::PointCloud2 _pc, double c)
    {
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

        pcl::PointCloud<pcl::PointXYZ>::Ptr _obs = _common_utils.pcl2_converter(_pc);
        kdtree.setInputCloud(_obs);

        pcl::PointXYZ searchPoint;
        searchPoint.x = point.x();
        searchPoint.y = point.y();
        searchPoint.z = point.z();

        // K nearest neighbor search

        // int K = 5;

        // std::vector<int> pointIdxNKNSearch(K);
        // std::vector<float> pointNKNSquaredDistance(K);

        // std::cout << KGRN << "K nearest neighbor search at (" << searchPoint.x 
        //             << " " << searchPoint.y 
        //             << " " << searchPoint.z
        //             << ") with K=" << K << std::endl;

        // if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        // {
        //     for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
        //     std::cout << "    "  <<   (*_obs)[ pointIdxNKNSearch[i] ].x 
        //                 << " " << (*_obs)[ pointIdxNKNSearch[i] ].y 
        //                 << " " << (*_obs)[ pointIdxNKNSearch[i] ].z 
        //                 << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
        // }

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

    bool kdtree_pcl(Vector3d point, pcl::PointCloud<pcl::PointXYZ>::Ptr _obs,
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

    // void initialize(Vector3d _start, 
    //     Vector3d _end, 
    //     sensor_msgs::PointCloud2 _pc,
    //     Vector3d _map_size,
    //     Vector3d _origin,
    //     double _step_size,
    //     double _obs_threshold,
    //     int _line_search_division)
    // {
    //     // printf("%s[rrtstar.h] Key Variables! \n", KBLU);
    //     start_node.position = _start;
    //     start_node.parent = NULL;
    //     total_nodes = 0;

    //     nodes[total_nodes++] = &start_node;
    //     end_node.position = _end;

    //     // printf("%s[rrtstar.h] PCL conversion! \n", KBLU);
    //     obs = _common_utils.pcl2_converter(_pc);
    //     obs_threshold = _obs_threshold;

    //     // printf("%s[rrtstar.h] Map parameters! \n", KBLU);
    //     map_size = _map_size;
    //     origin = _origin;
    //     step_size = _step_size;
    //     iter = 0;
    //     line_search_division = _line_search_division;
    //     printf("%s[rrtstar.h] Initialized! \n", KBLU);
    //     srand(time(NULL));
    // }

    void initialize(Vector3d _start, 
        Vector3d _end, 
        pcl::PointCloud<pcl::PointXYZ>::Ptr _pc,
        Vector3d _map_size,
        Vector3d _origin,
        double _step_size,
        double _obs_threshold,
        double min_height,
        double max_height,
        int _line_search_division)
    {
        // printf("%s[rrtstar.h] Key Variables! \n", KBLU);
        start_node.position = _start;
        start_node.parent = NULL;
        total_nodes = 0;

        nodes[total_nodes++] = &start_node;
        end_node.position = _end;

        // printf("%s[rrtstar.h] PCL conversion! \n", KBLU);
        // obs(new pcl::PointCloud<pcl::PointXYZ>);
        obs = _pc;
        obs_threshold = _obs_threshold;

        _min_height = min_height;
        _max_height = max_height;

        // printf("%s[rrtstar.h] Map parameters! \n", KBLU);
        map_size = _map_size;
        origin = _origin;
        step_size = _step_size;
        iter = 0;
        line_search_division = _line_search_division;
        printf("%s[rrtstar.h] Initialized! \n", KBLU);
        srand(time(NULL));
    }

    void run()
    {
        size_t num_points = obs->size();
        int total = static_cast<int>(num_points);
        double prev = ros::Time::now().toSec();
        printf("%s[rrtstar.h] Obstacle size %d! \n", KGRN, total);
        printf("%s[rrtstar.h] Start run process! \n", KGRN);
        while(!reached)
            rrt();
        printf("%sSolution found! with %d iter and %d nodes\n", 
            KGRN, iter, total_nodes);
        printf("%sTotal Time Taken = %lf!\n", KGRN, ros::Time::now().toSec() - prev);
    }

    std::vector<Vector3d> path_extraction()
    {
        Node up, down;
        int breaking = 0;
        down = end_node;
        up = *(end_node.parent);
        std::vector<Vector3d> path;
        int i = 0;
        while(1)
        {
            path.push_back(down.position);
            if(up.parent == NULL)
                break;
            up = *(up.parent);
            down = *(down.parent);
            i++;
        }

        return path;
    }
};
