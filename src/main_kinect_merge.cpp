/*********************************************************************
 *
 *  Copyright (c) 2014, Jeannette Bohg - MPI for Intelligent System
 *  (jbohg@tuebingen.mpg.de)
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
 *   * Neither the name of Jeannette Bohg nor the names of MPI
 *     may be used to endorse or promote products derived
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

/* main_kinect.cpp 
 * Test program that sets up simulator with specific camera parameters 
 * and object mesh. A number of object poses is sampled from which 
 * a desired measured output (depthmap, label image, point cloud) is 
 * generated and stored.
 */

#include <render_kinect/simulate.h>
#include <render_kinect/camera.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

void getOrbitTransforms(
    std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d>> & p_tfs, 
    const int & view_num = 10, 
    const double & distance = 1.0)
{
    p_tfs.resize(view_num);
    for (int i = 0; i < view_num; i++)
    {
        double pitch = (double)(i) / view_num * 2 * M_PI;
        p_tfs[i] = Eigen::Affine3d::Identity();
        p_tfs[i].translate(Eigen::Vector3d::UnitZ() * distance);
        p_tfs[i].rotate(Eigen::AngleAxis<double>(pitch, Eigen::Vector3d::UnitX()));
    }
}

// main function that generated a number of sample outputs for a given object mesh.
int main(int argc, char **argv)
{

    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " model_file.obj" << std::endl;
        exit(-1);
    }

    // Get the path to the object mesh model.
    std::string object_models_dir = "../obj_models/";
    std::stringstream full_path;
    full_path << object_models_dir << argv[1];

    // Get the path to the dot pattern
    std::string dot_path = "../data/kinect-pattern_3x3.png";

    // Camera Parameters
    render_kinect::CameraInfo cam_info;

    cam_info.width = 640;
    cam_info.height = 480;
    cam_info.cx_ = 320;
    cam_info.cy_ = 240;

    cam_info.z_near = 0.5;
    cam_info.z_far = 6.0;
    cam_info.fx_ = 580.0;
    cam_info.fy_ = 580.0;
    // baseline between IR projector and IR camera
    cam_info.tx_ = 0.075;

    // Type of noise
    //  cam_info.noise_ = render_kinect::GAUSSIAN;
    //  cam_info.noise_ = render_kinect::PERLIN;
    cam_info.noise_ = render_kinect::NONE;

    // Kinect Simulator
    render_kinect::Simulate Simulator(cam_info, full_path.str(), dot_path);

    // Number of samples
    int frames = 10;

    // sample a group of random transformations on a ball
    std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d>> p_tfs;
    getOrbitTransforms(p_tfs, frames, 1.0);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (int i = 0; i < frames; ++i)
    {
        // give pose and object name to renderer
        Eigen::Affine3d current_tf = p_tfs[i];
        Simulator.simulateMeasurement(current_tf);

        // store measurement result
        std::stringstream lD;
        lD << object_models_dir << "depth_orig" << std::setw(3) << std::setfill('0') << i << ".png";
        Simulator.store_depth(lD.str());

        pcl::PointCloud<pcl::PointXYZ> cloud_world;
        pcl::transformPointCloud(Simulator.point_cloud(),cloud_world,current_tf.inverse());

        cloud += cloud_world;
    }

    std::stringstream lD;
    lD << object_models_dir << "point_cloud" << ".ply";
    if (pcl::io::savePLYFileBinary(lD.str(), cloud) != 0)
      std::cout << "Couldn't store point cloud at " << lD.str() << std::endl;

    return 0;
}
