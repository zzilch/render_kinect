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

/* move_kinect.cpp 
 * Program publishes simulated observations from an object moving with
 * brownian motion.
 */

#include <ros/package.h>

#include <boost/filesystem.hpp>


#include <render_kinect/simulate.h>
#include <render_kinect/camera.h>
#include <render_kinect/robot_state.h>

#include <pose_tracking_interface/utils/robot_tracking_dataset.hpp>

// Mostly reading tools
#include <render_kinect/tools/rosread_utils.h>

#include <math.h>

#include <ros/time.h>


using namespace std;
using namespace Eigen;

namespace render_kinect
{
  class RobotDataGenerator
  {
  public:
    RobotDataGenerator() :
      nh_priv_("~") ,
      noisy_(false) ,
      frame_count_(0)
    {
      // get the path to the package
      std::string path = ros::package::getPath("render_kinect");
      
      // the path of our dataset will be the same as the config file
      string config_file; nh_priv_.getParam("config_file", config_file);
      boost::filesystem::path path_dataset = config_file;
      path_dataset = path_dataset.parent_path();

      dataset_ = new RobotTrackingDataset(path_dataset.string());
      nh_priv_.getParam("frame_count", max_frame_count_);

      // get the relevant parameters
      nh_priv_.param<bool>("noisy", noisy_, false);
      std::string joint_states_topic;
      nh_priv_.param<std::string>("joint_states_topic", joint_states_topic, "/joint_states");
      
      // Get the path to the dot pattern
      std::string dot_pattern_path;
      rosread_utils::getDotPatternPath(path, dot_pattern_path);
      
      // Get the path of the room (background mesh)
      std::string room_path;
      rosread_utils::getRoomPath(path, room_path);
      // Get a default room transform
      Eigen::Affine3d room_tf;
      rosread_utils::getRoomTransform(nh_priv_, path, room_tf);
      
      // Get the camera info
      render_kinect::CameraInfo cam_info;
      rosread_utils::getCameraInfo(nh_priv_, cam_info);
      
      // Create robot state that is responsible for converting joint angles to tfs
      robot_state_ = new RobotState;
      
      // Get the paths to the part_mesh_models
      std::vector<std::string> part_mesh_paths;
      std::vector<Eigen::Affine3d> part_mesh_transforms;
      robot_state_->GetPartMeshData(part_mesh_paths, part_mesh_transforms);
      
      // Initialize the kinect simulator
      simulator_ = new Simulate(cam_info, 
				part_mesh_paths, 
				dot_pattern_path, 
				rosread_utils::renderBackground(nh_priv_),
				room_path,
				room_tf);

      // set the original/default transform of the robot parts (otherwise they are set to Identity)
      simulator_->setOriginalTransform(part_mesh_transforms);

      // subscribe to joint states 
      joint_states_sub_ = nh_priv_.subscribe<sensor_msgs::JointState>(joint_states_topic, 
								      1,
								      &RobotDataGenerator::jointStateCallback, 
								      this);

      std::cout << "max frame count: " << max_frame_count_ << " rendering frames: " << std::endl;
    }

    ~RobotDataGenerator()
    {
      delete robot_state_;
      delete simulator_;
      delete dataset_;
    }

  private:
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
      ros::Time jnt_stamp = msg->header.stamp;
      joint_state_ = *msg;
      robot_state_->GetTransforms(joint_state_, current_tfs_);

      Eigen::VectorXd ground_truth_jnts;
      robot_state_->GetJointVector(joint_state_, ground_truth_jnts);

      sensor_msgs::JointStatePtr noisy_jnt_state;
      Eigen::VectorXd noisy_jnts;
      robot_state_->GetNoisyJointVector(joint_state_, noisy_jnt_state, noisy_jnts);

      // compute room orientation using the robot base
      Eigen::Affine3d room_tf;
      if(!robot_state_->GetRoomTransform(room_tf))
	ROS_ERROR("Using camera frame transform before KDL data has been initialized");
      // give it to the kinect simulator for rendering
      simulator_->setRoomTransform(room_tf);

      sensor_msgs::ImagePtr image;
      sensor_msgs::CameraInfoPtr camera_info;
      simulator_->simulateMeasurement(current_tfs_, image, camera_info);
      
      if(frame_count_ < max_frame_count_ && ros::ok()) {
	dataset_->AddFrame(image, camera_info, msg, noisy_jnt_state, ground_truth_jnts, noisy_jnts);
	cout << frame_count_ << ", " << flush;
      }      

      frame_count_++;

      if(dataset_->Size() == max_frame_count_) 
	{
	  dataset_->Store();
	  joint_states_sub_.shutdown();
	  ROS_INFO("Stored the dataset and stopped recording more data.");
	}
    }

    ros::NodeHandle nh_priv_;
    ros::Subscriber joint_states_sub_;
    
    RobotState *robot_state_;
    Simulate *simulator_;

    RobotTrackingDataset *dataset_;
    int max_frame_count_;
    int frame_count_;

    sensor_msgs::JointState joint_state_;

    std::vector<Eigen::Affine3d> current_tfs_;

    bool noisy_;

  }; // class RobotDataGenerator
} // namespace render_kinect


// main function that starts ros node that subscribes to joint_states topic
int main(int argc, char **argv)
{
  // initialize ros
  ros::init(argc, argv, "record_robot");
  
  render_kinect::RobotDataGenerator robot_data_generator;
  ros::spin();

  return 0;
}