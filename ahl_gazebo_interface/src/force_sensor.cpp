/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Daichi Yoshikawa
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
 *   * Neither the name of the Daichi Yoshikawa nor the names of its
 *     contributors may be used to endorse or promote products derived
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
 *
 * Author: Daichi Yoshikawa
 *
 *********************************************************************/

#include "ahl_gazebo_interface/force_sensor.hpp"
#include "ahl_gazebo_interface/exception.hpp"

using namespace ahl_gazebo_if;

ForceSensor::ForceSensor()
{
  ros::NodeHandle nh;
  client_add_force_sensor_ = nh.serviceClient<gazebo_msgs::AddForceSensor>(
    "/gazebo/add_force_sensor");
}

void ForceSensor::add(const std::string& joint_name)
{
  std::string sensor_name = joint_name + "::force_sensor";

  if(sensor_to_idx_.find(sensor_name) == sensor_to_idx_.end())
  {
    unsigned int size = sensor_to_idx_.size();
    sensor_to_idx_[sensor_name] = size;
    sensor_list_.push_back(sensor_name);

    fx_[sensor_name + "::fx"] = std::make_shared<ahl_utils::SharedMemory<double>>(sensor_name + "::fx");
    fy_[sensor_name + "::fy"] = std::make_shared<ahl_utils::SharedMemory<double>>(sensor_name + "::fy");
    fz_[sensor_name + "::fz"] = std::make_shared<ahl_utils::SharedMemory<double>>(sensor_name + "::fz");
    mx_[sensor_name + "::mx"] = std::make_shared<ahl_utils::SharedMemory<double>>(sensor_name + "::mx");
    my_[sensor_name + "::my"] = std::make_shared<ahl_utils::SharedMemory<double>>(sensor_name + "::my");
    mz_[sensor_name + "::mz"] = std::make_shared<ahl_utils::SharedMemory<double>>(sensor_name + "::mz");

    gazebo_msgs::AddForceSensor srv;
    srv.request.joint_name  = joint_name;
    srv.request.sensor_name = sensor_name;

    if(!client_add_force_sensor_.call(srv))
    {
      std::stringstream msg;
      msg << "Could not add force sensor : " << sensor_name;
      throw ahl_gazebo_if::Exception("ForceSensor::add", msg.str());
    }
  }
}

void ForceSensor::connect()
{
  sensor_num_ = sensor_list_.size();

  for(unsigned int i = 0; i < sensor_num_; ++i)
  {
    f_[sensor_list_[i]] = Eigen::VectorXd::Zero(6); // fxfyfz, mxmymz
  }
}

const Eigen::VectorXd& ForceSensor::getExternalForce(const std::string& joint_name)
{
  std::string sensor_name = joint_name + "::force_sensor";

  if(sensor_to_idx_.find(sensor_name) == sensor_to_idx_.end())
  {
    std::stringstream msg;
    msg << joint_name << " was not found in sensor_to_idx_.";
    throw ahl_gazebo_if::Exception("ForceSensor::getExternalForce", msg.str());
  }

  int idx = sensor_to_idx_[sensor_name];

  fx_[sensor_name + "::fx"]->read(f_[sensor_name][0]);
  fy_[sensor_name + "::fy"]->read(f_[sensor_name][1]);
  fz_[sensor_name + "::fz"]->read(f_[sensor_name][2]);
  mx_[sensor_name + "::mx"]->read(f_[sensor_name][3]);
  my_[sensor_name + "::my"]->read(f_[sensor_name][4]);
  mz_[sensor_name + "::mz"]->read(f_[sensor_name][5]);

  return f_[sensor_name];
}

const std::map<std::string, Eigen::VectorXd>& ForceSensor::getExternalForce()
{
  for(unsigned int i = 0; i < sensor_list_.size(); ++i)
  {
    if(f_.find(sensor_list_[i]) == f_.end())
    {
      std::stringstream msg;
      msg << sensor_list_[i] << " was not found in f_.";
      throw ahl_gazebo_if::Exception("ForceSensor::getExternalForce", msg.str());
    }

    f_[sensor_list_[i]] = this->getExternalForce(sensor_list_[i]);
  }

  return f_;
}
