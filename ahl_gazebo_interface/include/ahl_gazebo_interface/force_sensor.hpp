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

#ifndef __AHL_GAZEBO_INTERFACE_FORCE_SENSOR_HPP
#define __AHL_GAZEBO_INTERFACE_FORCE_SENSOR_HPP

#include <string>
#include <map>
#include <memory>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <gazebo_msgs/AddForceSensor.h>
#include <ahl_utils/shared_memory.hpp>

namespace ahl_gazebo_if
{

  class ForceSensor
  {
  public:
    explicit ForceSensor();

    void add(const std::string& joint_name);
    void connect();

    const Eigen::VectorXd& getExternalForce(const std::string& joint_name);
    const std::map<std::string, Eigen::VectorXd>& getExternalForce();

  private:
    std::map<std::string, int32_t> sensor_to_idx_;
    std::vector<std::string> sensor_list_;
    uint32_t sensor_num_;

    std::map<std::string, Eigen::VectorXd> f_;
    std::map<std::string, ahl_utils::SharedMemoryPtr<double>> fx_;
    std::map<std::string, ahl_utils::SharedMemoryPtr<double>> fy_;
    std::map<std::string, ahl_utils::SharedMemoryPtr<double>> fz_;
    std::map<std::string, ahl_utils::SharedMemoryPtr<double>> mx_;
    std::map<std::string, ahl_utils::SharedMemoryPtr<double>> my_;
    std::map<std::string, ahl_utils::SharedMemoryPtr<double>> mz_;

    ros::ServiceClient client_add_force_sensor_;
  };

  using ForceSensorPtr = std::shared_ptr<ForceSensor>;

} // namespace ahl_gazebo_if

#endif // __AHL_GAZEBO_INTERFACE_FORCE_SENSOR_HPP
