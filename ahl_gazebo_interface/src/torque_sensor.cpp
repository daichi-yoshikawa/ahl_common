#include "ahl_gazebo_interface/torque_sensor.hpp"
#include "ahl_gazebo_interface/exception.hpp"

using namespace ahl_gazebo_if;

TorqueSensor::TorqueSensor()
{
  ros::NodeHandle nh;
  client_add_torque_sensor_ = nh.serviceClient<gazebo_msgs::AddTorqueSensor>(
    "/gazebo/add_torque_sensor");
}

void TorqueSensor::add(const std::string& joint_name)
{
  std::string sensor_name = joint_name + "::torque_sensor";

  if(sensor_to_idx_.find(sensor_name) == sensor_to_idx_.end())
  {
    unsigned int size = sensor_to_idx_.size();
    sensor_to_idx_[sensor_name] = size;
    sensor_list_.push_back(sensor_name);

    state_[sensor_name] = std::make_shared<ahl_utils::SharedMemory<double>>(sensor_name);

    gazebo_msgs::AddTorqueSensor srv;
    srv.request.joint_name  = joint_name;
    srv.request.sensor_name = sensor_name;

    if(!client_add_torque_sensor_.call(srv))
    {
      std::stringstream msg;
      msg << "Could not add torque sensor : " << sensor_name;
      throw ahl_gazebo_if::Exception("TorqueSensor::add", msg.str());
    }
  }
}

void TorqueSensor::connect()
{
  sensor_num_ = sensor_list_.size();
  tau_ = Eigen::VectorXd::Zero(sensor_num_);
}

const Eigen::VectorXd& TorqueSensor::getJointTorque()
{
  for(unsigned int i = 0; i < sensor_num_; ++i)
  {
    if(sensor_to_idx_.find(sensor_list_[i]) == sensor_to_idx_.end())
    {
      std::stringstream msg;
      msg << sensor_list_[i] << " was not found in sensor_to_idx_.";
      throw ahl_gazebo_if::Exception("TorqueSensor::getSensedTorque", msg.str());
    }

    state_[sensor_list_[i]]->read(tau_[i]);
  }

  return tau_;
}
