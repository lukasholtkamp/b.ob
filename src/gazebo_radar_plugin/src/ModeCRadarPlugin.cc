/*
 * Copyright 2016-2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "gazebo_radar_plugin/ModeCRadarPlugin.hh"

#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/sensors/Noise.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorManager.hh>

#include <algorithm>
#include <sstream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "gazebo_radar_plugin/msg/mode_c_radar_summary.hpp"
#include "gazebo_radar_plugin/msg/mode_c_radar.hpp"

using namespace gazebo;

static const uint16_t DEFAULT_TRANSPONDER_CODE = 1200;

GZ_REGISTER_MODEL_PLUGIN(ModeCRadarPlugin)

std::string TrimNamespace(const std::string &modelName)
{
  // Trim namespaces
  size_t index = modelName.find_last_of('|');
  return modelName.substr(index + 1);
}

/// \brief Determine the type of a gazebo model from its name
std::string DetermineModelType(const std::string &modelName)
{
  std::string modelType(TrimNamespace(modelName));

  // Trim trailing underscore and number caused by inserting multiple of the same model
  size_t index = modelType.find_last_not_of("0123456789");
  if (modelType[index] == '_' && index > 1)
  {
    modelType = modelType.substr(0, index);
  }

  // Trim "_clone" suffix if exists
  index = modelType.rfind("_clone");
  if (index != std::string::npos)
  {
    modelType.erase(index);
  }

  return modelType;
}

uint16_t GetTransponderCode(const std::string &modelName)
{
  std::string reduced_str;
  for (char c : modelName)
  {
    if (isdigit(c))
    {
      reduced_str.push_back(c);
    }
  }
  if (!reduced_str.empty())
  {
    return std::stoi(reduced_str);
  }
  else
  {
    return DEFAULT_TRANSPONDER_CODE;
  }
}

/////////////////////////////////////////////////
ModeCRadarPlugin::ModeCRadarPlugin()
{
}

/////////////////////////////////////////////////
ModeCRadarPlugin::~ModeCRadarPlugin()
{
}

/////////////////////////////////////////////////
void ModeCRadarPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Load parameters
  this->robotNamespace = "logical_camera";
  if (_sdf->HasElement("robotNamespace"))
  {
    this->robotNamespace = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  }

  this->world = _parent->GetWorld();

  // Initialize ROS 2 node
  if (!rclcpp::ok())
  {
    RCLCPP_FATAL(this->rosnode->get_logger(),
                 "A ROS node for Gazebo has not been initialized, unable to load plugin. Load the Gazebo system plugin "
                 "'libgazebo_ros_api_plugin.so' in the gazebo_ros package");
    return;
  }

  this->rosnode = rclcpp::Node::make_shared(this->robotNamespace);

  this->onlyPublishKnownModels = false;
  if (_sdf->HasElement("known_model_types"))
  {
    RCLCPP_DEBUG(this->rosnode->get_logger(), "Only publishing known model types");
    this->onlyPublishKnownModels = true;
    this->knownModelTypes.clear();
    sdf::ElementPtr knownModelTypesElem = _sdf->GetElement("known_model_types");
    if (!knownModelTypesElem->HasElement("type"))
    {
      gzerr << "Unable to find <type> elements in the <known_model_types> section\n";
      return;
    }
    sdf::ElementPtr knownModelTypeElem = knownModelTypesElem->GetElement("type");
    while (knownModelTypeElem)
    {
      std::string type = knownModelTypeElem->Get<std::string>();
      this->knownModelTypes.push_back(type);
      knownModelTypeElem = knownModelTypeElem->GetNextElement("type");
    }
  }
  else
  {
    RCLCPP_DEBUG(this->rosnode->get_logger(), "Publishing all model types");
  }

  this->model = _parent;
  this->FindLogicalCamera();
  if (!this->sensor)
  {
    gzerr << "No logical camera found on any link\n";
    return;
  }

  // Handle noise model settings.
  if (_sdf->HasElement("position_noise"))
  {
    this->noiseModels["POSITION_NOISE"] =
        sensors::NoiseFactory::NewNoiseModel(_sdf->GetElement("position_noise")->GetElement("noise"),
                                             "logical_camera");
  }

  this->name = _parent->GetName();
  std::string radarTopic_ros = this->name + "_radar";
  if (_sdf->HasElement("radar_topic_ros"))
  {
    radarTopic_ros = _sdf->Get<std::string>("radar_topic_ros");
  }

  this->radar_sensor_frameid = this->name + "_radar";
  if (_sdf->HasElement("radar_sensor_frameid"))
  {
    _sdf->GetElement("radar_sensor_frameid")->GetValue()->Get(this->radar_sensor_frameid);
  }

  this->max_range = 10000;
  if (_sdf->HasElement("max_range"))
  {
    _sdf->GetElement("max_range")->GetValue()->Get(this->max_range);
  }

  this->hfov = 2 * M_PI;
  if (_sdf->HasElement("hfov"))
  {
    _sdf->GetElement("hfov")->GetValue()->Get(this->hfov);
  }

  this->vfov = M_PI;
  if (_sdf->HasElement("vfov"))
  {
    _sdf->GetElement("vfov")->GetValue()->Get(this->vfov);
  }

  // Initialize Gazebo transport node
  this->gzNode = transport::NodePtr(new transport::Node());
  this->gzNode->Init();

  this->imageSub = this->gzNode->Subscribe(
      this->sensor->Topic(),
      &ModeCRadarPlugin::HandleImage, // Bind directly to the member function
      this);

  RCLCPP_INFO(this->rosnode->get_logger(), "Subscribing to gazebo topic: %s", this->sensor->Topic().c_str());

  this->radarPub = this->rosnode->create_publisher<gazebo_radar_plugin::msg::ModeCRadarSummary>(radarTopic_ros, 10);

  RCLCPP_INFO(this->rosnode->get_logger(), "Publishing to ROS topic: %s", radarTopic_ros.c_str());
}

void ModeCRadarPlugin::FindLogicalCamera()
{
  sensors::SensorManager *sensorManager = sensors::SensorManager::Instance();

  // Go through each link's sensors until a logical camera is found
  for (physics::LinkPtr link : this->model->GetLinks())
  {
    for (unsigned int i = 0; i < link->GetSensorCount(); ++i)
    {
      sensors::SensorPtr sensor = sensorManager->GetSensor(link->GetSensorName(i));
      if (sensor->Type() == "logical_camera")
      {
        this->sensor = sensor;
        break;
      }
    }
    if (this->sensor)
    {
      this->cameraLink = link;
      break;
    }
  }
}

void ModeCRadarPlugin::HandleImage(const boost::shared_ptr<const gazebo::msgs::LogicalCameraImage> &_msg)
{
  // Handle the image message
  this->OnImage(*_msg); // Dereference the shared_ptr here
}

/////////////////////////////////////////////////
void ModeCRadarPlugin::OnImage(const gazebo::msgs::LogicalCameraImage &_msg)
{
  gazebo_radar_plugin::msg::ModeCRadarSummary radar_msg;

  radar_msg.header.stamp = this->rosnode->now();
  radar_msg.header.frame_id = this->radar_sensor_frameid;

  ignition::math::Pose3d cameraPose = gazebo::msgs::ConvertIgn(_msg.pose());

  std::ostringstream logStream;
  ignition::math::Pose3d modelPose;

  for (int i = 0; i < _msg.model_size(); ++i)
  {
    std::string modelName = _msg.model(i).name();
    std::string modelType = DetermineModelType(modelName);
    auto modelPtr = this->world->ModelByName(modelName);

    if (!this->ModelTypeToPublish(modelType))
    {
      logStream << "Not publishing model: " << modelName << " of type: " << modelType << std::endl;
    }
    else
    {
      logStream << "Publishing model: " << modelName << " of type: " << modelType << std::endl;
      modelPose = gazebo::msgs::ConvertIgn(_msg.model(i).pose());
      uint16_t t_code = GetTransponderCode(modelName);
      this->AddNoise(modelPose);
      this->AppendRadarContact(radar_msg, cameraPose, modelPose, t_code, radar_msg.header);
    }

    // Check any children models
    auto nestedModels = modelPtr->NestedModels();
    for (auto nestedModel : nestedModels)
    {
      modelName = nestedModel->GetName();
      modelType = DetermineModelType(modelName);
      if (!this->ModelTypeToPublish(modelType))
      {
        logStream << "Not publishing model: " << modelName << " of type: " << modelType << std::endl;
        continue;
      }
      logStream << "Publishing model: " << modelName << " of type: " << modelType << std::endl;
      modelPose = (nestedModel->WorldPose()) - cameraPose;
      uint16_t t_code = GetTransponderCode(modelName);
      this->AddNoise(modelPose);
      this->AppendRadarContact(radar_msg, cameraPose, modelPose, t_code, radar_msg.header);
    }
  }

  if (!logStream.str().empty())
  {
    RCLCPP_DEBUG(this->rosnode->get_logger(), "%s", logStream.str().c_str());
  }
  this->radarPub->publish(radar_msg);
}

bool ModeCRadarPlugin::ModelTypeToPublish(const std::string &modelType)
{
  bool publishModelType = true;

  if (this->onlyPublishKnownModels)
  {
    auto it = std::find(this->knownModelTypes.begin(), this->knownModelTypes.end(), modelType);
    bool knownModel = it != this->knownModelTypes.end();
    publishModelType = knownModel;
  }
  return publishModelType;
}

void ModeCRadarPlugin::AddNoise(ignition::math::Pose3d &pose)
{
  if (this->noiseModels.find("POSITION_NOISE") != this->noiseModels.end())
  {
    pose.Pos().X(this->noiseModels["POSITION_NOISE"]->Apply(pose.Pos().X()));
    pose.Pos().Y(this->noiseModels["POSITION_NOISE"]->Apply(pose.Pos().Y()));
    pose.Pos().Z(this->noiseModels["POSITION_NOISE"]->Apply(pose.Pos().Z()));
  }
}

void ModeCRadarPlugin::AppendRadarContact(
    gazebo_radar_plugin::msg::ModeCRadarSummary &radar_msg,
    const ignition::math::Pose3d &cameraPose, const ignition::math::Pose3d &modelPose,
    const uint16_t code, const std_msgs::msg::Header &header)
{
  gazebo_radar_plugin::msg::ModeCRadar contact;
  contact.header = header;

  ignition::math::Pose3d temp = modelPose; // GetYaw isn't const??
  contact.range = temp.Pos().Length();
  contact.bearing = atan2(-1 * modelPose.Pos().Y(), modelPose.Pos().X());
  double azimouth = asin(modelPose.Pos().Z() / contact.range);
  contact.altitude = cameraPose.Pos().Z() + modelPose.Pos().Z();
  contact.code = code;
  contact.ident = false;

  if (fabs(contact.bearing) * 2 < this->hfov &&
      fabs(azimouth) * 2 < this->vfov &&
      contact.range < this->max_range)
  {
    radar_msg.contacts.push_back(contact);
  }
}
