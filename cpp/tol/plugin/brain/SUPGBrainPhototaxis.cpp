/*
 * <one line to give the library's name and an idea of what it does.>
 * Copyright (C) 2016  Matteo De Carlo <matteo.dek@covolunablu.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "SUPGBrainPhototaxis.h"
#include "Helper.h"
#include <boost/make_shared.hpp>

using namespace tol;

SUPGBrainPhototaxis::SUPGBrainPhototaxis(const std::string &robot_name,
                                         revolve::brain::EvaluatorPtr evaluator,
                                         double light_radius_distance,
                                         const std::vector <std::vector<float>> &neuron_coordinates,
                                         const std::vector <revolve::gazebo::MotorPtr> &actuators,
                                         std::vector <revolve::gazebo::SensorPtr> &sensors,
                                         const PHASE testing_phase)
    : revolve::brain::SUPGBrainPhototaxis(
        robot_name,
        std::move(evaluator),
        nullptr,
        nullptr,
        light_radius_distance,
        neuron_coordinates,
        Helper::createWrapper(actuators),
        createEnhancedSensorWrapper(sensors),
        testing_phase
    )
        , relative_light_coordinates(2, 100.0)
{

    light_constructor_left = [this] (std::vector<float> coordinates)
        -> boost::shared_ptr<FakeLightSensor>
    {
        coordinates[0] += 4; //cm from the center
        ignition::math::Vector3d light_pos = this->generateLightPos(coordinates);
        // this function is not supposed to delete the light
        FakeLightSensor *new_sensor = new FakeLightSensor("sensor_left", 160, light_pos);
        if (light_sensor_left)
            light_sensor_left->replace(new_sensor);
        else
            light_sensor_left.reset(new_sensor);
        return light_sensor_left;
    };

    light_constructor_right = [this] (std::vector<float> coordinates)
        -> boost::shared_ptr<FakeLightSensor>
    {
        coordinates[0] -= 4; //cm from the center
        ignition::math::Vector3d light_pos = this->generateLightPos(coordinates);
        // this function is not supposed to delete the light
        FakeLightSensor *new_sensor = new FakeLightSensor("sensor_right", 160, light_pos);
        if (light_sensor_right)
            light_sensor_right->replace(new_sensor);
        else
            light_sensor_right.reset(new_sensor);
        return light_sensor_right;
    };

    light_constructor_left({1,1});
    light_constructor_right({1,1});
    sensors.push_back(light_sensor_left);
    sensors.push_back(light_sensor_right);
}

SUPGBrainPhototaxis::~SUPGBrainPhototaxis()
{}


ignition::math::Vector3d SUPGBrainPhototaxis::generateLightPos(std::vector<float> coordinates)
{
    ignition::math::Vector3d offset(coordinates[0]/100, coordinates[1]/100, 0);
    ignition::math::Vector3d light_pos = this->robot_position.CoordPositionAdd(offset);
    return light_pos;
}

void SUPGBrainPhototaxis::update(const std::vector <revolve::gazebo::MotorPtr> &motors,
                                 const std::vector <revolve::gazebo::SensorPtr> &sensors,
                                 double t, double step)
{
    revolve::brain::SUPGBrainPhototaxis::update(Helper::createWrapper(motors),
                                                Helper::createWrapper(sensors),
                                                t, step);
}

void SUPGBrainPhototaxis::updateRobotPosition(ignition::math::Pose3d &robot_position)
{
    this->robot_position = ignition::math::Pose3d(robot_position);
    light_sensor_left->updateRobotPosition(robot_position);
    light_sensor_right->updateRobotPosition(robot_position);
}


const std::vector<revolve::brain::SensorPtr> tol::SUPGBrainPhototaxis::createEnhancedSensorWrapper(const std::vector<revolve::gazebo::SensorPtr>& original)
{
    std::vector<revolve::brain::SensorPtr> result = Helper::createWrapper(original);
    result.push_back(boost::make_shared<tol::FakeLightSensor>("sensor_1_fake_filler", 0, ignition::math::Vector3d()));
    result.push_back(boost::make_shared<tol::FakeLightSensor>("sensor_2_fake_filler", 0, ignition::math::Vector3d()));

    return result;
}

void SUPGBrainPhototaxis::loadOfflineBrain(const std::string &filename)
{
    revolve::brain::SUPGBrainPhototaxis::loadOfflineBrain(filename);
    setOffline(true);
}

void SUPGBrainPhototaxis::setLightCoordinates(const std::vector<float> &relative_coordinates)
{
    this->relative_light_coordinates = std::vector<float>(relative_coordinates);
    revolve::brain::SUPGBrainPhototaxis::setLightCoordinates(relative_coordinates);
    if (sphere_model) {
        const ignition::math::Vector3d position = this->generateLightPos(relative_coordinates);
        const gazebo::math::Quaternion rotation;
        const gazebo::math::Pose light_pose(position, rotation);
        sphere_model->SetWorldPose(light_pose);
    }
}

void SUPGBrainPhototaxis::addLightModel(gazebo::physics::ModelPtr sphere_model)
{
    if (!this->sphere_model && sphere_model) {
        this->sphere_model = std::move(sphere_model);
        const ignition::math::Vector3d position = this->generateLightPos(relative_light_coordinates);
        const gazebo::math::Quaternion rotation;
        const gazebo::math::Pose light_pose(position, rotation);
        this->sphere_model->SetWorldPose(light_pose);
    }
}

gazebo::physics::ModelPtr SUPGBrainPhototaxis::getSphereModel() const
{
    return this->sphere_model;
}


