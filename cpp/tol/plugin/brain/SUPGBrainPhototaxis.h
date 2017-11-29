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

#ifndef TOL_SUPGBRAINPHOTOTAXIS_H
#define TOL_SUPGBRAINPHOTOTAXIS_H

#include "revolve/gazebo/brain/Brain.h"
#include "brain/SUPGBrainPhototaxis.h"
#include "../FakeLightSensor.h"

namespace tol {

class SUPGBrainPhototaxis : public revolve::gazebo::Brain
                          , private revolve::brain::SUPGBrainPhototaxis
{
public:
    SUPGBrainPhototaxis(const std::string &robot_name,
                        revolve::brain::EvaluatorPtr evaluator,
                        double light_radius_distance,
                        const std::vector< std::vector< float > >& neuron_coordinates,
                        const std::vector< revolve::gazebo::MotorPtr >& actuators,
                        std::vector< revolve::gazebo::SensorPtr >& sensors);
    virtual ~SUPGBrainPhototaxis();

    void update(const std::vector< revolve::gazebo::MotorPtr > &motors,
                const std::vector< revolve::gazebo::SensorPtr > &sensors,
                double t, double step) override;

    void updateRobotPosition(ignition::math::Pose3d &robot_position);

    void loadOfflineBrain(const std::string &filename);

    void setLightCoordinates(const std::vector<float> &relative_coordinates) override;

private: // methods
    static const std::vector<revolve::brain::SensorPtr>
    createEnhancedSensorWrapper(const std::vector<revolve::gazebo::SensorPtr> &original);


private:
    boost::shared_ptr<FakeLightSensor> light_sensor_left,
                                       light_sensor_right;
    ignition::math::Pose3<double> robot_position;
};

}

#endif // TOL_SUPGBRAINPHOTOTAXIS_H
