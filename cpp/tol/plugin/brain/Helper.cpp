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

#include <boost/make_shared.hpp>

#include "Helper.h"

using namespace tol;

const std::vector<revolve::brain::ActuatorPtr>
Helper::createWrapper(const std::vector<revolve::gazebo::MotorPtr> &original)
{
  std::vector<revolve::brain::ActuatorPtr> result;
  for (unsigned int i = 0; i < original.size(); i++) {
    result.push_back(boost::make_shared<tol::Actuator>(tol::Actuator(original[i])));
  }

  return result;
}

const std::vector<revolve::brain::SensorPtr>
Helper::createWrapper(const std::vector<revolve::gazebo::SensorPtr> &original)
{
  std::vector<revolve::brain::SensorPtr> result;
  for (unsigned int i = 0; i < original.size(); i++) {
    result.push_back(boost::make_shared<tol::Sensor>(tol::Sensor(original[i])));
  }

  return result;
}

Helper::RobotType Helper::parseRobotType(const std::string &value) {
    if (value.compare("spider9") == 0)
        return RobotType::spider9;
    if (value.compare("spider13") == 0)
        return RobotType::spider13;
    if (value.compare("spider17") == 0)
        return RobotType::spider17;

    if (value.compare("gecko7") == 0)
        return RobotType::gecko7;
    if (value.compare("gecko12") == 0)
        return RobotType::gecko12;
    if (value.compare("gecko17") == 0)
        return RobotType::gecko17;

    if (value.compare("snake5") == 0)
        return RobotType::snake5;
    if (value.compare("snake7") == 0)
        return RobotType::snake7;
    if (value.compare("snake9") == 0)
        return RobotType::snake9;

    if (value.compare("babyA") == 0)
        return RobotType::babyA;
    if (value.compare("babyB") == 0)
        return RobotType::babyB;
    if (value.compare("babyC") == 0)
        return RobotType::babyC;

    //default value
    std::cerr << "Impossible to parse robot type (" << value << ")\nThrowing exception!"<<std::endl;
    throw std::invalid_argument("robot type impossible to parse");
}

std::ostream &operator<<(std::ostream &os, tol::Helper::RobotType type) {
    switch (type) {
        case tol::Helper::RobotType::spider9 :
            os << "spider9";
            break;
        case tol::Helper::RobotType::spider13:
            os << "spider13";
            break;
        case tol::Helper::RobotType::spider17:
            os << "spider17";
            break;
        case tol::Helper::RobotType::gecko7  :
            os << "gecko7";
            break;
        case tol::Helper::RobotType::gecko12 :
            os << "gecko12";
            break;
        case tol::Helper::RobotType::gecko17 :
            os << "gecko17";
            break;
        case tol::Helper::RobotType::snake5  :
            os << "snake5";
            break;
        case tol::Helper::RobotType::snake7  :
            os << "snake7";
            break;
        case tol::Helper::RobotType::snake9  :
            os << "snake9";
            break;
        case tol::Helper::RobotType::babyA   :
            os << "babyA";
            break;
        case tol::Helper::RobotType::babyB   :
            os << "babyB";
            break;
        case tol::Helper::RobotType::babyC   :
            os << "babyC";
            break;
        default      :
            os.setstate(std::ios_base::failbit);
    }
    return os;
}


std::vector<std::vector<float>> Helper::GetCoordinatesFromRobotType(const Helper::RobotType robot_type)
{
    std::vector< std::vector< float > > coordinates;
    switch (robot_type)
    {
        case Helper::spider9:
            // SPIDER 9
            //     #
            //     #
            // # # O # #
            //     #
            //     #
            coordinates = std::vector< std::vector< float > >
                    ({
                             // Leg00Joint Leg01Joint
                             {1,    0},//    1},
                             {.5,   0},//    -1},
                             // Leg10Joint Leg11Joint
                             {-1,   0},//    1},
                             {-.5f, 0},//    -1},
                             // Leg20Joint Leg21Joint
                             {0,    1},//    1},
                             {0,    .5},//   -1},
                             // Leg30Joint Leg31Joint
                             {0,    -1},//   1},
                             {0,    -.5f}//, -1}
                     });
            break;
        case Helper::spider13:
            // SPIDER 13
            //       #
            //       #
            //       #
            // # # # O # # #
            //       #
            //       #
            //       #
            coordinates = std::vector< std::vector< float > >
                    ({
                             // Leg00Joint Leg01Joint Leg02Joint
                             {.333,   0},
                             {.666,   0},
                             {1,      0},
                             // Leg10Joint Leg11Joint Leg12Joint
                             {-.333f, 0},
                             {-.666f, 0},
                             {-1,     0},
                             // Leg20Joint Leg21Joint Leg22Joint
                             {0,      .333},
                             {0,      .666},
                             {0,      1},
                             // Leg30Joint Leg31Joint Leg32Joint
                             {0,      -.333f},
                             {0,      -.666f},
                             {0,      -1},
                     });
            break;
        case Helper::spider17:
            // SPIDER 17
            //         #
            //         #
            //         #
            //         #
            // # # # # O # # # #
            //         #
            //         #
            //         #
            //         #
            coordinates = std::vector< std::vector< float > >
                    ({
                             // Leg00Joint Leg01Joint Leg02Joint
                             {.25,   0},
                             {.5,    0},
                             {.75,   0},
                             {1,     0},
                             // Leg10Joint Leg11Joint Leg12Joint
                             {-.25f, 0},
                             {-.5f,  0},
                             {-.75f, 0},
                             {-1,    0},
                             // Leg20Joint Leg21Joint Leg22Joint
                             {0,     .25},
                             {0,     .5},
                             {0,     .75},
                             {0,     1},
                             // Leg30Joint Leg31Joint Leg32Joint
                             {0,     -.25f},
                             {0,     -.5f},
                             {0,     -.75f},
                             {0,     -1},
                     });
            break;
        case Helper::gecko7:
            // GECKO 5
            // #   #
            // O # #
            // #   #
            coordinates = std::vector< std::vector< float > >
                    ({
                             // Leg00Joint
                             {-1,   +1},
                             // Leg01Joint
                             {-1,   -1},
                             // BodyJoint0
                             {-.5f, 0},
                             // BodyJoint1
                             {+.5f, 0},
                             // Leg10Joint
                             {+1,   +1},
                             // Leg11Joint
                             {+1,   -1},
                     });
            break;
        case Helper::gecko12:

            // GECKO 12
            // #     #
            // #     #
            // O # # #
            // #     #
            // #     #
            coordinates = std::vector< std::vector< float > >
                    ({
                             // Leg00Joint Leg001Joint
                             {-1.0f, +0.5f},
                             {-1,    +1},
                             // Leg01Joint Leg011Joint
                             {-1.0f, -0.5f},
                             {-1,    -1},
                             // BodyJoint0 BodyJoint1 BodyJoint2
                             {-.5f,  0},
                             {0,     0},
                             {+.5f,  0},
                             // Leg10Joint Leg101Joint
                             {+1,    +0.5f},
                             {+1,    +1},
                             // Leg11Joint Leg111Joint
                             {+1,    -0.5f},
                             {+1,    -1},
                     });
            break;
        case Helper::gecko17:
            // GECKO 17
            // #     #
            // #     #
            // O # # #
            // #     #
            // #     #
            coordinates = std::vector< std::vector< float > >
                    ({
                             // Leg00Joint Leg001Joint Leg002Joint
                             {-1.0f,  +.333f},
                             {-1.0f,  +.666f},
                             {-1,     +1},
                             // Leg01Joint Leg011Joint Leg012Joint
                             {-1.0f,  -.333f},
                             {-1.0f,  -.333f},
                             {-1,     -1},
                             // BodyJoint0 BodyJoint1 BodyJoint2 BodyJoint3
                             {-.666f, 0},
                             {-.333f, 0},
                             {+.333f, 0},
                             {+.666f, 0},
                             // Leg10Joint Leg101Joint Leg102Joint
                             {+1,     +.333f},
                             {+1,     +.666f},
                             {+1,     +1},
                             // Leg11Joint Leg111Joint Leg112Joint
                             {+1,     -.333f},
                             {+1,     -.666f},
                             {+1,     -1},
                     });
            break;
        case Helper::snake5:


            // SNAKE 5
            //
            // # # O # #
            //
            coordinates = std::vector< std::vector< float > >
                    ({
                             // Leg00Joint
                             {-.5f, 0},
                             // Leg01Joint
                             {-1,   0},
                             // Leg10Joint
                             {+.5f, 0},
                             // Leg11Joint
                             {+1,   0},
                     });
            break;
        case Helper::snake7:
            // SNAKE 7
            //
            // # # # O # # #
            //
            coordinates = std::vector< std::vector< float > >
                    ({
                             // Leg00Joint
                             {-.333f, 0},
                             // Leg01Joint
                             {-.666f, 0},
                             // Leg02Joint
                             {-1,     0},
                             // Leg10Joint
                             {+.333f, 0},
                             // Leg11Joint
                             {+.666f, 0},
                             // Leg12Joint
                             {+1,     0},
                     });
            break;
        case Helper::snake9:
            // SNAKE 9
            //
            // # # # # O # # # #
            //
            coordinates = std::vector< std::vector< float > >
                    ({
                             // Leg00Joint
                             {-.25f, 0},
                             // Leg01Joint
                             {-.50f, 0},
                             // Leg02Joint
                             {-.75f, 0},
                             // Leg03Joint
                             {-1,    0},
                             // Leg10Joint
                             {+.25f, 0},
                             // Leg11Joint
                             {+.50f, 0},
                             // Leg12Joint
                             {+.75f, 0},
                             // Leg13Joint
                             {+1,    0},
                     });
            break;
        case Helper::babyA:

            // BABY 1
            // #
            // #   #
            // O # #
            // #   #
            coordinates = std::vector< std::vector< float > >
                    ({
                             // Leg00Joint
                             {-1.0f, +1},
                             // Leg01Joint
                             {-1.0f, -.3f},
                             // Leg011Joint
                             {-1.0f, -.6f},
                             // Leg021Joint
                             {-1.0f, -1.0f},
                             // BodyJoint0
                             {-.5f,  0},
                             // BodyJoint1
                             {+.5f,  0},
                             // Leg10Joint
                             {+1,    +1},
                             // Leg11Joint
                             {+1,    -1},
                     });
            break;
        case Helper::babyB:
            // BABY 2
            //
            //       #
            // # # # O # # #
            //       #
            //       #
            //       #
            coordinates = std::vector< std::vector< float > >
                    ({
                             // Leg00Joint Leg01Joint Leg02Joint
                             {1,     0},
                             {.666f, 0},
                             {.333f, 0},
                             // Leg10Joint
                             {-1,    0},
                             // Leg20Joint Leg21Joint Leg22Joint
                             {0,     1},
                             {0,     .666f},
                             {0,     .333f},
                             // Leg30Joint Leg31Joint Leg32Joint
                             {0,     -1},
                             {0,     -.666f},
                             {0,     .333f},
                     });
            break;
        case Helper::babyC:
            // BABY 3
            // #       #
            // #       x
            // #       #
            // O # # # #
            // #       #
            // #       #
            // #       #
            coordinates = std::vector< std::vector< float > >
                    ({
                             // Leg00Joint Leg001Joint Leg002Joint
                             {-1.0f,  +.333f},
                             {-1.0f,  +.666f},
                             {-1.0f,  +1},
                             // Leg01Joint Leg011Joint Leg012Joint
                             {-1.0f,  -.333f},
                             {-1.0f,  -.333f},
                             {-1.0f,  -1},
                             // BodyJoint0 BodyJoint1 BodyJoint2 BodyJoint3
                             {-.666f, 0},
                             {-.333f, 0},
                             {+.333f, 0},
                             {+.666f, 0},
                             // Leg10Joint Leg101Joint Leg102Joint
                             {+1.0f,  +.333f},
                             {+1.0f,  +.666f},
                             {+1.0f,  +1},
                             // Leg11Joint Leg111Joint Leg112Joint
                             {+1.0f,  -.333f},
                             {+1.0f,  -.666f},
                             {+1.0f,  -.9f},
                     });
            break;
    }

    return coordinates;
}