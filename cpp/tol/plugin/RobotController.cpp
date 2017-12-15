/*
 * RobotController.cpp
 *
 *  Created on: May 9, 2015
 *      Author: elte
 */

#include "RobotController.h"

#include "brain/RLPower_Splines.h"
#include "brain/RLPower_CPPN.h"
#include "brain/HyperNEAT_CPPN.h"
#include "brain/HyperNEAT_Splines.h"
#include "brain/HyperNEAT_MlmpCPG.h"
#include "brain/MLMPCPGBrain.h"
#include "brain/GenericLearnerBrain.h"
#include "brain/YamlBodyParser.h"
#include "brain/SUPGBrain.h"
#include "brain/SUPGBrainPhototaxis.h"
#include "brain/supg/SUPGGenomeManager.h"
#include "brain/Helper.h"

#include <boost/make_shared.hpp>

#include <revolve/gazebo/motors/Motor.h>
#include <revolve/gazebo/sensors/VirtualSensor.h>
#include <brain/learner/HyperAccNEATLearner_CPGController.h>

#define SUPG_SENSOR_DISTANCE 200 //cm

namespace tol
{

  const char *getVARenv(const char *var_name)
  {
    const char *env_p = std::getenv(var_name);
    if (env_p)
    {
      std::cout << "ENV " << var_name << " is: " << env_p << std::endl;
    }
    else
    {
      std::cout << "ENV " << var_name << " not found, using default value: ";
    }
    return env_p;
  }

  const NEAT::GeneticSearchType getGeneticSearchType(const std::string &value)
  {
    if (value.compare("PHASED") == 0)
    {
      return NEAT::GeneticSearchType::PHASED;
    }

    if (value.compare("BLENDED") == 0)
    {
      return NEAT::GeneticSearchType::BLENDED;
    }

    if (value.compare("COMPLEXIFY") == 0)
    {
      return NEAT::GeneticSearchType::COMPLEXIFY;
    }

    //default value
    return NEAT::GeneticSearchType::PHASED;
  }

  const char *getGeneticSearchType(const NEAT::GeneticSearchType value)
  {
    switch (value)
    {
      case NEAT::GeneticSearchType::BLENDED:
        return "NEAT::GeneticSearchType::BLENDED";
      case NEAT::GeneticSearchType::PHASED:
        return "NEAT::GeneticSearchType::PHASED";
      case NEAT::GeneticSearchType::COMPLEXIFY:
        return "NEAT::GeneticSearchType::COMPLEXIFY";
      default:
        return "undefined";
    }
  }

  void init_asyncneat(
          const std::string &robot_name,
          std::unique_ptr< NEAT::GenomeManager > custom_genome_manager)
  {

    if (custom_genome_manager)
    {
      AsyncNeat::Init(std::move(custom_genome_manager));
    }
    else
    {
      AsyncNeat::Init(robot_name);
    }
    unsigned int populationSize = 10;
    NEAT::real_t mutate_add_node_prob = 0.01;
    NEAT::real_t mutate_add_link_prob = 0.3;
    NEAT::GeneticSearchType
            geneticSearchType = NEAT::GeneticSearchType::COMPLEXIFY;

    if (const char *env_p = getVARenv("NEAT_POP_SIZE"))
    {
      try
      {
        populationSize = (unsigned int)std::stoul(env_p);
      } catch (const std::invalid_argument &e)
      {
        std::cout
                << "ERROR DECODING STRING \"NEAT_POP_SIZE\" to unsigned long:"
                << " using default value "
                << populationSize
                << " instead"
                << std::endl;
      }

    }
    else
    {
      std::cout << populationSize << std::endl;
    }

    if (const char *env_p = getVARenv("NEAT_MUTATE_ADD_NODE_PROB"))
    {
      try
      {
        mutate_add_node_prob = (float)std::stod(env_p);
      } catch (const std::invalid_argument &e)
      {
        std::cout
                << "ERROR DECODING STRING \"NEAT_MUTATE_ADD_NODE_PROB\" to double:"
                << " using default value "
                << mutate_add_node_prob
                << " instead"
                << std::endl;
      }
    }
    else
    {
      std::cout << mutate_add_node_prob << std::endl;
    }

    if (const char *env_p = getVARenv("NEAT_MUTATE_ADD_LINK_PROB"))
    {
      try
      {
        mutate_add_link_prob = (float)std::stod(env_p);
      } catch (const std::invalid_argument &e)
      {
        std::cout
                << "ERROR DECODING STRING \"NEAT_MUTATE_ADD_LINK_PROB\" to double:"
                << " using default value "
                << mutate_add_link_prob
                << " instead"
                << std::endl;
      }
    }
    else
    {
      std::cout << mutate_add_link_prob << std::endl;
    }

    if (const char *env_p = getVARenv("NEAT_SEARCH_TYPE"))
    {
      geneticSearchType = getGeneticSearchType(env_p);
    }
    else
    {
      std::cout << getGeneticSearchType(geneticSearchType) << std::endl;
    }

    AsyncNeat::SetPopulationSize(populationSize); // 10 - 25 - 50 - 75 - 100 - 1000
    AsyncNeat::SetMutateAddNodeProb(mutate_add_node_prob);
    AsyncNeat::SetMutateAddLinkProb(mutate_add_link_prob);
    std::cout
            << "Setting up genetic search type to: "
            << getGeneticSearchType(geneticSearchType)
            << std::endl;
    AsyncNeat::SetSearchType(geneticSearchType);
  }

  RobotController::RobotController()
    : algorithm("undefined")
  {
  }

  RobotController::~RobotController()
  {
    AsyncNeat::CleanUp();
  }

  void RobotController::Load(
          ::gazebo::physics::ModelPtr _parent,
          sdf::ElementPtr _sdf)
  {
    ::revolve::gazebo::RobotController::Load(_parent, _sdf);
    std::cout << "ToL Robot loaded." << std::endl;
  }

  void RobotController::LoadBrain(sdf::ElementPtr sdf)
  {
    try
    {
      evaluator_ = boost::make_shared< Evaluator >();
      const std::string &robot_name = this->model->GetName();

      unsigned int motor_n = 0; //motors_.size();
      for (const auto &motor : motors_)
      {
        motor_n += motor->outputs();
      }
      unsigned int sensor_n = 0; //sensors_.size();
      for (const auto &sensor : sensors_)
      {
        sensor_n += sensor->inputs();
      }

      if (not sdf->HasElement("rv:brain"))
      {
        std::cerr
                << "No robot brain detected, this is probably an error."
                << std::endl;
        return;
      }
      auto brain = sdf->GetElement("rv:brain");

      if (not brain->HasAttribute("algorithm"))
      {
        std::cerr << "Brain does not define type, this is probably an error."
                  << std::endl;
        return;
      }

      algorithm = brain->GetAttribute("algorithm")->GetAsString();

      std::cout<<"Loading brain: "<<algorithm<<std::endl;

      if ("rlpower::spline" == algorithm)
      {
        brain_.reset(new tol::RLPower_Splines(
                robot_name,
                brain,
                evaluator_,
                motors_,
                sensors_));
      }
      else if ("rlpower::net" == algorithm)
      {
        brain_.reset(new tol::RLPower_CPG(
                robot_name,
                brain,
                evaluator_,
                motors_,
                sensors_));
      }
      else if ("hyperneat::net" == algorithm)
      {
        brain_.reset(new tol::HyperNEAT_CPG(
                robot_name,
                brain,
                evaluator_,
                motors_,
                sensors_));
      }
      else if ("rafhyperneat::mlmp_cpg" == algorithm)
      {
        brain_.reset(new tol::HyperNEAT_MlmpCPG(
                robot_name,
                brain,
                evaluator_,
                motors_,
                sensors_));
      }
      else if ("hyperneat::spline" == algorithm)
      {
        brain_.reset(new tol::HyperNEAT_Splines(
                robot_name,
                brain,
                evaluator_,
                motors_,
                sensors_));
      }
      else if ("rlpower::mlmp_cpg" == algorithm)
      {
        brain_.reset(new tol::MlmpCPGBrain(
                robot_name,
                evaluator_,
                motor_n,
                sensor_n));
      }
      else if ("hyperneat::mlmp_cpg" == algorithm)
      {

        init_asyncneat(robot_name, std::unique_ptr< NEAT::GenomeManager >());
        std::string modelName = this->model->GetName();
        tol::YamlBodyParser *parser = new tol::YamlBodyParser();
        modelName = modelName.substr(0, modelName.find("-")) + ".yaml";
        parser->parseFile(modelName);
        std::vector< std::vector< bool>> connections = parser->connections();
        std::vector< std::vector< float>>
                cpgs_coordinates = parser->coordinates();
        brain_.reset(new tol::GenericLearnerBrain(
                new revolve::brain::HyperAccNEATLearner_CPGController(
                        robot_name,
                        evaluator_,
                        sensor_n,
                        motor_n,
                        2, // coordinates cardinality
                        connections,
                        cpgs_coordinates,
                        30, //seconds
                        999 // -1 // infinite evaluations
                )
        ));
      }
      else if ("hyperneat::supg_phototaxis" == algorithm)
      {
        init_asyncneat(
                robot_name,
                std::unique_ptr< NEAT::GenomeManager >(new SUPGGenomeManager(
                        robot_name)
                ));

        std::string modelName = this->model->GetName();
        size_t start = modelName.rfind('/')+1;
        size_t end = modelName.find('-');
        modelName = modelName.substr(start, end-start);
        std::cout<<"init hyperneat::supg_phototaxis; modelName = "<<modelName<<std::endl;
        const std::string robot_type_str = modelName;
        const Helper::RobotType
                robot_type = Helper::parseRobotType(robot_type_str);
        std::cout
                << "Loading SUPG configuration for robot "
                << robot_type
                << std::endl;


        std::vector< std::vector< float > > coordinates = Helper::GetCoordinatesFromRobotType(robot_type);

        revolve::brain::SUPGBrainPhototaxis::PHASE phase;
        {
          std::string phase_string;
          if (const char *env_p = getVARenv("SUPG_PHASE"))
          {
            phase_string = env_p;
          }
          else
          {
            phase_string = "CENTER";
            std::cout << "CENTER" << std::endl;
          }

          if (phase_string == "CENTER") phase = revolve::brain::SUPGBrainPhototaxis::PHASE::CENTER;
          else if (phase_string == "LEFT") phase = revolve::brain::SUPGBrainPhototaxis::PHASE::LEFT;
          else if (phase_string == "RIGHT") phase = revolve::brain::SUPGBrainPhototaxis::PHASE::RIGHT;
          else if (phase_string == "MORELEFT") phase = revolve::brain::SUPGBrainPhototaxis::PHASE::MORELEFT;
          else if (phase_string == "MORERIGHT") phase = revolve::brain::SUPGBrainPhototaxis::PHASE::MORERIGHT;
          else throw std::runtime_error("PHASE to test not recognized, please set the ENV variable \"SUPG_PHASE\" accordingly");
        }

        //brain_.reset(new SUPGBrain(evaluator_, coordinates, motors_, sensors_));
        SUPGBrainPhototaxis *brain = new SUPGBrainPhototaxis(
                robot_name,
                evaluator_,
                SUPG_SENSOR_DISTANCE,
                coordinates,
                motors_,
                sensors_,
                phase
      );

        sdf::SDF sphereSDF;
        // spawn is underground so it won't influence the simulation if unused
        sphereSDF.SetFromString("\
              <sdf version ='1.5'>\
                <model name ='sphere'>\
                  <pose>1 0 -.5 0 0 0</pose>\
                  <static>false</static>\
                    <link name ='link'>\
                      <gravity>true</gravity>\
                      <pose>0 0 0 0 0 0</pose>\
                      <collision name ='collision'>\
                        <geometry>\
                          <sphere><radius>0.01</radius></sphere>\
                        </geometry>\
                      </collision>\
                    <visual name ='visual'>\
                      <geometry>\
                        <sphere><radius>0.01</radius></sphere>\
                      </geometry>\
                    </visual>\
                  </link>\
                </model>\
              </sdf>");
        // Demonstrate using a custom model name.
        sdf::ElementPtr model = sphereSDF.Root()->GetElement("model");
        model->GetAttribute("name")->SetFromString("unique_sphere");
        world->InsertModelSDF(sphereSDF);

        brain_.reset(brain);

      }
      else if ("hyperneat::supg_phototaxis::replay" == algorithm)
      {
          init_asyncneat(
                  robot_name,
                  std::unique_ptr<NEAT::GenomeManager>(new SUPGGenomeManager(
                          robot_name)
                  ));

          std::string modelName = this->model->GetName();
          size_t start = modelName.rfind('/') + 1;
          size_t end = modelName.find('-');
          modelName = modelName.substr(start, end - start);
          std::cout << "init hyperneat::supg_phototaxis::replay; modelName = " << modelName << std::endl;
          const std::string robot_type_str = modelName;
          const Helper::RobotType
                  robot_type = Helper::parseRobotType(robot_type_str);
          std::cout
                  << "Loading SUPG configuration for robot "
                  << robot_type
                  << std::endl;


          revolve::brain::SUPGBrainPhototaxis::PHASE phase;
          {
            std::string phase_string;
            if (const char *env_p = getVARenv("SUPG_PHASE"))
            {
                phase_string = env_p;
            }
            else
            {
                phase_string = "CENTER";
                std::cout << "CENTER" << std::endl;
            }

            if (phase_string == "CENTER") phase = revolve::brain::SUPGBrainPhototaxis::PHASE::CENTER;
            else if (phase_string == "LEFT") phase = revolve::brain::SUPGBrainPhototaxis::PHASE::LEFT;
            else if (phase_string == "RIGHT") phase = revolve::brain::SUPGBrainPhototaxis::PHASE::RIGHT;
            else if (phase_string == "MORELEFT") phase = revolve::brain::SUPGBrainPhototaxis::PHASE::MORELEFT;
            else if (phase_string == "MORERIGHT") phase = revolve::brain::SUPGBrainPhototaxis::PHASE::MORERIGHT;
            else throw std::runtime_error("PHASE to test not recognized, please set the ENV variable \"SUPG_PHASE\" accordingly");
          }

          std::vector<std::vector<float> > coordinates = Helper::GetCoordinatesFromRobotType(robot_type);

          SUPGBrainPhototaxis *brain =
                  new SUPGBrainPhototaxis(robot_name,
                                          evaluator_,
                                          SUPG_SENSOR_DISTANCE,
                                          coordinates,
                                          motors_,
                                          sensors_,
                                          phase);

          brain->loadOfflineBrain(getVARenv("GENOME_FILE"));
          sdf::SDF sphereSDF;
          sphereSDF.SetFromString("\
                  <sdf version ='1.5'>\
                    <model name ='sphere'>\
                      <pose>1 0 .01 0 0 0</pose>\
                      <static>false</static>\
                        <link name ='link'>\
                          <gravity>true</gravity>\
                          <pose>0 0 0 0 0 0</pose>\
                          <collision name ='collision'>\
                            <geometry>\
                              <sphere><radius>0.01</radius></sphere>\
                            </geometry>\
                          </collision>\
                        <visual name ='visual'>\
                          <geometry>\
                            <sphere><radius>0.01</radius></sphere>\
                          </geometry>\
                        </visual>\
                      </link>\
                    </model>\
                  </sdf>");
          // Demonstrate using a custom model name.
          sdf::ElementPtr model = sphereSDF.Root()->GetElement("model");
          model->GetAttribute("name")->SetFromString("unique_sphere");
          world->InsertModelSDF(sphereSDF);
          gazebo::physics::ModelPtr sphere_model = world->GetModel("unique_sphere");
          brain->addLightModel(sphere_model);



          std::vector<float> relative_coordinates;

          static const double pi = std::acos(-1);
          static const double angle_15 = pi/12;
          static const double angle_52_5 = 7*pi/24;

          const double radius = SUPG_SENSOR_DISTANCE;
          const float x_52_5 = (const float) (std::cos(angle_52_5) * radius);
          const float y_52_5 = (const float) (std::sin(angle_52_5) * radius);
          const float x_15   = (const float) (std::cos(angle_15) * radius);
          const float y_15   = (const float) (std::sin(angle_15) * radius);

          switch (phase) {
              case revolve::brain::SUPGBrainPhototaxis::PHASE::CENTER:
                  relative_coordinates = {0, -static_cast<float>(radius)};
                  break;
              case revolve::brain::SUPGBrainPhototaxis::PHASE::LEFT:
                  relative_coordinates = {-x_52_5, -y_52_5};
                  break;
              case revolve::brain::SUPGBrainPhototaxis::PHASE::RIGHT:
                  relative_coordinates = {x_52_5, -y_52_5};
                  break;
              case revolve::brain::SUPGBrainPhototaxis::PHASE::MORELEFT:
                  relative_coordinates = {-x_15, -y_15};
                  break;
              case revolve::brain::SUPGBrainPhototaxis::PHASE::MORERIGHT:
                  relative_coordinates = {x_15, -y_15};
                  break;
              default:
                  std::cerr << "PLEASE USE A VALID LIGHT COORDINATE PHASE!" << std::endl;
                  throw std::runtime_error("PLEASE USE A VALID LIGHT COORDINATE PHASE!");
          }

          brain->setLightCoordinates(relative_coordinates);

          brain_.reset(brain);
      }
      else
      {
        std::cout << "Calling default ANN brain." << std::endl;
        revolve::gazebo::RobotController::LoadBrain(sdf);
      }
    } catch (std::exception &e)
    {
      // needed because otherwise the exception dies silently and debugging is a nightmare
      std::cerr
              << "################################################################################\n"
              << "Exception occurred while running RobotController::LoadBrain:\n"
              << "exception: "
              << e.what()
              << "\n################################################################################"
              << std::endl;
    }
  }

  void
  RobotController::DoUpdate(const gazebo::common::UpdateInfo info)
  {
    revolve::gazebo::RobotController::DoUpdate(info);
    evaluator_->updatePosition(this->model->GetRelativePose().Ign());
    auto pose = this->model->GetRelativePose().Ign();
    evaluator_->updatePosition(pose);

    if ("hyperneat::supg_phototaxis" == algorithm
        || "hyperneat::supg_phototaxis::replay" == algorithm)
    {
        SUPGBrainPhototaxis *supg_brain = reinterpret_cast<SUPGBrainPhototaxis *>(brain_.get());
        supg_brain->updateRobotPosition(pose);
        if (!supg_brain->getSphereModel())
        {
            gazebo::physics::ModelPtr sphere_model = world->GetModel("unique_sphere");
            supg_brain->addLightModel(sphere_model);
        }
    }
  }

// EVALUATOR CODE
  RobotController::Evaluator::Evaluator()
  {
    currentPosition_.Reset();
    previousPosition_.Reset();
  }

  void
  RobotController::Evaluator::start()
  {
    previousPosition_ = currentPosition_;
  }

  double
  RobotController::Evaluator::fitness()
  {
    double dS = sqrt(
            std::pow(previousPosition_.Pos().X() - currentPosition_.Pos().X(),
                     2) +
            std::pow(previousPosition_.Pos().Y() - currentPosition_.Pos().Y(),
                     2));
    return dS / 30.0; // dS / RLPower::FREQUENCY_RATE
  }

  void
  RobotController::Evaluator::updatePosition(const ignition::math::Pose3d pose)
  {
    currentPosition_ = pose;
  }


} /* namespace tol */
