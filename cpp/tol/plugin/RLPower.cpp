//
// Created by Milan Jelisavcic on 28/03/16.
//

#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"

#include "Helper.h"
#include "RLPower.h"

namespace tol {

RLPower::RLPower(std::string modelName,
                 sdf::ElementPtr brain,
                 EvaluatorPtr evaluator,
                 std::vector<revolve::gazebo::MotorPtr> &actuators,
                 std::vector<revolve::gazebo::SensorPtr> &sensors) :
        revolve::brain::RLPower(
                modelName,
                parseSDF(brain),
                evaluator,
                actuators.size(),
                sensors.size()
        )
{}

RLPower::~RLPower()
{}

void
RLPower::update(const std::vector<revolve::gazebo::MotorPtr> &actuators,
                const std::vector<revolve::gazebo::SensorPtr> &sensors,
                double t,
                double step)
{
  revolve::brain::RLPower::update(
          Helper::createWrapper(actuators),
          Helper::createWrapper(sensors),
          t,
          step
  );
}

RLPower::Config
RLPower::parseSDF(sdf::ElementPtr brain)
{
  Config config;
//        // Create transport node
//        node_.reset(new ::gazebo::transport::Node());
//        node_->Init();
//
//        // Listen to network modification requests
//        alterSub_ = node_->Subscribe("~/" + modelName + "/modify_neural_network",
//                                     &RLPower::modify, this);

  // Read out brain configuration attributes
  config.algorithm_type = brain->HasAttribute("type") ? brain->GetAttribute("type")->GetAsString() : "A";

  config.evaluation_rate = brain->HasAttribute("evaluation_rate") ?
                           std::stod(brain->GetAttribute("evaluation_rate")->GetAsString()) :
                           RLPower::EVALUATION_RATE;
  config.interpolation_spline_size = brain->HasAttribute("interpolation_spline_size") ?
                                     std::stoul(brain->GetAttribute("interpolation_spline_size")->GetAsString()) :
                                     RLPower::INTERPOLATION_CACHE_SIZE;
  config.max_evaluations = brain->HasAttribute("max_evaluations") ?
                           std::stoul(brain->GetAttribute("max_evaluations")->GetAsString()) :
                           RLPower::MAX_EVALUATIONS;
  config.max_ranked_policies = brain->HasAttribute("max_ranked_policies") ?
                               std::stoul(brain->GetAttribute("max_ranked_policies")->GetAsString()) :
                               RLPower::MAX_RANKED_POLICIES;
  config.noise_sigma = brain->HasAttribute("init_sigma") ?
                       std::stod(brain->GetAttribute("init_sigma")->GetAsString()) :
                       RLPower::SIGMA_START_VALUE;
  config.sigma_tau_correction = brain->HasAttribute("sigma_tau_correction") ?
                                std::stod(brain->GetAttribute("sigma_tau_correction")->GetAsString()) :
                                RLPower::SIGMA_TAU_CORRECTION;
  config.source_y_size = brain->HasAttribute("init_spline_size") ?
                         std::stoul(brain->GetAttribute("init_spline_size")->GetAsString()) :
                         RLPower::INITIAL_SPLINE_SIZE;
  config.update_step = brain->HasAttribute("update_step") ?
                       std::stoul(brain->GetAttribute("update_step")->GetAsString()) :
                       RLPower::UPDATE_STEP;
  config.policy_load_path = brain->HasAttribute("policy_load_path") ?
                            brain->GetAttribute("policy_load_path")->GetAsString() : "";

  return config;
}

} /* namespace tol */
