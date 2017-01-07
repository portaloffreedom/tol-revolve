#include "hyper_ext_nn.h"
#include "helper.h"
#include "sensor.h"
#include "actuator.h"
#include "body.h"
#include "brain/conversion.h"
#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"

#include <fstream>


namespace tol {


    HyperExtNN::HyperExtNN(std::string modelName,
			   sdf::ElementPtr brain,
		     tol::EvaluatorPtr evaluator,
                     const std::vector<revolve::gazebo::MotorPtr> &actuators,
                     const std::vector<revolve::gazebo::SensorPtr> &sensors) 
    :  revolve::brain::ConvSplitBrain<boost::shared_ptr<revolve::brain::ExtNNConfig>, CPPNEAT::GeneticEncodingPtr>(&revolve::brain::convertForExtNNFromHyper, &revolve::brain::convertForHyperFromExtNN, modelName) {
//  	sleep(20);
	    
	//initialise controller
	std::string name(modelName.substr(0, modelName.find("-")) + ".yaml");
	Body body(name);
	std::pair<std::map<int, unsigned int>, std::map<int, unsigned int>> in_out = body.get_input_output_map(actuators, sensors);
	revolve::brain::input_map = in_out.first;
	revolve::brain::output_map = in_out.second;
	revolve::brain::cpg_network = revolve::brain::convertForController(body.get_coupled_cpg_network());
	revolve::brain::neuron_coordinates = body.get_id_to_coordinate_map();
	controller = boost::shared_ptr<revolve::brain::ExtNNController1> (new revolve::brain::ExtNNController1(modelName,
														revolve::brain::cpg_network,  
														Helper::createWrapper(actuators),
														Helper::createWrapper(sensors)));
	
	//initialise learner
	CPPNEAT::Learner::LearningConfiguration learn_conf = parseLearningSDF(brain);
	revolve::brain::set_brain_spec(true);
	learn_conf.start_from = body.get_hyper_neat_network();
	CPPNEAT::MutatorPtr mutator(new CPPNEAT::Mutator(revolve::brain::brain_spec,
					0.8,
					learn_conf.start_from->min_max_innov_numer().second,
					100,
					std::vector<CPPNEAT::Neuron::Ntype>(),
					true));
	learner = boost::shared_ptr<CPPNEAT::Learner>(new CPPNEAT::Learner(mutator, 
									   learn_conf));
	evaluator_ = evaluator;
    }

    HyperExtNN::~HyperExtNN()
    {

    }


    void HyperExtNN::update(const std::vector<revolve::gazebo::MotorPtr> &actuators,
                         const std::vector<revolve::gazebo::SensorPtr> &sensors,
                         double t,
                         double step) {
// 	std::cout << "yay" << std::endl;
        revolve::brain::ConvSplitBrain<boost::shared_ptr<revolve::brain::ExtNNConfig>, CPPNEAT::GeneticEncodingPtr>::update(
                Helper::createWrapper(actuators),
                Helper::createWrapper(sensors),
                t, step
        );
    }
    CPPNEAT::Learner::LearningConfiguration HyperExtNN::parseLearningSDF(sdf::ElementPtr brain) {
        CPPNEAT::Learner::LearningConfiguration config;

        // Read out brain configuration attributes
        config.asexual = brain->HasAttribute("asexual") ?
                                 (brain->GetAttribute("asexual")->GetAsString() == "true") :
                                 CPPNEAT::Learner::ASEXUAL;
        config.pop_size = brain->HasAttribute("pop_size") ?
                                 std::stoi(brain->GetAttribute("pop_size")->GetAsString()) :
                                 CPPNEAT::Learner::POP_SIZE;
        config.tournament_size = brain->HasAttribute("tournament_size") ?
                                 std::stoi(brain->GetAttribute("tournament_size")->GetAsString()) :
                                 CPPNEAT::Learner::TOURNAMENT_SIZE;			 
        config.num_children = brain->HasAttribute("num_children") ?
                                 std::stoi(brain->GetAttribute("num_children")->GetAsString()) :
                                 CPPNEAT::Learner::NUM_CHILDREN;
        config.weight_mutation_probability = brain->HasAttribute("weight_mutation_probability") ?
                                 std::stod(brain->GetAttribute("weight_mutation_probability")->GetAsString()) :
                                 CPPNEAT::Learner::WEIGHT_MUTATION_PROBABILITY;
        config.weight_mutation_sigma = brain->HasAttribute("weight_mutation_sigma") ?
                                 std::stod(brain->GetAttribute("weight_mutation_sigma")->GetAsString()) :
                                 CPPNEAT::Learner::WEIGHT_MUTATION_SIGMA;
        config.param_mutation_probability = brain->HasAttribute("param_mutation_probability") ?
                                 std::stod(brain->GetAttribute("param_mutation_probability")->GetAsString()) :
                                 CPPNEAT::Learner::PARAM_MUTATION_PROBABILITY;
        config.param_mutation_sigma = brain->HasAttribute("param_mutation_sigma") ?
                                 std::stod(brain->GetAttribute("param_mutation_sigma")->GetAsString()) :
                                 CPPNEAT::Learner::PARAM_MUTATION_SIGMA;
        config.max_generations = brain->HasAttribute("max_generations") ?
                                 std::stoi(brain->GetAttribute("max_generations")->GetAsString()) :
                                 CPPNEAT::Learner::MAX_GENERATIONS;
        config.speciation_threshold = brain->HasAttribute("speciation_threshold") ?
                                 std::stod(brain->GetAttribute("speciation_threshold")->GetAsString()) :
                                 CPPNEAT::Learner::SPECIATION_TRESHOLD;
        config.repeat_evaluations = brain->HasAttribute("repeat_evaluations") ?
                                 std::stoi(brain->GetAttribute("repeat_evaluations")->GetAsString()) :
                                 CPPNEAT::Learner::REPEAT_EVALUATIONS;
        config.initial_structural_mutations = brain->HasAttribute("initial_structural_mutations") ?
                                 std::stoi(brain->GetAttribute("initial_structural_mutations")->GetAsString()) :
                                 CPPNEAT::Learner::INITIAL_STRUCTURAL_MUTATIONS;
	config.interspecies_mate_probability = brain->HasAttribute("interspecies_mate_probability") ?
				 std::stod(brain->GetAttribute("interspecies_mate_probability")->GetAsString()) :
				 CPPNEAT::Learner::INTERSPECIES_MATE_PROBABILITY;
        return config;
    }

} /* namespace tol */

