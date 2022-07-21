#ifndef MANUAL_GUIDANCE_STRATEGY_HPP_
#define MANUAL_GUIDANCE_STRATEGY_HPP_

// Project Header files
#include "ControlStrategy.hpp"
#include "Trajectory.hpp"

class ManualGuidanceStrategy : public ControlStrategy {

public:

	/*
	* @brief Constructor of ControlStrategy
	* @param r: the pointer to RobotInterface
	*/
	ManualGuidanceStrategy(RobotInterface* r = nullptr) : ControlStrategy(r) {}

	/*
	* @brief Destroyer of ControlStrategy
	*/
	~ManualGuidanceStrategy() {}

	/**
	* @brief Init function
	* Initialize the control strategy
	*/
	void init();

	/**
	* @brief Main function
	* Implements the control law associated to the ControlStrategy
	*/
	Eigen::VectorXf controlLaw();

private:



};


#endif // MANUAL_GUIDANCE_STRATEGY_HPP_