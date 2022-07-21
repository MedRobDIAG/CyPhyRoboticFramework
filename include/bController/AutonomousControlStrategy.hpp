#ifndef AUTONOMOUS_CONTROL_STRATEGY_HPP_
#define AUTONOMOUS_CONTROL_STRATEGY_HPP_

// Project Header files
#include "ControlStrategy.hpp"
#include "Trajectory.hpp"
#include "SinTrajectory.hpp"

class AutonomousControlStrategy : public ControlStrategy {

public:

	/*
	* @brief Constructor of ControlStrategy
	* @param r: the pointer to RobotInterface
	*/
	AutonomousControlStrategy(RobotInterface* r = nullptr, Trajectory* t = nullptr) : ControlStrategy(r), traj(t) {
	
		this->Tbee_0.setIdentity();

	}

	/*
	* @brief Destroyer of ControlStrategy
	*/
	~AutonomousControlStrategy() {}

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

	/**
	* @brief Set function
	* Set the trajectory object
	* @param t: the pointer to the Trajectory object to be set
	*/
	inline void setTrajectory(Trajectory* t) {
		delete this->traj;
		this->traj = t;
	}

private:

	Trajectory* traj;					//<! Trajectory object
	Eigen::Matrix4f Tbee_0;				//!< Pose of the end-effector in the base frame at time 0

};


#endif // AUTONOMOUS_CONTROL_STRATEGY_HPP_