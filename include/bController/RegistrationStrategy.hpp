#ifndef REGISTRATION_STRATEGY_HPP_
#define REGISTRATION_STRATEGY_HPP_

// Project Header files
#include "TaskStrategy.hpp"
#include "VREPProxy.hpp"
#include "AbdomenPhantom.hpp"

// Eigen Header files
#include <Eigen\Dense>

#define MAX_REGISTRATION_POINTS_NUM 4

class RegistrationStrategy : public TaskStrategy {


public:

	/*
	* @brief Constructor of RegistrationStrategy
	* @param p: the pointer to Phantom
	* @param v: the pointer to VREPProxy
	* @param r: the pointer to RobotInterface
	*/
	RegistrationStrategy(AbdomenPhantom* p, VREPProxy* v = nullptr, RobotInterface* r = nullptr);

	/*
	* @brief Destroyer of RegistrationStrategy
	*/
	~RegistrationStrategy();

	/**
	* @brief Init function
	* Initialize the TaskStrategy class
	*/
	void init();

	/**
	* @brief Main function
	* Implements the task of the RegistrationStrategy
	* @return the return state of the routine
	*/
	int execTask();

	/**
	* @brief Terminate function
	* Terminate the RegistrationStrategy class
	*/
	void terminate();

	/**
	* @brief Check function
	* Check if all the points for the registration task have been acquired
	* @return if all the points for the registration task have been acquired, false otherwise
	*/
	inline bool areRegPointsAcquired() { return (this->regPointsNum) >= MAX_REGISTRATION_POINTS_NUM; }

	/** +++ OBSOLETE +++ **/
	/**
	* @brief Set function
	* Set the pose of the phantom in the V-REP simulator
	* @param T: the input transformation defining the pose of the phantom
	* @param simRunning: if the set has to be done while the simulation in running (default is false)
	*/
	//void setPhantomPoseInSimulator(const Eigen::Matrix4d& T, const bool& simRunning = false);

private:


	int regPointsNum;				//!< Points acquired for registration
	Eigen::MatrixXd robotPoints;	//!< Set of points acquired in the robot frame
	
	/* Entities */
	AbdomenPhantom* phantom;				//!< Pointer to Phantom object
};


#endif // REGISTRATION_STRATEGY_HPP_
