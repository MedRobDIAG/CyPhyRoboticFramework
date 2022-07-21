#ifndef TASK_STRATEGY_HPP_
#define TASK_STRATEGY_HPP_

// Eigen Header files
#include <Eigen/Dense>

// Project Header files
#include "RobotInterface.hpp"
#include "SystemState.hpp"
#include "VREPProxy.hpp"

enum TASK_RETURN_CODE {PROCESSING, FINISHED};

class TaskStrategy {

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/*
	* @brief Constructor of ControlStrategy
	* @param r: the pointer to RobotInterface
	*/
	TaskStrategy(VREPProxy* v, RobotInterface* r = nullptr) : vrep(v), robot(r) {
	
		this->simPort = 19995; // default value
		this->time_ = 0.0;
		
	}

	/*
	* @brief Destroyer of ControlStrategy
	*/
	~TaskStrategy() {}

	/**
	* @brief Init function
	* Initialize the TaskStrategy class
	*/
	virtual void init() = 0;

	/**
	* @brief Main function
	* Implements the control law associated to the ControlStrategy
	* @return the return state of the routine
	*/
	virtual int execTask() = 0;

	/**
	* @brief Terminate function
	* Terminate the TaskStrategy class
	*/
	virtual void terminate() = 0;

	/**
	* @brief Set function
	* Set the RobotInterface instance
	* @param r: the RobotInterface pointer
	*/
	inline void setRobotInstance(RobotInterface* r) {
		this->robot = r;
	}

	/**
	* @brief Set function
	* Set the VREPProxy instance
	* @param v: the VREPProxy pointer
	*/
	inline void setSimulatorInstance(VREPProxy* v) {
		this->vrep = v;
	}

	/**
	* @brief Set function
	* Set the simulation port
	* @param sp: simulation port
	*/
	inline void setSimPort(const int& sp) { this->simPort = sp; }

	/**
	* @brief Set function
	* Set the timestamp of the TaskStrategy object
	* @param t_: the timestamp
	*/
	inline void setTimeStamp(const double& t) { this->time_ = t; }

	/**
	* @brief Update function
	* Update the timestamp with the input sample time dt
	* @param dt: the sample time
	*/
	inline void updateTimeStamp(const double& dt) { this->time_ += dt; }

protected:

	/* Proxies */
	VREPProxy* vrep;				//!< Pointer to a VREPProxy object

	/* Entities */
	RobotInterface* robot;			//!< Pointer to a RobotInterface object

	int simPort;					//!< Port number to communicate with simulator
	double time_;					//!< Time variable
};



#endif // TASK_STRATEGY_HPP_
