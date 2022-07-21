#ifndef CONTROL_STRATEGY_HPP_
#define CONTROL_STRATEGY_HPP_

// Eigen Header files
#include <Eigen/Dense>

// Project Header files
#include "RobotInterface.hpp"

class ControlStrategy{

public:

	/** PLEASE READ THIS CAREFULLY **/
	/* When we define a class containing fixed-size Eigen structures,
	* and we later instantiate such class dynamically, it occurs an issue
	* that may result in undesired and unexpected program crash if not
	* properly handled, due to ACCESS VIOLATIONS.
	* To prevent this, every class containing fixed-size Eigen variables
	* must add the line below in the public part of the class definition,
	* in order to overload the new operator and explicitly handle alignment
	* problems in the variable definitions. If you have a polymorphic structure
	* among your class, place this macro in the Base class, like here.
	*/
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/*
	* @brief Constructor of ControlStrategy
	* @param r: the pointer to RobotInterface
	*/
	ControlStrategy(RobotInterface* r = nullptr) : robot(r) {}

	/*
	* @brief Destroyer of ControlStrategy
	*/
	~ControlStrategy() {}

	/**
	* @brief Init function
	* Initialize the control strategy
	*/
	virtual void init() = 0;

	/**
	* @brief Main function
	* Implements the control law associated to the ControlStrategy
	*/
	virtual Eigen::VectorXf controlLaw() = 0;

	/**
	* @brief Set function
	* Set the RobotInterface instance
	* Note that ControlStrategy class acts as a Context class for the RobotInterface
	* object, that is its abstract strategy class
	* @param r: the RobotInterface pointer
	*/
	inline void setRobotInstance(RobotInterface* r) { this->robot = r; }

	/**
	* @brief Set function
	* Set the kinematic constraint matrix
	* @param cMat: the kinematic constraint matrix
	*/
	inline void setKinematicConstraint(const Eigen::Matrix6f& cMat) { 
		this->constraintMat = cMat; 
	}

protected:

	RobotInterface* robot;			//!< Pointer to a RobotInterface object
	Eigen::Matrix6f constraintMat;	//!< Kinematic constraint matrix
};



#endif // CONTROL_STRATEGY_HPP_
