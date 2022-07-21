#ifndef TELEOP_CONTROL_TASK_HPP_
#define TELEOP_CONTROL_TASK_HPP_

// Project Header files
#include "ControlTask.hpp"
#include "Geomagic.hpp"

class TeleopControlTask : public ControlTask {

public:

	/**
	* @brief Constructor of ControlTask object with RobotInterface pointer argument
	* @param tc: the task class
	* @param td: the task dimension
	* @param ri: the RobotInterface pointer argument
	* @param h: the Geomagic pointer argument
	*
	*/
	TeleopControlTask(const int& tc, const int& td, RobotInterface* ri = nullptr, Geomagic* h = nullptr);

	/**
	* @brief Default destroyer of ControlTask object
	*
	*/
	~TeleopControlTask();

	/**
	* @brief Update function
	* Update the feed forward input vector
	*/
	void updateFeedForwardInput();

	/**
	* @brief Update function
	* Update the task vector
	*/
	void updateTaskVector();

	/**
	* @brief Update function
	* Update the task reference vector
	*/
	void updateTaskReferenceVector();

	/**
	* @brief Update function
	* Update the task Jacobian matrix
	*/
	void updateTaskJacobianMatrix();


private:

	Geomagic* teleopDev;	//!< Pointer to the haptic device
};

#endif // TELEOP_CONTROL_TASK_HPP_
