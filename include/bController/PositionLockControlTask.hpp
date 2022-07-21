#ifndef POSITION_LOCK_CONTROL_TASK_HPP_
#define POSITION_LOCK_CONTROL_TASK_HPP_

// Project Header files
#include "ControlTask.hpp"

class PositionLockControlTask : public ControlTask {

public:

	/**
	* @brief Constructor of PositionLockControlTask object with RobotInterface pointer argument
	* @param tc: the task class
	* @param td: the task dimension
	* @param ri: the RobotInterface pointer argument
	*
	*/
	PositionLockControlTask(const int& tc, const int& td, RobotInterface* ri = nullptr);

	/**
	* @brief Default destroyer of ControlTask object
	*
	*/
	~PositionLockControlTask();

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

	/**
	* @brief Reset function
	* Reset the constraint flag
	*/
	inline void resetReference() { this->constraintSet = false; }

private:

	bool constraintSet;	//!< Flag stating if the constraint has been set

};

#endif // POSITION_LOCK_CONTROL_TASK_HPP_
