// Project Header files
#include "PositionLockControlTask.hpp"
#include "utils.hpp"

/**
* @brief Constructor of PositionLockControlTask object with RobotInterface pointer argument
* @param tc: the task class
* @param td: the task dimension
* @param ri: the RobotInterface pointer argument
*
*/
PositionLockControlTask::PositionLockControlTask(const int& tc, const int& td, RobotInterface* ri) : ControlTask(tc, td, ri) {

	this->constraintSet = false;

	// Initialize the task reference vector with respect to the current orientation
	// (Linear motion is equivalent to keep robot EE orientation fixed)
	this->updateTaskReferenceVector();

}

/**
* @brief Default destroyer of ControlTask object
*
*/
PositionLockControlTask::~PositionLockControlTask() {}

/**
* @brief Update function
* Update the feed forward input vector
*/
void PositionLockControlTask::updateFeedForwardInput() {}

/**
* @brief Update function
* Update the task vector
*/
void PositionLockControlTask::updateTaskVector() {

	Eigen::Vector3f pbee; // Rotation matrix expressing robot ee wrt robot base

	// Get EE position
	pbee = this->robot->getEEPosition();

	// Set the task vector
	this->r = pbee;

}

/**
* @brief Update function
* Update the task reference vector
*/
void PositionLockControlTask::updateTaskReferenceVector() {

	Eigen::Vector3f pbee; // Rotation matrix expressing robot ee wrt robot base

	if (!this->constraintSet) {

		// Get EE position
		pbee = this->robot->getEEPosition();

		// Set the task vector
		this->rd = pbee;

		// Set the flag of the constraint as set
		this->constraintSet = true;
	}

}

/**
* @brief Update function
* Update the task Jacobian matrix
*/
void PositionLockControlTask::updateTaskJacobianMatrix() {

	Eigen::MatrixXf Jbe;
	int jointNum;

	// Initialize variables
	jointNum = this->robot->getJointNum();
	Jbe.setZero(this->taskDim, jointNum);

	// Get Jacobian
	Jbe = robot->getJacobian();

	// Set Jacobian task
	this->J = Jbe.topRows(SPACE_DIM); // Linear Jacobian



}
