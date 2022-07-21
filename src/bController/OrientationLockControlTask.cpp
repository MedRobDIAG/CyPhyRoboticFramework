// Project Header files
#include "OrientationLockControlTask.hpp"
#include "utils.hpp"

/**
* @brief Constructor of OrientationLockControlTask object with RobotInterface pointer argument
* @param tc: the task class
* @param td: the task dimension
* @param ri: the RobotInterface pointer argument
*
*/
OrientationLockControlTask::OrientationLockControlTask(const int& tc, const int& td, RobotInterface* ri) : ControlTask(tc, td, ri) {

	this->constraintSet = false;

	// Initialize the task reference vector with respect to the current orientation
	// (Linear motion is equivalent to keep robot EE orientation fixed)
	this->updateTaskReferenceVector();

}

/**
* @brief Default destroyer of ControlTask object
*
*/
OrientationLockControlTask::~OrientationLockControlTask() {}

/**
* @brief Update function
* Update the feed forward input vector
*/
void OrientationLockControlTask::updateFeedForwardInput() {}

/**
* @brief Update function
* Update the task vector
*/
void OrientationLockControlTask::updateTaskVector() {

	Eigen::Matrix3f Rrbee; // Rotation matrix expressing robot ee wrt robot base
	Eigen::Vector3f angles; // 3D angles of the EE robot orientation (which representation?)

	// Get Rotation matrices
	Rrbee = this->robot->getEERotMat();

	// Extract angles
	angles = rotmat2rpy(Rrbee); 

	// Set the task vector
	this->r = angles;

}

/**
* @brief Update function
* Update the task reference vector
*/
void OrientationLockControlTask::updateTaskReferenceVector() {

	Eigen::Matrix3f Rrbee; // Rotation matrix expressing robot ee wrt robot base
	Eigen::Vector3f angles; // 3D angles of the EE robot orientation (which representation?)

	if (!this->constraintSet) {
		// Get Rotation matrices
		Rrbee = this->robot->getEERotMat();

		// Extract angles
		angles = rotmat2rpy(Rrbee);

		//angles = Eigen::Vector3f::Ones()*0.3 + angles;

		// Set the task reference vector
		this->rd = angles;
		std::cout << "Desired RPY angles = " << angles.transpose() << std::endl;

		// Set the flag of the constraint as set
		this->constraintSet = true;
	}

}

/**
* @brief Update function
* Update the task Jacobian matrix
*/
void OrientationLockControlTask::updateTaskJacobianMatrix() {

	Eigen::MatrixXf Jbe;
	Eigen::Matrix3f Trpy, Tinv;
	Eigen::Vector3f rpy;
	int jointNum;

	// Initialize variables
	jointNum = this->robot->getJointNum();
	Jbe.setZero(this->taskDim, jointNum);

	// Get Jacobian
	Jbe = robot->getJacobian();

	// Fill the RPY transformation matrix
	rpy = rotmat2rpy(this->robot->getEERotMat());
	Trpy << cos(rpy(1))*cos(rpy(2)), -sin(rpy(2)), 0,
			cos(rpy(1))*sin(rpy(2)),  cos(rpy(2)), 0,
			           -sin(rpy(1)),            0, 1;
	
	Tinv = Trpy.inverse();

	// Set Jacobian task
	this->J = Tinv * Jbe.bottomRows(SPACE_DIM); // Angular Jacobian



}
