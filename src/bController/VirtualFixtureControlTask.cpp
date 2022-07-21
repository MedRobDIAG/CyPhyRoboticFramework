// Project Header files
#include "VirtualFixtureControlTask.hpp"
#include "utils.hpp"

/**
* @brief Constructor of VirtualFixtureControlTask object with RobotInterface pointer argument
* @param tc: the task class
* @param td: the task dimension
* @param ri: the RobotInterface pointer argument
*
*/
VirtualFixtureControlTask::VirtualFixtureControlTask(const int& tc, const int& td, RobotInterface* ri) : ControlTask(tc, td, ri) {

	this->constraintSet = false;
	this->init();

	// Initialize variables
	this->Tbee0 = this->robot->getTbee();
	this->selectMatrix.setIdentity();

	// Initialize the task reference vector with respect to the current orientation
	// (Linear motion is equivalent to keep robot EE orientation fixed)
	this->updateTaskReferenceVector();

}

/**
* @brief Constructor of VirtualFixtureControlTask object with selection matrix and RobotInterface pointer arguments
* @param S: the selection matrix
* @param ri: the RobotInterface pointer argument
*
*/
VirtualFixtureControlTask::VirtualFixtureControlTask(const int& tc, const Eigen::Matrix6i& S, RobotInterface* ri) : ControlTask(tc, 0, ri) {

	// Useful variables 
	Eigen::Vector6i selectedDirs;
	std::vector < int > nzEntries;

	// Initialize constraintSet to false
	this->constraintSet = false;

	// Assign the Selection matrix
	this->setSelectionMatrix(S);

	// Assign as Task Dimension the number of 1s in the selection matrix diagonal
	this->taskDim = SPACE_DIM * 2 - this->selectMatrix.diagonal().sum();

	// Initialize the task variables
	this->init();

	// Initialize indexes vector
	selectedDirs = Eigen::Vector6i::Ones() - this->selectMatrix.diagonal();
	for (int i = 0; i < SPACE_DIM * 2; i++) {
		if (selectedDirs(i) > 0) {
			nzEntries.push_back(i);
		}
	}
	this->indexes.setZero(nzEntries.size()); // nzEntries.size() should be equivalent to this->taskDim
	std::memcpy(this->indexes.data(), nzEntries.data(), nzEntries.size() * sizeof(int));

	// Initialize the task reference vector with respect to the current orientation
	// (Linear motion is equivalent to keep robot EE orientation fixed)
	this->updateTaskReferenceVector();

}


/**
* @brief Default destroyer of ControlTask object
*
*/
VirtualFixtureControlTask::~VirtualFixtureControlTask() {}

/**
* @brief Update function
* Update the feed forward input vector
*/
void VirtualFixtureControlTask::updateFeedForwardInput() {}

/**
* @brief Update function
* Update the task vector
*/
void VirtualFixtureControlTask::updateTaskVector() {


	Eigen::Vector3f pbee; // Rotation matrix expressing robot ee wrt robot base
	Eigen::Matrix3f Rrbee; // Rotation matrix expressing robot ee wrt robot base
	Eigen::Vector3f angles; // 3D angles of the EE robot orientation (which representation?)
	Eigen::Vector6f r6;
	Eigen::Matrix4f Tbe, Tee0eet;

	// Get EE Transformation matrix
	Tbe = this->robot->getTbee();
	Tee0eet = Tbee0.inverse() * Tbe;

	// Get EE position
	//pbee = this->robot->getEEPosition();
	pbee = Tee0eet.block<3,1>(0,3);

	// Get Rotation matrices
	//Rrbee = this->robot->getEERotMat();
	Rrbee = Tee0eet.topLeftCorner(3,3);

	// Extract angles
	angles = rotmat2rpy(Rrbee); 

	// Set the full task vector
	r6.topRows(SPACE_DIM) = pbee;
	r6.bottomRows(SPACE_DIM) = angles;

	// Update the Task vector
	for (int i = 0; i < this->taskDim; i++) {
		this->r(i) = r6(indexes(i));
	}
}

/**
* @brief Update function
* Update the task reference vector
*/
void VirtualFixtureControlTask::updateTaskReferenceVector() {

	if (!this->constraintSet) {

		Eigen::Vector6f r6;

		// Fill the full 6D task reference vector, from which we later extract required directions
		//r6.topRows(SPACE_DIM) = robot->getEEPosition();
		//r6.bottomRows(SPACE_DIM) = rotmat2rpy(robot->getEERotMat());
		r6.setZero(this->taskDim);

		// Update the Task Reference vector
		for (int i = 0; i < this->taskDim; i++) {
			this->rd(i) = r6(indexes(i));
		}

		// Set the initial Transformation matrix
		this->Tbee0 = robot->getTbee();

		// Set the flag of the constraint as set
		this->constraintSet = true;
	}
}

/**
* @brief Update function
* Update the task Jacobian matrix
*/
void VirtualFixtureControlTask::updateTaskJacobianMatrix() {
	

	Eigen::MatrixXf Jbe, J6;
	Eigen::Matrix3f Trpy, Tinv, Ree0b;
	Eigen::Vector3f rpy;
	int jointNum;

	// Initialize variables
	jointNum = this->robot->getJointNum();
	Jbe.setZero(this->taskDim, jointNum);
	J6.setZero(SPACE_DIM * 2, jointNum);

	// Get Jacobian
	Jbe = robot->getJacobian();

	// Get Roation matrix
	Ree0b = (this->Tbee0.topLeftCorner(3, 3)).transpose();

	// Fill the RPY transformation matrix
	rpy = rotmat2rpy(this->robot->getEERotMat());
	Trpy << cos(rpy(1))*cos(rpy(2)), -sin(rpy(2)), 0,
		cos(rpy(1))*sin(rpy(2)), cos(rpy(2)), 0,
		-sin(rpy(1)), 0, 1;

	Tinv = Trpy.inverse();

	// Set Jacobian task
	J6.topRows(SPACE_DIM) = Ree0b * Jbe.topRows(SPACE_DIM);
	J6.bottomRows(SPACE_DIM) =  Tinv * Ree0b * Jbe.bottomRows(SPACE_DIM);

	// Update the Task Jacobian matrix
	for (int i = 0; i < this->taskDim; i++) {
		this->J.row(i) = J6.row(indexes(i));
	}
	
}
