// Project Header files
#include "TeleopControlTask.hpp"

/**
* @brief Constructor of ControlTask object with RobotInterface pointer argument
* @param td: the task dimension
* @param ri: the RobotInterface pointer argument
* @param h: the Geomagic pointer argument
*
*/
TeleopControlTask::TeleopControlTask(const int& tc, const int& td, RobotInterface* ri, Geomagic* h) : ControlTask(tc, td, ri) {

	this->teleopDev = h;
	this->init();

}

/**
* @brief Default destroyer of ControlTask object
*
*/
TeleopControlTask::~TeleopControlTask() {}

/**
* @brief Update function
* Update the feed forward input vector
*/
void TeleopControlTask::updateFeedForwardInput() {

	Eigen::Vector6f vHaptic; // 6D velocity vector in the haptic frame
	Eigen::Matrix3f Rhr; // Rotation matrix expressing robot wrt haptic device
	Eigen::Vector6f vRobot; // 6D velocity vector in the robot frame
	Eigen::Vector6f vRobotEE; // 6D constrained velocity vector in the robot frame
	Eigen::Matrix3f Rrbee; // Rotation matrix expressing robot ee wrt robot base
	float lvelScale, avelScale, masterForceScale; // Velocity and force scale factors
	int jointNum = robot->getJointNum();

	// Get the velocity of the HIP in the Haptic device frame
	vHaptic = this->teleopDev->getHIPVel();
			 
	// Get scale factors
	lvelScale = this->teleopDev->getLinVelScale();
	avelScale = this->teleopDev->getAngVelScale();

	// Get Rotation matrices
	Rhr = this->teleopDev->getRms();
	Rrbee = this->robot->getEERotMat();

	// Rotate the linear velocity (angular optionally) in the robot frame
	vRobot.topRows(SPACE_DIM) = (Rhr.transpose()) * vHaptic.topRows(SPACE_DIM);
	vRobot.bottomRows(SPACE_DIM) = vHaptic.bottomRows(SPACE_DIM); // pre-multiply by Rhr^T if you want to rotate angular velocity as well

	// Scale the velocity
	vRobot.topRows(SPACE_DIM) *= lvelScale;
	vRobot.bottomRows(SPACE_DIM) *= -avelScale;

	// Rotate the reference velocity from base frame to end-effector frame
	vRobotEE.topRows(SPACE_DIM) = Rrbee.transpose() * vRobot.topRows(SPACE_DIM);
	vRobotEE.bottomRows(SPACE_DIM) = Rrbee.transpose() * vRobot.bottomRows(SPACE_DIM);

	// Set the feedforward control input
	if (this->teleopDev->isClutchActive()) {
		this->uff = vRobotEE;
	}
	else {
		this->uff.setZero(this->taskDim);
	}
	
}

/**
* @brief Update function
* Update the task vector
*/
void TeleopControlTask::updateTaskVector() {

	this->r.setZero();

}

/**
* @brief Update function
* Update the task reference vector
*/
void TeleopControlTask::updateTaskReferenceVector() {

	this->rd.setZero();

}

/**
* @brief Update function
* Update the task Jacobian matrix
*/
void TeleopControlTask::updateTaskJacobianMatrix() {

	Eigen::MatrixXf Jee, Jbe; 
	Eigen::Matrix3f Rrbee;
	int jointNum;

	// Initialize variables
	jointNum = this->robot->getJointNum(); 
	Jbe.setZero(this->taskDim, jointNum);
	Jee.setZero(this->taskDim, jointNum);
	Rrbee = this->robot->getEERotMat();

	// Get Jacobian
	Jbe = robot->getJacobian();
	Jee.topRows(SPACE_DIM) = Rrbee.transpose() * Jbe.topRows(SPACE_DIM);
	Jee.bottomRows(SPACE_DIM) = Rrbee.transpose() * Jbe.bottomRows(SPACE_DIM);

	// Set Jacobian Task
	this->J = Jee;

}


