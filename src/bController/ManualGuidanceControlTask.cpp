// Project Header files
#include "ManualGuidanceControlTask.hpp"


/**
* @brief Constructor of ManualGuidanceControlTask object with RobotInterface pointer argument
* @param tc: the task class
* @param td: the task dimension
* @param ri: the RobotInterface pointer argument
*
*/
ManualGuidanceControlTask::ManualGuidanceControlTask(const int& tc, const int& td, RobotInterface* ri) : ControlTask(tc,td,ri) {

	this->init();

	this->withLPFilter = false;
	this->resThresh = 0.2;
	this->residual.setZero(ri->getJointNum());
}

/**
* @brief Default destroyer of AutonomousControlTask object
*
*/
ManualGuidanceControlTask::~ManualGuidanceControlTask() {}

/**
* @brief Update function
* Update the feed forward input vector
*/
void ManualGuidanceControlTask::updateFeedForwardInput() {

	this->uff.setZero(this->taskDim);

}

/**
* @brief Update function
* Update the task vector
*/
void ManualGuidanceControlTask::updateTaskVector(){

	// Get robot residual
	Eigen::VectorXf res = this->robot->getResidualVector();

	// Set task vector
	for (int i = 0; i < res.size(); i++) {

		if (this->withLPFilter) {
		
			// Filter the residual vector
			this->residual(i) = low_pass_filter(res(i), this->residual(i), this->sampleTime, 1.0 / this->fcut);

		}

		// Evaluate the joint velocity control inputs
		double opposite_sign = (res(i) > 0) ? 1.0 : -1.0;
		if (std::fabs(res(i)) > this->resThresh) {
			this->r(i) = - (std::fabs(res(i)) - this->resThresh) * opposite_sign;
		}

	}
}

/**
* @brief Update function
* Update the task reference vector
*/
void ManualGuidanceControlTask::updateTaskReferenceVector() {

	this->rd.setZero(this->taskDim);

}

/**
* @brief Update function
* Update the task Jacobian matrix
*/
void ManualGuidanceControlTask::updateTaskJacobianMatrix() {

	Eigen::MatrixXf In;
	int n = robot->getJointNum();

	In.setIdentity(n, n);

	this->J = In;

	/*Eigen::MatrixXf Jr, JT;

	// Get Robot Jacobian matrix
	Jr = this->robot->getJacobian();
	JT = Jr.transpose();

	// Set Task Jacobian
	this->J = JT;//*/

}