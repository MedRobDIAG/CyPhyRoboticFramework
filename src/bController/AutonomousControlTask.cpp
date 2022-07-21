// Project Header files
#include "AutonomousControlTask.hpp"

/**
* @brief Constructor of AutonomousControlTask object with RobotInterface pointer argument
* @param tc: the task class
* @param td: the task dimension
* @param ri: the RobotInterface pointer argument
* @param t: the Trajectory pointer argument
*
*/
AutonomousControlTask::AutonomousControlTask(const int& tc, const int& td, RobotInterface* ri, Trajectory* t ) : ControlTask(tc, td, ri), traj(t) {

	this->init();

	// Initialize variables
	this->Tbee0 = robot->getTbee();
	this->traj->reset();
}

/**
* @brief Default destroyer of AutonomousControlTask object
*
*/
AutonomousControlTask::~AutonomousControlTask() {}

/**
* @brief Update function
* Update the feed forward input vector
*/
void AutonomousControlTask::updateFeedForwardInput() {

	Eigen::Vector6f ref;
	Eigen::Vector3f vbee_t, vd;
	Eigen::Matrix3f Rbee_0;
	
	// Get the initial position and rotation matrix of the ee wrt the base frame
	Rbee_0 = this->Tbee0.topLeftCorner(SPACE_DIM, SPACE_DIM);

	if (!this->traj->isTrajectoryDone()) {
		
		// Get the current trajectory derivative sample
		this->traj->updateTime();

		ref = this->traj->eval();
		vd = ref.bottomRows(SPACE_DIM);

		// Convert data in the base frame
		vbee_t = Rbee_0 * vd;

		// Build the feed-foward input
		this->uff.topRows(SPACE_DIM) = vbee_t;
		this->uff.bottomRows(SPACE_DIM).setZero(); //<--- To be filled
	}



}

/**
* @brief Update function
* Update the task vector
*/
void AutonomousControlTask::updateTaskVector() {

	Eigen::Vector3f pb;


	if (!this->traj->isTrajectoryDone()) {

		// Get the current end-effector position
		pb = this->robot->getEEPosition();

		// Update the Task vector
		this->r.topRows(SPACE_DIM) = pb;
		this->r.bottomRows(SPACE_DIM).setZero();

	}

}

/**
* @brief Update function
* Update the task reference vector
*/
void AutonomousControlTask::updateTaskReferenceVector() {

	Eigen::Vector6f ref;
	Eigen::Vector3f pbee_0, pbee_t, pd;
	Eigen::Matrix3f Rbee_0;

	// Get the initial position and rotation matrix of the ee wrt the base frame
	Rbee_0 = this->Tbee0.topLeftCorner(SPACE_DIM, SPACE_DIM);
	pbee_0 = this->Tbee0.block<3, 1>(0, 3);

	if (!this->traj->isTrajectoryDone()) {

		// Get the current trajectory derivative sample
		this->traj->updateTime();
		ref = this->traj->eval();
		pd = ref.topRows(SPACE_DIM);

		// Convert data in the base frame
		pbee_t = Rbee_0 * pd + pbee_0;		
		this->rd.topRows(SPACE_DIM) = pbee_t;
		this->rd.bottomRows(SPACE_DIM).setZero();

	}
}

/**
* @brief Update function
* Update the task Jacobian matrix
*/
void AutonomousControlTask::updateTaskJacobianMatrix() {

	this->J = this->robot->getJacobian();

}
