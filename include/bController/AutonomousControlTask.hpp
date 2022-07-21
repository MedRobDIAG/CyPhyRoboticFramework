#ifndef AUTONOMOUS_CONTROL_TASK_HPP_
#define AUTONOMOUS_CONTROL_TASK_HPP_

// Project Header files
#include "ControlTask.hpp"
#include "SinTrajectory.hpp"

class AutonomousControlTask : public ControlTask {

public:

	/**
	* @brief Constructor of AutonomousControlTask object with RobotInterface pointer argument
	* @param tc: the task class
	* @param td: the task dimension
	* @param ri: the RobotInterface pointer argument
	* @param t: the Trajectory pointer argument
	*
	*/
	AutonomousControlTask(const int& tc, const int& td, RobotInterface* ri = nullptr, Trajectory* t = nullptr);

	/**
	* @brief Default destroyer of AutonomousControlTask object
	*
	*/
	~AutonomousControlTask();

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
	* @brief Set function
	* Set the trajectory object
	* @param t: the pointer to the Trajectory object to be set
	*/
	inline void setTrajectory(Trajectory* t) {
		delete this->traj;
		this->traj = t;
	}

private:

	Trajectory* traj;		//!< Pointer to the Trajectory object
	Eigen::Matrix4f Tbee0;	//!< 4x4 homogeneous matrix of the end effector initial pose

};

#endif // AUTONOMOUS_CONTROL_TASK_HPP_
