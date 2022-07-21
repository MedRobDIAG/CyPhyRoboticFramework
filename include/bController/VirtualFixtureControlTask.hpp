#ifndef VIRTUAL_FIXTURE_CONTROL_TASK_HPP_
#define VIRTUAL_FIXTURE_CONTROL_TASK_HPP_

// Project Header files
#include "ControlTask.hpp"
#include <Eigen/Core>

class VirtualFixtureControlTask : public ControlTask {

public:

	/**
	* @brief Constructor of VirtualFixtureControlTask object with RobotInterface pointer argument
	* @param tc: the task class
	* @param td: the task dimension
	* @param ri: the RobotInterface pointer argument
	*
	*/
	VirtualFixtureControlTask(const int& tc, const int& td, RobotInterface* ri = nullptr);

	/**
	* @brief Constructor of VirtualFixtureControlTask object with selection matrix and RobotInterface pointer arguments
	* @param S: the selection matrix
	* @param ri: the RobotInterface pointer argument
	*
	*/
	VirtualFixtureControlTask(const int& tc, const Eigen::Matrix6i& S, RobotInterface* ri = nullptr);

	/**
	* @brief Default destroyer of ControlTask object
	*
	*/
	~VirtualFixtureControlTask();

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
	* Set the selection matrix stating the direction of the virtual fixture
	* @param S: the selection matrix
	*/
	inline void setSelectionMatrix(const Eigen::Matrix6i& S) { this->selectMatrix = S; }

private:

	bool constraintSet;	//!< Flag stating if the constraint has been set
	Eigen::Matrix4f Tbee0;			//!< 4x4 homogeneous matrix of the end effector initial pose
	Eigen::Matrix6i selectMatrix;	//!< Selection matrix choosing the direction of the virtual fixture
	Eigen::ArrayXi indexes;			//!< Indexes resulting from selectMatrix. Proxy data of selectionMatrix
};

#endif // VIRTUAL_FIXTURE_CONTROL_TASK_HPP_
