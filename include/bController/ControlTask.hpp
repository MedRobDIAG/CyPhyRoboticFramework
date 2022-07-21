#ifndef CONTROL_TASK_HPP_
#define CONTROL_TASK_HPP_

// Project Header files
#include "RobotInterface.hpp"

// Eigen Header files
#include <Eigen\Dense>

enum TASK_CLASS {POSITION_CONTROL, FORCE_CONTROL, JOINT_CONTROL, KINEMATIC_CONSTRAINT};

class ControlTask {

public:
	/** PLEASE READ THIS CAREFULLY **/
	/* When we define a class containing fixed-size Eigen structures,
	* and we later instantiate such class dynamically, it occurs an issue
	* that may result in undesired and unexpected program crash if not
	* properly handled, due to ACCESS VIOLATIONS.
	* To prevent this, every class containing fixed-size Eigen variables
	* must add the line below in the public part of the class definition,
	* in order to overload the new operator and explicitly handle alignment
	* problems in the variable definitions. If you have a polymorphic structure
	* among your class, place this macro in the Base class, like here.
	*/
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
	* @brief Constructor of ControlTask object with task dimension argument
	* @param tc: the task class
	* @param td: the task dimension
	* @param ri: the pointer to Robotinterface object
	*
	*/
	ControlTask(const int& tc, const int& td, RobotInterface* ri = nullptr) : taskClass(tc), taskDim(td), robot(ri) {

		// Initialize the dynamic vectors with the set task dimension
		this->init();

	}

	/**
	* @brief Default destroyer of ControlTask object
	*
	*/
	~ControlTask() {}
	
	/**
	* @brief Init function
	* Initialize the dynamic vectors of the class with the set task dimension parameter
	*/
	inline void init() {
	
		int jointNum = robot->getJointNum();

		if (this->taskDim != -1) {
			
			uff.setZero(this->taskDim);
			r.setZero(this->taskDim);
			rd.setZero(this->taskDim);
			J.setZero(this->taskDim, jointNum);
			this->setGainMatrix(1.0);
		}

	}

	/**
	* @brief Get function
	* Get the class of the Control Task
	* @return the taskClass
	*/
	inline int getTaskClass() { return this->taskClass; }

	/**
	* @brief Update function (virtual)
	* Update the feed forward input vector
	*/
	virtual void updateFeedForwardInput() = 0;

	/**
	* @brief Update function (virtual)
	* Update the task vector
	*/
	virtual void updateTaskVector() = 0;

	/**
	* @brief Update function (virtual)
	* Update the task reference vector
	*/
	virtual void updateTaskReferenceVector() = 0;

	/**
	* @brief Update function (virtual)
	* Update the task Jacobian matrix
	*/
	virtual void updateTaskJacobianMatrix() = 0;

	/**
	* @brief Closed-loop function
	* Compute the closed-loop input based on the updated information
	* @return the closed-loop input of the task
	*/
	inline Eigen::VectorXf computeClosedLoopInput() {
	
		return (this->uff - this->K * (this->r - this->rd));
	
	}

	/**
	* @brief Task error function
	* Compute the task error vector
	* @return the task error vector
	*/
	inline Eigen::VectorXf computeTaskError() {

		return (this->r - this->rd);

	}

	/**
	* @brief Set function
	* Set the gain matrix of the control task
	* @param K: the gain matrix
	*/
	inline void setGainMatrix(const Eigen::MatrixXf& Kgain) { this->K = Kgain; }

	/**
	* @brief Set function
	* Set the gain matrix of the control task
	* @param gain: the gain scalar
	*/
	inline void setGainMatrix(const float& gain) { this->K = Eigen::MatrixXf::Identity(this->taskDim, this->taskDim) * gain; }

	/**
	* @brief Get function
	* Get the gain matrix of the control task
	* @return the gain matrix
	*/
	inline Eigen::MatrixXf getGainMatrix() { return this->K; }

	/**
	* @brief Update function
	* Update the Control Task
	*/
	inline void updateTask() {
	
		this->updateTaskReferenceVector();
		this->updateTaskVector();
		this->updateFeedForwardInput();
		this->updateTaskJacobianMatrix();

	}

	/**
	* @brief Get function
	* Get the task vector of the control task class
	* @return the task vector
	*/
	inline Eigen::VectorXf getTaskVector() { return this->r; }

	/**
	* @brief Get function
	* Get the task reference vector of the control task class
	* @return the task reference vector
	*/
	inline Eigen::VectorXf getTaskRefVector() { return this->rd; }

	/**
	* @brief Get function
	* Get the task Jacobian matrix of the control task class
	* @return the task Jacobian matrix 
	*/
	inline Eigen::MatrixXf getTaskJacobian() { return this->J; }

	/**
	* @brief Get function
	* Get the feed-forward input vector of the control task class
	* @return the feed-forward input vector
	*/
	inline Eigen::VectorXf getFFInput() { return this->uff; }

	/**
	* @brief Get function
	* Get the task dimension
	* @return the task dimension
	*/
	inline int getTaskDim() { return this->taskDim; }

protected:

	int taskDim;				//!< Dimension of the task
	int taskClass;				//!< Type of the task
	Eigen::VectorXf uff;		//!< Feed-forward control input
	Eigen::VectorXf r;			//!< Task vector
	Eigen::VectorXf rd;			//!< Task reference vector
	Eigen::MatrixXf J;			//!< Task jacobian matrix
	Eigen::MatrixXf K;			//!< Task gain matrix
	RobotInterface* robot;		//!< Pointer to a RobotInterface object

};

#endif // CONTROL_TASK_HPP_
