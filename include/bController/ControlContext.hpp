#ifndef CONTROL_CONTEXT_HPP_
#define CONTROL_CONTEXT_HPP_

// Project Header files
#include "BusinessLogicInterface.hpp"
#include "ControlStrategy.hpp"
#include "ControlTask.hpp"

enum TASK_PRIORITY {HIGHEST_PRIORITY = 0, MIDDLE_LEVEL_PRIORITY, LOWEST_PRIORITY};

class ControlContext : public BusinessLogicInterface {

public:

	/**
	* @brief Constructor of ControlContext
	*/
	ControlContext(ControlStrategy* s = nullptr) : 
		strategy(s)
		{}
	
	
	/**
	* @brief Destroyer of ControlContext
	*/
	~ControlContext() { 
		delete this->strategy;
	}

	/**
	* @brief Init function
	*/
	void init();

	/**
	* @brief Set function
	* Set the ControlStrategy object for the considered ControlContext object
	* @param s: the ControlStrategy object
	*/
	inline void setControlStrategy(ControlStrategy* s) {
		delete this->strategy;
		this->strategy = s;
	}

	/**
	* @brief Main loop function
	*
	*/
	void mainLoop();

	/**
	* @brief Clear function
	*/
	void clear();

	/**
	* @brief Add function
	* Add a new task to the stack of tasks to be solved
	* @param task: the pointer to the ControlTask object to be added to the stack
	* @param priority: the desired priority of the task in the stack
	* @param oveerideTask: if a task of the same type in the stack must be ovverridden by the new task
	*/
	void addNewTask(ControlTask* task, const int& priority = HIGHEST_PRIORITY, const bool& overrideTask = false);

	/**
	* @brief Update fucntion
	* Update the state of the tasks to be solved in the stack
	*/
	void updateTasks();

	/**
	* @brief Sort fucntion
	* Sort the tasks to be solved in the stack according to their priority
	*/
	void sortTasks();

	/**
	* @brief Print function
	* Print the status of the stack of tasks
	*/
	void printStackState();

	/**
	* @brief Clear function
	* Clear from the stack of tasks all the tasks with given input class
	* @param tc: the inpu task class to be cleared from the stack
	*/
	void clearTaskClassFromStack(const int& tc);

	/**
	* @brief Solve function
	* Apply the task priority formulation to solve the stack of control tasks
	* @return the final joint velocity solution vector
	*/
	Eigen::VectorXf solveControlStack();

	/**
	* @brief Update function
	* Update the control modality (among teleoperation, autonomous and manual guidance) based on user request
	*/
	void checkCtrlModeRequest();

	/**
	* @brief Update function
	* Update the kinematic constraint (among linear motion, angular motion, virtual fixture, RCM and no constraints) based on user request
	*/
	void checkKinConstraintRequest();

private:

	ControlStrategy* strategy; 										//!< Pointer to a ControlStrategy object
	std::vector < std::pair < int, ControlTask* > > stackOfTasks;	//!< Stack of tasks to be solved
};



#endif // CONTROL_CONTEXT_HPP_
