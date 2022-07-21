#ifndef TASK_CONTEXT_HPP_
#define TASK_CONTEXT_HPP_

// Project Header files
#include "BusinessLogicInterface.hpp"
#include "TaskStrategy.hpp"

class TaskContext : public BusinessLogicInterface {

public:

	/**
	* @brief Constructor of ControlContext
	*/
	TaskContext(TaskStrategy* s = nullptr) :
		routine(s)	{}


	/**
	* @brief Destroyer of TaskContext
	*/
	~TaskContext() {
		delete this->routine;
	}

	/**
	* @brief Init function
	*/
	void init();

	/**
	* @brief Set function
	* Set the TaskStrategy object for the considered ControlContext object
	* @param s: the TaskStrategy object
	*/
	inline void setTaskStrategy(TaskStrategy* r) {
		delete this->routine;
		this->routine = r;
		this->routine->setSimPort(this->simPort);
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
	* @brief Update function
	* Check for user requests of new changes on the running tasks
	*/
	void checkTaskRequests();

private:

	TaskStrategy* routine; 		//!< Pointer to a TaskStrategy object


};



#endif // TASK_CONTEXT_HPP_
