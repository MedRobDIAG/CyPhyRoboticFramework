// Project Header files 
#include "UserProxy.hpp"

/**
* @brief Executable function
*/
void UserProxy::exec() {

	// Init the UserProxy
	this->init();

	// Init the SystemManager and its structures
	this->sys.init();

	// Launch the UserProxy thread
	this->startThread();

	// Launch the required set of threads from SystemManager 
	this->sys.startSystemThreads();

	// Join the required set of threads from SystemManager 
	this->sys.joinSystemThreads();

	// Join the UserProxy thread
	this->joinThread();

}
