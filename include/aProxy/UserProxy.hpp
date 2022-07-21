#ifndef USER_PROXY_HPP_
#define USER_PROXY_HPP_

// Project Header files
#include "ExtSystemProxy.hpp"
#include "SystemManager.hpp"
#include "Timer.hpp"

class UserProxy : public ExtSystemProxy {

public:

	/**
	* @brief Default contructor of UserProxy class
	*
	*/
	UserProxy() : ExtSystemProxy() {}

	/**
	* @brief Default destroyer of UserProxy class
	*
	*/
	~UserProxy() {}

	/**
	* @brief Executable function
	*/
	void exec();

	/**
	* @brief Default init function
	*/
	virtual void init() = 0;

	/**
	* @brief Default run function
	*/
	virtual void run() = 0;

	/**
	* @brief Default clear function
	*/
	virtual void clear() = 0;


protected:

	SystemManager sys;			//!< Instance off System Manager
};


#endif // USER_PROXY_HPP_