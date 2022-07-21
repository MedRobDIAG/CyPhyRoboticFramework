#ifndef BUSINESS_LOGIC_INTERFACE_HPP_
#define BUSINESS_LOGIC_INTERFACE_HPP_

// Standard Header files
#include <map>

// System Header files
#include <thread>
#include <map>

// Project Header files
#include "ExtSystemProxy.hpp"
#include "Instrument.hpp"
#include "Configuration.hpp"
#include "Timer.hpp"
#include "SystemState.hpp"

__declspec(align(16)) class BusinessLogicInterface {

public:

	/**
	* @brief Default constructor of HLInterface object (virtual)
	*
	*/
	BusinessLogicInterface() {}

	/**
	* @brief Default destroyer of HLInterface object (virtual)
	*
	*/
	~BusinessLogicInterface() {}

	/**
	* @brief Init function (virtual)
	*
	*/
	virtual void init() = 0;

	/**
	* @brief Run function
	*
	*/
	inline void run() { hlthread = std::thread(&BusinessLogicInterface::mainLoop, this); }

	/**
	* @brief Joint function
	*
	*/
	inline void join() { hlthread.join(); }

	/**
	* @brief Set function
	* Set the thread_terminate flag to true
	*/
	inline void exit() { this->exit_loop = true; }

	/**
	* @brief Set function
	* Restore the thread_terminate flag to false
	*/
	inline void restore() { this->exit_loop = false; }

	/**
	* @brief Set function
	* Start the HLInterface
	*/
	inline void start() { this->started = true; this->accomplished = false; }

	/**
	* @brief Set function
	* Stop the HLInterface
	*/
	inline void stop() { this->accomplished = true; this->started = false; }

	/**
	* @brief Check function
	* Check if the Task has started
	* @return true if the Task has started, false otherwise
	*/
	inline bool isStarted() { return this->started; }

	/**
	* @brief Check function
	* Check if the Task has been accomplished
	* @return true if the Task has been accomplished, false otherwise
	*/
	inline bool isDone() { return this->accomplished; }

	/**
	* @brief Wait function
	* Waits for the user input requesting the task start
	*/
	void BusinessLogicInterface::waitForStartRequest() {

		Timer clock;

		while (!this->started && this->ok()) {
			clock.timeSleep(0.01);
		}

	}

	/**
	* @brief Check function
	* Check if the thread associated to the current HLInterface is running
	* @return true if the thread is running
	*/
	inline bool ok() { return !this->exit_loop; }

	/**
	* @brief Set function
	* Set the connection port number of the simulator server for the given HLInterface
	* @param port_: the connection port number 
	*/
	inline void setSimPort(const int& port_) { this->simPort = port_; }

	/**
	* @brief Main loop function
	*
	*/
	virtual void mainLoop() = 0;

	/**
	* @brief Clear function
	*/
	virtual void clear() = 0;

	/**
	* @brief new operator ovverriding function
	*/
	void* operator new(size_t i) { return _mm_malloc(i, 16); }

	/**
	* @brief delete operator ovverriding function
	*/
	void operator delete(void* p) { _mm_free(p); }

	/**
	* @brief Get function
	* Get the list of proxies
	* @return the map list of proxies
	*/
	inline std::map < std::string, ExtSystemProxy* > getProxyList() { return this->proxies; }

	/**
	* @brief Get function
	* Get the list of instruments
	* @return the map list of instruments
	*/
	inline std::map < std::string, Instrument* > getInstrumentList() { return this->instruments; }


	/**
	* @brief Set function 
	* Set the proxies on the current BusinessLogicInterface object
	*/
	inline void setProxyList(std::map < std::string, ExtSystemProxy* >* p) { this->proxies = *p; }

	/**
	* @brief Set function 
	* Set the entities on the current BusinessLogicInterface object
	*/
	inline void setInstrumentList(std::map < std::string, Instrument* >* i) { this->instruments = *i; }

	/**
	* @brief Set function
	* Set the time variable of the BusinessLogicInterface class
	* @param t: the timestamp to be set
	*/
	inline void setTimeStamp(const double& t) { this->time_ = t; }

	/**
	* @brief Get function
	* Get the time variable of the BusinessLogicInterface class
	* @return the timestamp to be set
	*/
	inline double getTimeStamp() { return this->time_ ; }


protected:

	SystemState* systemstate;		//*** No need to define here as class member if this is singleton. TODO: Remove from here
	Configuration* config;			//*** No need to define here as class member if this is singleton. TODO: Remove from here

	std::thread hlthread;			//!< Thread object of the corresponding interface
	bool exit_loop;					//!< Boolean variable stating if the thread has to be terminated
	int simPort;					//!< Connection port number of the simulator server for the given HLInterface
	bool started;					//!< State if the HLInterface has started
	bool accomplished;				//!< State if the HLInterface has finished
	double time_;					//!< Local storage of timestamp

	/* Proxies */
	std::map < std::string, ExtSystemProxy* > proxies; //!< Map of proxies objects

	/* Instruments */
	std::map < std::string, Instrument* > instruments; //!< Map of proxies objects

};


#endif // BUSINESS_LOGIC_INTERFACE_HPP_
