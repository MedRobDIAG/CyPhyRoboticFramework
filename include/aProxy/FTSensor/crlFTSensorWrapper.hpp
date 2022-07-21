#ifndef FTSENSORPROXY_HPP_
#define FTSENSORPROXY_HPP_

//Project Header files
#include "ExtSystemProxy.hpp"

// OS Header files
#include <Windows.h>
#include <iostream>

#define WRENCH_DIM 6

class crlFTSensorWrapper : public ExtSystemProxy {

public:

	/**
	* @brief Default contructor of FTSensorProxy class
	*
	*/
	crlFTSensorWrapper();

	/**
	* @brief Default destroyer of FTSensorProxy class
	*
	*/
	~crlFTSensorWrapper();

	/**
	* @brief Default init function
	*/
	void init();

	/**
	* @brief Default run function
	*/
	void run();

	/**
	* @brief Default clear function
	*/
	void clear();

	/**
	* @brief Default thread start function
	*/
	//inline void startFTthread() { std::cout << "Calling startFTthread()..." << std::endl; this->proxy_thread = boost::thread(boost::bind(&crlFTSensorWrapper::run, this)); std::cout << "Boost thread for FTSensor called" << std::endl; };

	/**
	* @brief Join thread function
	* Join the previously launched thread for the current ExtSystemProxy object
	*/
	//inline void joinFTthread() { this->proxy_thread.join(); }


	/**
	* @brief Reset function
	* Request to reset the bias of the sensor
	*/
	inline void requestBiasReset() { this->resetBias = true; }

	/**
	* @brief Get function
	* Retrieve the current wrench measurement of the sensor
	* @param (out) a static double-precision vector containing the current F/T measurement of the sensor
	*/
	inline void getCurrentWrenchMeas(double meas[]) { std::memcpy(meas,this->wrench, WRENCH_DIM * sizeof(double)); }

private:

	bool resetBias;				//!< Flag stating if the bias has to be reset
	double wrench[WRENCH_DIM];	//!< wrench vector
	HANDLE h;					//!< WinNT handle
};


#endif // FTSENSORPROXY_HPP_
