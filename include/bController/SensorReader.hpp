#ifndef SENSOR_READER_HPP_
#define SENSOR_READER_HPP_

// Project Header files
#include "BusinessLogicInterface.hpp"

class SensorReader : public BusinessLogicInterface {

public:


	/**
	* @brief Default constructor of the SensorReader object
	*
	*/
	SensorReader();

	/**
	* @brief Default destroyer of the SensorReader object
	*
	*/
	~SensorReader();


	/**
	* @brief Init function
	*/
	void init();

	/**
	* @brief Main loop function
	*
	*/
	void mainLoop();

	/**
	* @brief Clear function
	*/
	void clear();

private:


};

#endif // SENSOR_READER_HPP_
