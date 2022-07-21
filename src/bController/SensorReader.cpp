// Project Header files
#include "SensorReader.hpp"

/**
* @brief Default constructor of the SensorReader object
*
*/
SensorReader::SensorReader() : BusinessLogicInterface() {


}

/**
* @brief Default destroyer of the SensorReader object
*
*/
SensorReader::~SensorReader() {}


/**
* @brief Init function
*/
void SensorReader::init() {

	this->exit_loop = false;
	this->started = false;
	this->accomplished = false;

	// Initialize the singleton SystemState and Configuration classes
	// SystemState
	this->systemstate = SystemState::GetInstance();

	// Configuration
	this->config = Configuration::GetInstance(""); // Input filename string is useless here, as the singleton class is supposed to be already instantiated by SystemManager

}

/**
* @brief Main loop function
*
*/
void SensorReader::mainLoop() {

	// Time variables
	double tictoc, dt, rate, tic, toc, tac, Ts, des_rate;
	float t_curr, t_prev;
	Timer clock;
	des_rate = 200.0;
	clock.setRate(des_rate);
	Ts = 1.0 / des_rate;


	while (this->ok()) {

		// Measure starting time
		tic = clock.getCurTime();

		//----------------------------------------------------------------//
		// Do stuff here... 

		//----------------------------------------------------------------//

		// Measure the ending time and the elapsed time
		toc = clock.getCurTime();
		tictoc = clock.elapsedTime(tic, toc);

		// Wait until Ts
		if (tictoc < Ts) {
			clock.timeSleep(Ts - tictoc);
		}

		// Measure the final time after sleep to check the actual rate of the thread
		tac = clock.getCurTime();
		dt = clock.elapsedTime(tic, tac);
		//this->simTime += Ts;
		//debugPrint<double>("[Controller] Running rate:", 1.0 / dt);

	}

}

/**
* @brief Clear function
*/
void SensorReader::clear() {}
