#ifndef LOGGER_HPP_
#define LOGGER_HPP_

// System Header files
#include <fstream>
#include <iostream>
#include <sstream>

// Project Header files
#include "BusinessLogicInterface.hpp"
#include "KUKARobot.hpp"
#include "Geomagic.hpp"
#include "FTSensor.hpp"

namespace Eigen {
	typedef Eigen::Matrix<float, TWIST_DIM, 1> Vector6f;
	typedef Eigen::Matrix<double, TWIST_DIM, 1> Vector6d;
}


class Logger : public BusinessLogicInterface {


public:

	/**
	* @brief Default constructor of the Logger class
	*
	*/
	Logger();

	/**
	* @brief Default destroyer of the Logger class
	*
	*/
	~Logger() {}

	/**
	* @brief Set function
	* Set the proxies on the current HLInterface object
	*/
	void setProxies(std::map < std::string, ExtSystemProxy* >& proxies);

	/**
	* @brief Set function
	* Set the entities on the current HLInterface object
	*/
	void setInstruments(std::map < std::string, Instrument* >& instruments);

	/**
	* @brief Init function
	*
	*/
	void init();
	
	/**
	* @brief Run function 
	*/
	void mainLoop();

	/**
	* @brief Clear function
	*/
	void clear();

	/**
	* @brief Create function
	* Create the logging folder
	* @return the path of the logging folder
	*/
	std::string createMainFolder();

	/**
	* @brief Create function
	* Create a generic folder with the name specified by folderName, in the path folderPath
	* @param folderName the name of the folder to be created
	* @return 1 if the folder has been created, 0 if it already exists and -1 in case of error
	*/
	int createFolder(const char* folderName);

	/**
	* @brief Start function
	* Start the data logging
	*/
	inline void start() { this->startLog = true; this->time_ = 0.0; }

	/**
	* @brief Stop function
	* Stop the data logging
	*/
	inline void stop() { this->startLog = false; this->saveLog = false; this->time_ = 0.0; }

	/**
	* @brief Check function
	* Check if the Logger is logging
	* @return true if the Logger is logging, false otherwise
	*/
	inline bool isLogging() { return this->startLog; }

	/**
	* @brief Save request function
	* Set the saveLog flag to request saving the data acquired so far
	*/
	void requestSaveAndWait();

	/**
	* @brief Save and close function
	* Save the data acquired so far on the corresponding files, and reset the logger
	*/
	void saveLogsAndReset();

private:

	std::string folderPath;					//!< Path of the folder in which data should have been saved/loaded
	bool startLog;							//!< State when to start logging data
	bool saveLog;							//!< State if acquired data must be saved
	//double time_;							//!< Logging time
	
	/* Entities */
	RobotInterface* robot;
	Geomagic* haptic;
	FTSensor* ftsensor;

	/* Stringstream objects */
	std::stringstream qmSS;
	std::stringstream qcSS;
	std::stringstream qmdotSS;
	std::stringstream qcdotSS;
	std::stringstream taumsrSS;
	std::stringstream TbeeSS;
	std::stringstream vbeeSS;
	std::stringstream ftmeasSS;
	std::stringstream vhapticSS;
	std::stringstream clutchStateSS;
	std::stringstream myResidualSS;
	std::stringstream friResidualSS;
	std::stringstream friCartFroceSS;
	std::stringstream friGravityVectorSS;
	std::stringstream friInertiaMatrixSS;
	std::stringstream modelGravityVectorSS;
	std::stringstream modelInertiaMatrixSS;
	std::stringstream resFbeeSS;
	std::stringstream resFriFbeeSS;
	std::stringstream detJJTSS;
	std::stringstream detJTJSS;
};



#endif // LOGGER_HPP_
