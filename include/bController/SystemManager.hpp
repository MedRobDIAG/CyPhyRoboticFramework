#ifndef SYSTEM_MANAGER_HPP_
#define SYSTEM_MANAGER_HPP_


// Project Header files
#include "BusinessLogicInterface.hpp"
#include "RobotProxy.hpp"
#include "TeleopProxy.hpp"
#include "HapticProxy.hpp"
#include "TaskContext.hpp"
#include "ControlContext.hpp"
#include "SensorReader.hpp"
#include "Logger.hpp"

class SystemManager : public BusinessLogicInterface {

public:

	/**
	* @brief Default constructor of the SystemManager object
	*
	*/
	SystemManager();

	/**
	* @brief Default destroyer of the SystemManager object
	*
	*/
	~SystemManager();

	/**
	* @brief Init function
	*/
	void init();

	/**
	* @brief Set function
	* Set and initialize the proxies objects based on the initial settings of the Configuration object
	*/
	void initProxies();
	
	/**
	* @brief Instrument init function
	* Initialize the instruments to be shared with Task and Controller objects
	*/
	void initInstruments();

	/**
	* @brief Robot initial pose function
	* Set the robot (real or virtual) to the desired initial robot pose
	*/
	void setInitialRobotPose();

	/**
	* @brief BL class init function
	* Initialize the BL instance given by the input pointer
	* @param instance: the BL class to be initialized
	*/
	void initBLInstance(BusinessLogicInterface* instance);

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
	* @brief Run function
	* Run the BusinessLogic class threads (Task, Controller, Logger, SensorReader)  and
	* to exchange data and keep updated the state of the program, based on
	* the user inputs
	*/
	void startSystemThreads();

	/**
	* @brief Join thread function
	* Join the previously launched threads
	*/
	void joinSystemThreads();

	/**
	* @brief Check function
	* Check if the activities in the system have been concluded
	*/
	bool isActivityOver() { return this->systemstate->getTaskInfo().taskState == STOPPED_STATE; }

	/**
	* @brief Set function
	* Set the user action in SystemState class
	* @param a: the user action
	*/
	inline void setSystemAction(const int& a) { this->systemstate->setAction(a); }

	/**
	* @brief Set function
	* Set the show functionality in SystemState class
	* @param a: the show functionality flag
	*/
	inline void setSystemShowFcn(const int& s) { this->systemstate->setShow(s); }

	/**
	* @brief Set function
	* Set the task state in SystemState class
	* @param state: the task state
	*/
	inline void setSystemTaskState(const int& s) { this->systemstate->setTaskState(s); }

	/**
	* @brief Set function
	* Set the task type in SystemState class
	* @param state: the task type
	*/
	inline void setSystemTaskType(const int& t) { this->systemstate->setTaskType(t); }

	/**
	* @brief Set function
	* Set the constraint state in SystemState class
	* @param state: the constraint state
	*/
	inline void setSystemConstraintState(const int& s) { this->systemstate->setConstraintState(s); }

	/**
	* @brief Set function
	* Set the constraint type in SystemState class
	* @param state: the constraint type
	*/
	inline void setSystemConstraintType(const int& t) { this->systemstate->setConstraintType(t); }

	/**
	* @brief Set function
	* Set the control state in SystemState class
	* @param state: the control state
	*/
	inline void setSystemControlState(const int& s) { this->systemstate->setControlState(s); }

	/**
	* @brief Set function
	* Set the control mode in SystemState class
	* @param cm: the control mode
	*/
	inline void setSystemControlMode(const int& cm) { this->systemstate->setControlMode(cm); }

	/**
	* @brief Get function
	* Get the task state from SystemState class
	* @return the task state
	*/
	inline int getSystemTaskState() { return this->systemstate->getTaskInfo().taskState; }

	/**
	* @brief Lock function
	* Lock the mutex on the keyboard resource in the SystemState class
	*/
	inline void lockKBMutexInSysState() { this->systemstate->lockKBMtx(); }

	/**
	* @brief Unlock function
	* Unlock the mutex on the keyboard resource in the SystemState class
	*/
	inline void unlockKBMutexInSysState() { this->systemstate->unlockKBMtx(); }

	/**
	* @brief Reset function
	* Send the request to reset the F/T Sensor bias
	*/
	void resetSystemFTSensorBias(); 

	/**
	* @brief Update function
	* Update the state of the robot from data acquired from the
	* corresponding proxy
	* @param rs: the RobotState structure
	* @param online: online processing flag (default true)
	*/
	void updateRobot(const RobotState& rs, const bool& online = true);

	/**
	* @brief Update function
	* Update the state of the haptic device from data acquired from the
	* corresponding proxy
	* @param hs: the HapticState structure
	*/
	void updateHapticDevice(const HapticState& hs);

	/**
	* @brief Update function
	* Update the state of the teleop device from data acquired from the
	* corresponding proxy
	* @param data: a void pointer to the acquired data (internal static cast is required based on the type of device)
	*/
	void updateTeleopDevice(TeleopState* ts);

	/**
	* @brief Update function
	* Update the state of the F/T sensor from data acquired from the
	* corresponding proxy
	* @param meas: the F/T sensor measurement
	*/
	void updateFTSensor(const Eigen::Vector6f& meas);

private:


	TaskContext task;					//!< Instance of TaskContext, that manages the different task to set
	ControlContext ctrl;				//!< Instance of ControllerContext, that manages the different controllers to set
	Logger log;							//!< Instance of Logger, that logs data stored in the Entity classes

	

};


#endif // SYSTEM_MANAGER_HPP_
