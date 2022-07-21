#ifndef SYSTEM_STATE_HPP_
#define SYSTEM_STATE_HPP_

// Standard Header files
#include <mutex>

// Eigen Header files
#include <Eigen\Dense>

// Project Header files
#include "utils.hpp"

enum STATE { UNDEFINED_STATE = -1, REQUESTED_STATE, SET_STATE, RUNNING_STATE, STOPPING_STATE, STOPPED_STATE, WAITING_STATE };
enum ACTION { REQUESTED, CLEARED };
enum TASK { UNDEFINED_TASK = 0, REGISTRATION_TASK, PLANNING_TASK, NEEDLE_TISSUE_KV_ESTIMATION_TASK, NEEDLE_TISSUE_GEROVICH_ESTIMATION_TASK, FORCE_FEEDBACK_NEEDLE_INSERTION_TASK};
enum CONTROL {UNDEFINED_CONTROL = 0, TELEOPERATION_MODE, AUTONOMOUS_MODE, MANUAL_GUIDANCE_MODE};
enum CONSTRAINT_TYPE {NO_CONSTRAINT = 0, LINEAR_MOTION_CONSTRAINT, ANGULAR_MOTION_CONSTRAINT, VIRTUAL_FIXTURE_CONSTRAINT, RCM_CONSTRAINT, NUM_CONSTRAINTS};

struct ConstraintInfo {

	int constraintType;						//!< Type of constraint
	int constraintState;					//!< State of constraint

};

struct TaskInfo {

	int taskType;							//!< Type of task
	int taskState;							//!< State of task 

};

struct ControlInfo{

	int ctrlMode;							//!< Type of control 
	int ctrlState;							//!< State of the control
	int kinContranitType;					//!< Type of kinematic constraint

};

struct SensorInfo {

	std::vector <std::string> sensors;		//!< Array of connected sensor names
	std::string robotType;					//!< Type of robot in use
	int hapticDeviceNum;					//!< Number of connected haptic devices

};


/**
* The Singleton class defines the `GetInstance` method that serves as an
* alternative to constructor and lets clients access the same instance of this
* class over and over.
*/
class SystemState {


protected:

	/*
	* @brief Constructor of the SystemState class with log number argument
	* @param logNum: the number of the log to be loaded in the Dataset instance
	*/
	SystemState() {}

	/**
	* @brief Destroyer of the SystemState class
	*/
	~SystemState() {}


public:


	/**
	* Singletons should not be cloneable.
	*/
	SystemState(SystemState &other) = delete;

	/**
	* Singletons should not be assignable.
	*/
	void operator=(const SystemState &) = delete;

	/**
	* This is the static method that controls the access to the singleton
	* instance. On the first run, it creates a singleton object and places it
	* into the static field. On subsequent runs, it returns the client existing
	* object stored in the static field.
	*/
	inline static SystemState *GetInstance()
	{
		if (pinstance_ == nullptr)
		{
			std::lock_guard<std::mutex> lock(mutex_);
			if (pinstance_ == nullptr)
			{
				pinstance_ = new SystemState();
			}
		}
		return pinstance_;
	}

	/**
	* @brief Set function
	* Set the user action
	* @param act: the user action
	*/
	inline void setAction(const int& act) { this->action = act; }

	/**
	* @brief Get function
	* Get the user action
	* @return the user action
	*/
	inline int getAction() { return this->action; }

	/**
	* @brief Set function
	* Set the show function flag
	* @param s: the show function flag
	*/
	inline void setShow(const int& s) { this->show= s; }

	/**
	* @brief Get function
	* Get the show function flag
	* @return the show function flag
	*/
	inline int getShow() { return this->show; }

	/**
	* @brief Set function
	* Set the control state
	* @param state: the control state
	*/
	inline void setControlState(const int& state) { this->ctrl_i.ctrlState = state; }

	/**
	* @brief Set function
	* Set the control mode
	* @param state: the control mode
	*/
	inline void setControlMode(const int& mode) { this->ctrl_i.ctrlMode = mode; }

	/**
	* @brief Set function
	* Set the task state
	* @param state: the task state
	*/
	inline void setTaskState(const int& state) { this->task_i.taskState = state; }

	/**
	* @brief Set function
	* Set the task type
	* @param state: the task type
	*/
	inline void setTaskType(const int& type) { this->task_i.taskType = type; }
	/**
	* @brief Set function
	* Set the constraint state
	* @param state: the constraint state
	*/
	inline void setConstraintState(const int& state) { this->constr_i.constraintState = state; }

	/**
	* @brief Set function
	* Set the constraint type
	* @param state: the constraint type
	*/
	inline void setConstraintType(const int& type) { this->constr_i.constraintType = type; }

	/**
	* @brief Add function
	* Add the input sensor name to the list of connected sensors
	* @params sname: the name of the input sensor
	*/
	inline void addSenorName(const std::string& sname) { this->sens_i.sensors.push_back(sname); }

	/**
	* @brief Set function
	* Set the number of connected haptic devices
	* @params hnum: the number of connected happtic devices
	*/
	inline void setHapticDeviceNum(const int& hnum) { this->sens_i.hapticDeviceNum = hnum; }

	/**
	* @brief Set function
	* Set the name of the employed robot
	* @params rname: the name of the employed robot
	*/
	inline void setRobotTypeName(const std::string& rname) { this->sens_i.robotType = rname; }

	/**
	* @brief Get function
	* Get the TaskInfo structure
	* @return the TaskInfo structure
	*/
	inline TaskInfo getTaskInfo() { return this->task_i; }

	/**
	* @brief Get function
	* Get the Controlinfo structure
	* @return the Controlinfo structure
	*/
	inline ControlInfo getCtrlInfo() { return this->ctrl_i; }

	/**
	* @brief Get function
	* Get the SensorInfo structure
	* @return the SensorInfo structure
	*/
	inline SensorInfo getSensorInfo() { return this->sens_i; }

	/**
	* @brief Get function
	* Get the ConstraintInfo structure
	* @return the ConstraintInfo structure
	*/
	inline ConstraintInfo getConstraintInfo() { return this->constr_i; }

	/**
	* @brief Get function
	* Get the number of haptic devices
	* @return the number of haptic devices
	*/
	inline int getHapticDeviceNum() { return this->sens_i.hapticDeviceNum; }

	/**
	* @brief Get function
	* Get the kinematic constraint type flag
	* @return the kinematic constraint type flag
	*/
	inline int getKinConstraint() { return this->ctrl_i.kinContranitType; }

	/**
	* @brief Get function
	* Get the robot type string
	* @return the robot type string
	*/
	inline std::string getRobotType() { return this->sens_i.robotType; }

	/**
	* @brief Set function
	* Set the Log path in the SystemState class
	* @param path: the log path
	*/
	inline void setLogPath(const std::string& path) { this->logPath = path; }

	/**
	* @brief Get function
	* Get the Log path in the SystemState class
	* @return the log path
	*/
	inline std::string getLogPath() { return this->logPath;}

	/**
	* @brief Lock function
	* Lock the mutex on the keyboard resource
	*/
	inline void lockKBMtx() { this->kbMtx.lock(); }

	/**
	* @brief Unlock function
	* Unlock the mutex on the keyboard resource
	*/
	inline void unlockKBMtx() { this->kbMtx.unlock(); }


private:

	/**
	* The Singleton's constructor/destructor should always be private to
	* prevent direct construction/desctruction calls with the `new`/`delete`
	* operator.
	*/
	static SystemState * pinstance_;
	static std::mutex mutex_;

	std::mutex kbMtx;						//!< Mutex on the keyboard resource
	int action;								//!< User action
	int show;								//!< Show function 
	bool ftBiasResetRequest;				//!< Flag to request the reset of the F/T sensor bias
	std::string logPath;					//!< Path where the log is stored
	TaskInfo task_i;						//!< Struct with information about task
	ControlInfo ctrl_i;						//!< Struct with information about controller
	SensorInfo sens_i;						//!< Struct with information about connected sensors
	ConstraintInfo constr_i;				//!< Struct with information about kinematic constraints
											// ...

};




#endif // SYSTEM_STATE_HPP_
