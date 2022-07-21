#ifndef VREPPROXY_HPP_
#define VREPPROXY_HPP_

// System Header files
#include <map>
#include <vector>

// Eigen Header files
#include <Eigen/Dense>

// V-REP header files
extern "C" {
#include "extApi.h"
}

//Project Header files
#include "ExtSystemProxy.hpp"

// Space dimension is 3...
#ifndef SPACE_DIM
#define SPACE_DIM 3
#endif // SPACE_DIM

#define SIM_TIMESTEP 0.05

namespace Eigen {
	
	typedef Eigen::Matrix<float, 6, 1> Vector6f;
}

class VREPProxy : public ExtSystemProxy {

public:

	/**
	* @brief Default contructor of VREPProxy class
	*
	*/
	VREPProxy();


	/* @brief Default destroyer of VREPProxy class
	*
	*/
	~VREPProxy();

	/**
	* @brief Copy constructor of the VREPProxy class
	* @param vp the VREPProxy
	*/
	VREPProxy(VREPProxy& vp);

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
	* @brief Set function
	* Set the IP address to connect to the V-REP simulator
	* @param ipaddr the IP address
	*/
	inline void setIPAddress(const std::string& ipaddr) { this->IPaddr = ipaddr; }
	
	/**
	* @brief Get function
	* Retrieves the IP address used to connect to the V-REP simulator
	* @return the IP address
	*/
	inline std::string getIPAddress() { return this->IPaddr; }

	/**
	* @brief Get function
	* Retrieves the list of simulated objects name used during the simulation
	* @return the list of object names
	*/
	inline std::map < std::string, int > getSimObjects() { return this->simObjects; }

	/**
	* @brief Start simulation function
	* Play the simulation in the currently opened V-REP scene
	* @param the connection port of the assigned server (default is 19997)
	*/
	void startSim(const int& port = 19997);

	/**
	* @brief Stop simulation function
	* Stop the simulation in the currently opened V-REP scene
	* @param the connection port of the assigned server (default is 19997)
	*/
	void stopSim(const int& port = 19997);

	/**
	* @brief Set function
	* Set the pose for the V-REP object with name objName
	* @param objName the name of the V-REP object
	* @param T the transformation matrix
	* @param relObjName the name of the V-REP object with respect to which the pose has to be set
	* @param mode the remote API call mode
	* @param the connection port of the assigned server (default is 19997)
	*/
	void VREPProxy::setObjectPose(const char* objName, const Eigen::Matrix4d& T, const char* relObjName, const int& mode, const int& port = 19997);

	/**
	* @brief Set function
	* Set the position pos for the V-REP object with name objName
	* @param objName the name of the V-REP object
	* @param pos the position vector
	* @param mode the remote API call mode
	* @param the connection port of the assigned server (default is 19997)
	*/
	void setObjectPosition(const char* objName, const float* pos, const char* relObjName = "world", const int& mode = simx_opmode_oneshot, const int& port = 19997);

	/**
	* @brief Set function
	* Set the quaternion quat for the V-REP object with name objName
	* @param objName the name of the V-REP object
	* @param quaternion the quaternion vector (convention: x y z w)
	* @param relObjName the name of the V-REP object with respect to which the quaternion has to be set
	* @param mode the remote API call mode
	* @param the connection port of the assigned server (default is 19997)
	*/
	void setObjectQuaternion(const char* objName, const float* quat, const char* relObjName = "world", const int& mode = simx_opmode_oneshot, const int& port = 19997);

	/**
	* @brief Get function
	* Get the position pos for the V-REP object with name objName
	* @param objName the name of the V-REP object
	* @param pos the position vector [output]
	* @param mode the remote API call mode
	* @param the connection port of the assigned server (default is 19997)
	*/
	void getObjectPosition(const char* objName, float* pos, const char* relObjName = "world", const int& mode = simx_opmode_oneshot, const int& port = 19997);

	/**
	* @brief Get function
	* Set the quaternion quat for the V-REP object with name objName
	* @param objName the name of the V-REP object
	* @param quaternion the quaternion vector (convention: x y z w) [output]
	* @param relObjName the name of the V-REP object with respect to which the quaternion has to be set
	* @param mode the remote API call mode
	* @param the connection port of the assigned server (default is 19997)
	*/
	void getObjectQuaternion(const char* objName, float* quat, const char* relObjName = "world", const int& mode = simx_opmode_oneshot, const int& port = 19997);

	/**
	* @brief Set function
	* Set the position value q of the joint with name objName 
	* This function requires the V-REP dynamics engine to be NOT ENABLED.
	* If the dynamics engine is enabled, see setJointTargetPosition instead.
	* @param objName the name of the joint object in the V-REP scene
	* @param q the joint position value
	* @param the connection port of the assigned server (default is 19997)
	*/
	void setJointPosition(const char* objName, const float& q, const int& = 19997);

	/**
	* @brief Set function
	* Set the target position value q of the joint with name objName
	* This function requires the V-REP dynamics engine to be ENABLED
	* If the dynamics engine is not enabled, see setJointPosition instead.
	* @param objName the name of the joint object in the V-REP scene
	* @param q the joint position value
	* @param the connection port of the assigned server (default is 19997)
	*/
	void setJointTargetPosition(const char* objName, const float& q, const int& port = 19997);

	/**
	* @brief Set function
	* Set the target velocity value qdot of the joint with name objName
	* @param objName the name of the joint object in the V-REP scene
	* @param qdot the joint target velocity value
	* @param the connection port of the assigned server (default is 19997)
	*/
	void setJointTargetVelocity(const char* objName, const float& qdot, const int& port = 19997);

	/**
	* @brief Get function
	* Get the position value q of the joint with name objName
	* @param objName the name of the joint object in the V-REP scene
	* @param op_flag the operation mode flag
	* @param the connection port of the assigned server (default is 19997)
	* @return q the joint position value
	*/
	float getJointPosition(const char* objName, const int& op_flag, const int& port = 19997);

	/**
	* @brief Set function
	* Set the value of the floating signal with name signal
	* @param signal name of the signal
	* @param value value of the signal
	* @param op_flag the operation mode flag
	* @param the connection port of the assigned server (default is 19997)
	*/
	void setFloatSignal(const char* signal, const float& value, const int& op_flag, const int& port = 19997);


	/**
	* @brief Get function
	* Get the value of the floating signal with name signal
	* @param signal name of the signal
	* @param value value of the signal
	* @param op_flag the operation mode flag
	* @param the connection port of the assigned server (default is 19997)
	*/
	void VREPProxy::getFloatSignal(const char* signal, float& value, const int& op_flag, const int& port = 19997);

	/**
	* @brief Read function
	* Read the current measurement of the simulated force sensor in V-REP
	* @param objName the name of the force sensor object in V-REP
	* @param state the output state of the simulated force sensor
	* @param op_flag the operation mode flag
	* @param port the connection port of the assigned server (default is 19997)
	* @return the 6D vector containing the force and torque simulated measurement
	*/
	Eigen::Vector6f readForceSensor(const char* objName, int& state, const int& op_flag, const int& port = 19997);

	/**
	* @brief Clear function
	* Clear the floating signal with name signal
	* @param signal name of the signal
	* @param op_flag the operation mode flag
	* @param the connection port of the assigned server (default is 19997)
	*/
	void clearFloatSignal(const char* signal, const int& op_flag, const int& port = 19997);

	/**
	* @brief Load function
	* Load the names of the objects that have to be controlled in the V-REP simulation
	* @param names the vector of names
	* @param the connection port of the assigned server (default is 19997)
	*/
	void loadNames(const std::vector < std::string >& names, const int& port = 19997);

	/**
	* @brief Set function
	* Set the flag for the synchronous mode
	* @param sync the flag for the synchronous mode
	* @param the connection port of the assigned server (default is 19997)
	*/
	void setSynchMode(const bool& sync, const int& port = 19997);

	/**
	* @brief Trigger function
	* Trigger the next simulation step
	* @param the connection port of the assigned server (default is 19997)
	*/
	void triggerSim(const int& port = 19997);

	/**
	* @brief Set function
	* States if the dynamics engine of the V-REP simulator should be enabled or not
	* @param dyn: true if the dynamics engine has to be enabled, false otherwise
	* @param the connection port of the assigned server (default is 19997)
	*/
	void enableDynamicEngine(const bool& dyn, const int& port = 19997);

	/**
	* @brief Get function
	* Check if the dynamics engine of the V-REP simulator should be enabled or not
	* @param the connection port of the assigned server (default is 19997)
	* @return true if the dynamics engine has to be enabled, false otherwise
	*/
	bool isDynamicEngineEnabled(const int& port = 19997);

	/**
	* @brief Debug function
	* Print the names of all the V-REP objects loaded for the simulation
	*/
	void printObjectSNameSet();

	/**
	* @brief Set function
	* Create a dummy object in the V-REP scene and set to the desired position
	* @param pos: the position in which the dummy object has to be set
	*/
	void showPoint(const float* pos,const int& port = 19997);

private:

	std::string IPaddr;							//!< IP address of the V-REP simulator server
	std::map < int, int > portIdMap;			//!< Map of <port,id> V-REP clients
	std::map < std::string, int > simObjects;	//!< Map of <name,id> V-REP objects

	/**
	* @brief Init function
	* @param id: the remote client id for the connection with the V-REP server
	*/
	void initObjectHandles(const int& id);

};


#endif // VREPREMOTESERVICE_HPP_
