// Project Header files
#include "VREPProxy.hpp"
#include "Timer.hpp"
#include "utils.hpp"

/**
* @brief Default contructor of VREPProxy class
*
*/
VREPProxy::VREPProxy() : ExtSystemProxy() {

	this->IPaddr = "127.0.0.1";
	
	this->available = false;
	this->running = false;

}

/**
* @brief Default destroyer of VREPProxy class
*
*/
VREPProxy::~VREPProxy() {}

/**
* @brief Copy constructor of the VREPProxy class
* @param vp the VREPProxy
*/
VREPProxy::VREPProxy(VREPProxy& vp) {

}


/**
* @brief Default init function
*/
void VREPProxy::init() {

	// Debug print
	debugPrint<char>("\n[VREPProxy] This is a debug print", NULL);

	this->portIdMap.insert({ 19997, -1 });
	this->portIdMap.insert({ 19996, -1 });
	this->portIdMap.insert({ 19995, -1 });//*/

	// Close eventual pending open communication channels
	simxFinish(-1);

	// Try to open a new connection with V-REP (timeout at 5s)
	//this->clientID = simxStart((simxChar*)this->IPaddr.c_str(), this->port, true, true, 50, 5);
	std::map< int, int >::iterator it;
	bool avail = true;
	for (it = this->portIdMap.begin(); it != this->portIdMap.end(); ++it) {
		it->second = simxStart((simxChar*)this->IPaddr.c_str(), it->first, true, true, 50, 5);

		debugPrint<std::string>("\nVREP IP address:", this->IPaddr);
		debugPrint<int>(std::string(std::string("V-REP client ID for the server n. ") + std::to_string(it->first)).c_str(), it->second);

		if (it->second != -1) {
			debugPrint<char>("\nV-REP simulator is successfully connected! ", NULL);
			this->availability(true && avail);
		}
		else {
			debugPrint<char>("\nCould not connect to V-REP. Check the V-REP settings... ", NULL);
			this->availability(false);
		}

		avail = avail && this->isAvailable();
		debugPrint<bool>("[VREP] avail = ", avail);
	}


}


/**
* @brief Load function
* Load the names of the objects that have to be controlled in the V-REP simulation
* @param names the vector of names
* @param the connection port of the assigned server (default is 19997)
*/
void VREPProxy::loadNames(const std::vector < std::string >& names, const int& port){


	// Fill the key value of the object map with the input names
	int N = names.size();
	for (int i = 0; i < N; i++) {
		this->simObjects.insert({ names[i],-1 });
	}

	// Init the object handles
	this->initObjectHandles(this->portIdMap[port]);

}

/**
* @brief Init function
* Init the handles of the V-REP objects loaded from the set of names
* @param id: the remote client id for the connection with the V-REP server
*/
void VREPProxy::initObjectHandles(const int& id) {


	std::map< std::string, int >::iterator it;
	for (it = this->simObjects.begin(); it != this->simObjects.end(); ++it) {

		int hdl;
		simxGetObjectHandle(id, (it->first).c_str(), &hdl, simx_opmode_blocking);
		it->second = hdl;

		//debugPrint<int>(std::string(std::string("\nGetting handle of the object ") + it->first + std::string(": ")).c_str(), it->second);
	}

}


/**
* @brief Default run function
*/
void VREPProxy::run() {

	// Required variables

	// Time variables
	double dt, et, rate, tic, toc, tac, Ts;
	Timer clock;

	// Get the clock rate
	rate = clock.getRate();
	Ts = 1.0 / rate;

	while (this->isRunning()) {

		// Measure starting time
		tic = clock.getCurTime();

		//----------------------------------------------------------------//
		// Do stuff here... 
		//----------------------------------------------------------------//

		// Measure the ending time and the elapsed time
		toc = clock.getCurTime();
		dt = clock.elapsedTime(tic, toc);

		// Wait until Ts
		if (dt < Ts) {
			clock.timeSleep(Ts - dt);
		}

		// Measure the final time after sleep to check the actual rate of the thread
		tac = clock.getCurTime();
		et = clock.elapsedTime(tic, tac);

		//debugPrint<double>("[VREPProxy] Running rate", 1.0 / et);
	}

}

/**
* @brief Default clear function
*/
void VREPProxy::clear() {

	// Start the V-REP simulation
	std::cout << "Stopping simulation ... ";
	this->stopSim();
	std::cout << " done." << std::endl;

	// Call the corresponding remote V-REP API to close the communication with V-REP
	std::map< int, int >::iterator it;
	for (it = this->portIdMap.begin(); it != this->portIdMap.end(); ++it) {
		simxFinish(it->second);
	}

	// Set the availability flag to false
	this->availability(false);
	this->setRunning(false);

}


/**
* @brief Start simulation function
* Play the simulation in the currently opened V-REP scene
* @param the connection port of the assigned server (default is 19997)
*/
void VREPProxy::startSim(const int& port){

	// Call the corresponding remote V-REP API to start the simulation
	simxStartSimulation(this->portIdMap[port], simx_opmode_blocking);

	// Set running flag on true
	this->setRunning(true);

}

/**
* @brief Stop simulation function
* Stop the simulation in the currently opened V-REP scene
* @param the connection port of the assigned server (default is 19997)
*/
void VREPProxy::stopSim(const int& port) {

	// Call the corresponding remote V-REP API to stop the simulation
	simxStopSimulation(this->portIdMap[port], simx_opmode_blocking);


}




/**
* @brief Set function
* Set the pose for the V-REP object with name objName
* @param objName the name of the V-REP object
* @param T the transformation matrix 
* @param relObjName the name of the V-REP object with respect to which the pose has to be set
* @param mode the remote API call mode
* @param the connection port of the assigned server (default is 19997)
*/
void VREPProxy::setObjectPose(const char* objName, const Eigen::Matrix4d& T, const char* relObjName, const int& mode, const int& port) {

	float p_[SPACE_DIM];
	float q_[4];

	if (this->isAvailable()) {

		// Set on V-REP
		Eigen::Vector3f eigp = (T.block<3, 1>(0, 3)).cast<float>();
		Eigen::Matrix3d Rkp = T.topLeftCorner(SPACE_DIM, SPACE_DIM);
		Eigen::Quaternionf q(Rkp.cast<float>());
		std::memcpy(p_, eigp.data(), sizeof(float)*SPACE_DIM);

		q_[0] = q.coeffs()(0);
		q_[1] = q.coeffs()(1);
		q_[2] = q.coeffs()(2);
		q_[3] = q.coeffs()(3);//*/

		// Set position
		this->setObjectPosition("_Phantom", p_, "KukaOrigin", mode, port);

		// Wait for a while
		Timer clock;
		clock.timeSleep(0.5);

		// Set orientation
		this->setObjectQuaternion("_Phantom", q_, "KukaOrigin", mode, port);

	}

}


/**
* @brief Set function
* Set the position pos for the V-REP object with name objName
* @param objName the name of the V-REP object
* @param pos the position vector
* @param relObjName the name of the V-REP object with respect to which the position has to be set
* @param mode the remote API call mode
* @param the connection port of the assigned server (default is 19997)
*/
void VREPProxy::setObjectPosition(const char* objName, const float* pos, const char* relObjName, const int& mode, const int& port) {

	int hdl = this->simObjects[std::string(objName)];
	int ref = (std::string(relObjName).compare("world") == 0) ? (-1) : (this->simObjects[std::string(relObjName)]);

	// Set position
	int ret = simxSetObjectPosition(this->portIdMap[port], hdl, ref, pos, mode);

}


/**
* @brief Set function
* Set the quaternion quat for the V-REP object with name objName
* @param objName the name of the V-REP object
* @param quaternion the quaternion vector (convention: x y z w)
* @param relObjName the name of the V-REP object with respect to which the quaternion has to be set
* @param mode the remote API call mode
* @param the connection port of the assigned server (default is 19997)
*/
void VREPProxy::setObjectQuaternion(const char* objName, const float* quat, const char* relObjName, const int& mode, const int& port) {

	int hdl = this->simObjects[std::string(objName)];
	int ref = (std::string(relObjName).compare("world") == 0) ? (-1) : (this->simObjects[std::string(relObjName)]);

	// Set position
	int ret = simxSetObjectQuaternion(this->portIdMap[port], hdl, ref, quat, mode);
}


/**
* @brief Get function
* Get the position pos for the V-REP object with name objName
* @param objName the name of the V-REP object
* @param pos the position vector [output]
* @param mode the remote API call mode
* @param the connection port of the assigned server (default is 19997)
*/
void VREPProxy::getObjectPosition(const char* objName, float* pos, const char* relObjName, const int& mode, const int& port) {

	int hdl = this->simObjects[std::string(objName)];
	float pos_[SPACE_DIM];
	int ref = (std::string(relObjName).compare("world") == 0) ? (-1) : (this->simObjects[std::string(relObjName)]);

	int ret = simxGetObjectPosition(this->portIdMap[port], hdl, ref, pos, mode);

}

/**
* @brief Get function
* Set the quaternion quat for the V-REP object with name objName
* @param objName the name of the V-REP object
* @param quaternion the quaternion vector (convention: x y z w) [output]
* @param relObjName the name of the V-REP object with respect to which the quaternion has to be set
* @param mode the remote API call mode
* @param the connection port of the assigned server (default is 19997)
*/
void VREPProxy::getObjectQuaternion(const char* objName, float* quat, const char* relObjName, const int& mode, const int& port) {

	int hdl = this->simObjects[std::string(objName)];
	int ref = (std::string(relObjName).compare("world") == 0) ? (-1) : (this->simObjects[std::string(relObjName)]);

	simxGetObjectQuaternion(this->portIdMap[port], hdl, ref, quat, mode);

}

/**
* @brief Set function
* Set the position value q of the joint with name objName
* This function requires the V-REP dynamics engine to be NOT ENABLED.
* If the dynamics engine is enabled, see setJointTargetPosition instead.
* @param objName the name of the joint object in the V-REP scene
* @param q the joint position value
* @param the connection port of the assigned server (default is 19997)
*/
void VREPProxy::setJointPosition(const char* objName, const float& q, const int& port) {

	simxSetJointPosition(this->portIdMap[port], this->simObjects[objName], q, simx_opmode_oneshot);

}

/**
* @brief Set function
* Set the target position value q of the joint with name objName
* This function requires the V-REP dynamics engine to be ENABLED
* If the dynamics engine is not enabled, see setJointPosition instead.
* @param objName the name of the joint object in the V-REP scene
* @param q the joint position value
* @param the connection port of the assigned server (default is 19997)
*/
void VREPProxy::setJointTargetPosition(const char* objName, const float& q, const int& port) {

	simxSetJointTargetPosition(this->portIdMap[port], this->simObjects[objName], q, simx_opmode_oneshot);

}

/**
* @brief Set function
* Set the target velocity value qdot of the joint with name objName
* @param objName the name of the joint object in the V-REP scene
* @param qdot the joint target velocity value
* @param the connection port of the assigned server (default is 19997)
*/
void VREPProxy::setJointTargetVelocity(const char* objName, const float& qdot, const int& port) {

	simxSetJointTargetVelocity(this->portIdMap[port], this->simObjects[objName], qdot, simx_opmode_oneshot);

}


/**
* @brief Get function
* Get the position value q of the joint with name objName
* @param objName the name of the joint object in the V-REP scene
* @param op_flag the operation mode flag
* @param the connection port of the assigned server (default is 19997)
* @return q the joint position value
*/
float VREPProxy::getJointPosition(const char* objName, const int& op_flag, const int& port) {

	float q;

	int ret = simxGetJointPosition(this->portIdMap[port], this->simObjects[objName], &q, op_flag);

	if (port == 19995) {
		//debugPrint<int>("\n[Teleop in VREPProxy] client ID = ", this->portIdMap[port]);
		//debugPrint<int>("\n[Teleop in VREPProxy] ret = ", ret);
		//debugPrint<float>("\n[Teleop in VREPProxy] q(i) = ", q);
	}
	return q;

}

/**
* @brief Set function
* Set the value of the floating signal with name signal
* @param signal name of the signal
* @param value value of the signal
* @param op_flag the operation mode flag
* @param the connection port of the assigned server (default is 19997)
*/
void VREPProxy::setFloatSignal(const char* signal, const float& value, const int& op_flag, const int& port) {

	simxSetFloatSignal(this->portIdMap[port], signal, value, op_flag);

}

/**
* @brief Get function
* Get the value of the floating signal with name signal
* @param signal name of the signal
* @param value value of the signal
* @param op_flag the operation mode flag
* @param the connection port of the assigned server (default is 19997)
*/
void VREPProxy::getFloatSignal(const char* signal, float& value, const int& op_flag, const int& port) {

	simxGetFloatSignal(this->portIdMap[port], signal, &value, op_flag);
}

/**
* @brief Read function
* Read the current measurement of the simulated force sensor in V-REP
* @param objName the name of the force sensor object in V-REP
* @param state the output state of the simulated force sensor
* @param op_flag the operation mode flag
* @param port the connection port of the assigned server (default is 19997)
* @return the 6D vector containing the force and torque simulated measurement
*/
Eigen::Vector6f VREPProxy::readForceSensor(const char* objName, int& state, const int& op_flag, const int& port) {

	Eigen::Vector6f out;
	float force[3];
	float torque[3];

	simxReadForceSensor(this->portIdMap[port], this->simObjects[objName], (simxUChar*)(&state), force, torque, op_flag);

	for (int i = 0; i < SPACE_DIM; i++) {
		out(i) = force[i];
		out(i + 3) = torque[i];
	}

	return out;

}


/**
* @brief Clear function
* Clear the floating signal with name signal
* @param signal name of the signal
* @param op_flag the operation mode flag
* @param the connection port of the assigned server (default is 19997)
*/
void VREPProxy::clearFloatSignal(const char* signal, const int& op_flag, const int& port) {

	simxClearFloatSignal(this->portIdMap[port], signal, op_flag);

}

/**
* @brief Set function
* Set the flag for the synchronous mode
* @param sync the flag for the synchronous mode
* @param the connection port of the assigned server (default is 19997)
*/
void VREPProxy::setSynchMode(const bool& sync, const int& port) {

	simxSynchronous(this->portIdMap[port], sync);

}

/**
* @brief Trigger function
* Trigger the next simulation step
* @param the connection port of the assigned server (default is 19997)
*/
void VREPProxy::triggerSim(const int& port) {

	simxInt ping;
	simxSynchronousTrigger(this->portIdMap[port]);
	simxGetPingTime(this->portIdMap[port],&ping);
}


/**
* @brief Set function
* States if the dynamics engine of the V-REP simulator should be enabled or not
* @param dyn: true if the dynamics engine has to be enabled, false otherwise
* @param the connection port of the assigned server (default is 19997)
*/
void VREPProxy::enableDynamicEngine(const bool& dyn, const int& port){

	simxSetBooleanParameter(this->portIdMap[port], sim_boolparam_dynamics_handling_enabled, dyn, simx_opmode_blocking);

}


/**
* @brief Get function
* Check if the dynamics engine of the V-REP simulator should be enabled or not
* @param the connection port of the assigned server (default is 19997)
* @return true if the dynamics engine has to be enabled, false otherwise
*/
bool VREPProxy::isDynamicEngineEnabled(const int& port) {

	simxUChar ret;

	simxGetBooleanParameter(this->portIdMap[port], sim_boolparam_dynamics_handling_enabled, &ret, simx_opmode_oneshot);

	return ((ret == 1) ? true : false);

}


/**
* @brief Debug function
* Print the names of all the V-REP objects loaded for the simulation
*/
void VREPProxy::printObjectSNameSet() {

	std::map < std::string, int >::iterator it;
	for (it = this->simObjects.begin(); it != this->simObjects.end(); ++it) {
		//debugPrint<std::string>("\nLoaded object with name ", it->first);
	}

}


/**
* @brief Set function
* Create a dummy object in the V-REP scene and set to the desired position
* @param pos: the position in which the dummy object has to be set
*/
void VREPProxy::showPoint(const float* pos, const int& port) {

	simxInt dummyHandle;

	// Create the dummy
	simxCreateDummy(this->portIdMap[port], 0.005, NULL, &dummyHandle, simx_opmode_blocking);

	// Set the position of the dummy
	simxSetObjectPosition(this->portIdMap[port], dummyHandle, this->simObjects["KukaOrigin"], pos, simx_opmode_blocking);

}
