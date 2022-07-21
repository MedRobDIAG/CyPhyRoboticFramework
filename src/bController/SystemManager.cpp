// Standard Header files
#include <iostream>

// Project Header files
#include "SystemManager.hpp"
#ifdef WITH_WEART
#include "WeartProxy.hpp"
#endif //WITH_WEART

#include "RobotProxy.hpp"
#ifdef WITH_FRI
#include "KukaProxy.hpp"
#endif // WITH_FRI
#ifdef WITH_FORCE_SENSOR
#include "crlFTSensorWrapper.hpp"
#endif // WITH_FORCE_SENSOR
#include "VREPProxy.hpp"
#ifdef WITH_GEOMAGIC
#include "GeomagicProxy.hpp"
#endif // WITH_GEOMAGIC
#ifdef WITH_ARMBAND
#include "ArmbandProxy.hpp"
#endif // WITH_ARMBAND
#ifdef WITH_HRING
#include "HRingProxy.hpp"
#endif // WITH_HRING

#include "KUKARobot.hpp"
#include "Geomagic.hpp"
#include "Armband.hpp"
#include "HapticInterface.hpp"
#include "Phantom.hpp"
#include "FTSensor.hpp"
#include "Dataset.hpp"
#include "AbdomenPhantom.hpp"
#include "MultiLayerGelPhantom.hpp"
#include "HRing.hpp"
#include "WeartDevice.hpp"

/**
* Static methods should be defined outside the class.
*/
SystemState* SystemState::pinstance_{ nullptr };
std::mutex SystemState::mutex_;

/**
* Static methods should be defined outside the class.
*/
Configuration* Configuration::pinstance_{ nullptr };
std::mutex Configuration::mutex_;

/**
* Static methods should be defined outside the class.
*/
Dataset* Dataset::pinstance_{ nullptr };
std::mutex Dataset::mutex_;
std::mutex Dataset::idxMtx;


/**
* @brief Default constructor of the SystemManager object
*
*/
SystemManager::SystemManager() {}

/**
* @brief Default destroyer of the SystemManager object
*
*/
SystemManager::~SystemManager() {

	// Delete dynamic pointers to proxies and instruments
	/* Proxies */
	std::map < std::string, ExtSystemProxy* >::iterator proxies_it; //!< Map of proxies objects

	/* Instruments */
	std::map < std::string, Instrument* >::iterator instruments_it; //!< Map of proxies objects

	// Delete proxy pointers
	for (proxies_it = this->proxies.begin(); proxies_it != this->proxies.end(); proxies_it++) {
		delete proxies_it->second;
	}

	// Delete Instruments pointers
	for (instruments_it = this->instruments.begin(); instruments_it != this->instruments.end(); instruments_it++) {
		delete instruments_it->second;
	}

}

/**
* @brief Init function
*/
void SystemManager::init() {

	this->exit_loop = false;
	this->started = false;
	this->accomplished = false;
	this->time_ = 0.0;

	// Initialize the singleton SystemState and Configuration classes
	// SystemState
	this->systemstate = SystemState::GetInstance();

	// Configuration
	this->config = Configuration::GetInstance("config/Configuration.txt"); // Input filename string is useless here, as the singleton class is supposed to be already instantiated by SystemManager

	/// Initialize the singleton SystemState and Configuration classes 
	/// (this initialization is done onlye here, not in other classes)
	// SystemState
	this->systemstate->setTaskState(UNDEFINED_STATE);
	this->systemstate->setTaskType(UNDEFINED_TASK);
	this->systemstate->setControlMode(UNDEFINED_CONTROL);
	this->systemstate->setControlState(UNDEFINED_STATE);
	this->systemstate->setHapticDeviceNum(0);
	this->systemstate->setRobotTypeName("");

	// Configuration
	this->config->loadFromFile();

	// Let SystemManager initialize the list of proxies
	this->initProxies();

	// Let SystemManager initialize the list of instruments
	this->initInstruments();

	// Set initial robot pose
	this->setInitialRobotPose();

	/// Let SystemManager initialize the rest of BL classes
	// Init task
	this->initBLInstance(&this->task);

	// Init controller
	this->initBLInstance(&this->ctrl);

	// Init logger
	this->initBLInstance(&this->log);
}

/**
* @brief Set function
* Set and initialize the proxies objects based on the initial settings of the Configuration object
*/
void SystemManager::initProxies() {

	// Local instance of VREPProxy
	VREPProxy *vp = nullptr;

	std::vector < std::string > hapticList = config->getHapticDeviceList();

#ifdef WITH_GEOMAGIC
	GeomagicProxy* geoPtr = nullptr;

	std::vector < std::string >::iterator geoHapticIt = std::find(hapticList.begin(), hapticList.end(), std::string("Geomagic"));
	bool geoHapticFound = geoHapticIt != hapticList.end();
	if (geoHapticFound) {
		std::cout << "Found Geomagic in the list!" << std::endl;
	}
	else {
		std::cout << "Couldn't find Geomagic in the list!" << std::endl;
	}


	// Create a dynamic instance of the Geomagic proxy
	// TODO: If a different interface is used in the future for teleoperation, you must generalize here
	//if (config->getHapticDeviceType() == "Geomagic" || config->getTeleopDeviceType() == "Geomagic") {
	if (geoHapticFound || config->getTeleopDeviceType() == "Geomagic") {
		geoPtr = new GeomagicProxy();

		// Set the Geomagic proxy implementing both teleop and haptic functionalities
		this->proxies.insert({ "GeomagicProxy", geoPtr });
	}
#endif // WITH_GEOMAGIC


	// vvv ++++ CASE WITH ONE SINGLE HAPTIC DEVICE ++++++ vvvv //
	/// Instantiate the appropriate proxies, based on the specifications of the Configuration file
	// HAPTIC DEVICE PROXY
	/*if (config->getHapticDeviceType() == "Geomagic") {

		// Instantiate inner GeomagicHapticProxy
		GeomagicHapticProxy* geoHaptic = geoPtr->instantiateHaptic();
		geoPtr->enableHaptics(true);

		this->proxies.insert({ "HapticProxy", geoHaptic });
		std::cout << "Setting Geomagic Haptic Proxy class ... " << std::endl;
	}
	else if (config->getHapticDeviceType() == "Armband"){
		this->proxies.insert({"HapticProxy", new ArmbandProxy()});
		std::cout << "Setting Armband Proxy class ... " << std::endl;
	}
	else {
		std::cout << "Impossible to set any Haptic Proxy class. " << std::endl;
		// Add analogous lines if you add other types of haptic devices
		// ...
	}//*/
	// ^^^ ++++ CASE WITH ONE SINGLE HAPTIC DEVICE ++++++ ^^^ //


	// vvv ++++ CASE WITH LIST OF HAPTIC DEVICES ++++++ vvv //
	for (int i = 0; i < hapticList.size(); i++) {
		std::string proxyName;
#ifdef WITH_GEOMAGIC
		if (hapticList[i] == "Geomagic") {
			std::cout << "Found Geomagic in hapticList " << std::endl;
			// Instantiate inner GeomagicHapticProxy
			GeomagicHapticProxy* geoHaptic = geoPtr->instantiateHaptic();
			geoPtr->enableHaptics(true);

			proxyName = std::string("HapticProxy") + std::to_string(i + 1);
			this->proxies.insert({ proxyName.c_str(), geoHaptic });
			std::cout << "Setting Geomagic Haptic Proxy class ... " << std::endl;
		}
#endif // WITH_GEOMAGIC
#ifdef WITH_ARMBAND
		if (hapticList[i] == "Armband") {
			std::cout << "Found Armband in hapticList " << std::endl;

			proxyName = std::string("HapticProxy") + std::to_string(i + 1);
			this->proxies.insert({ proxyName, new ArmbandProxy() });
			std::cout << "Setting Armband Proxy class ... " << std::endl;
		}
#endif // WITH_ARMBAND
#ifdef WITH_HRING
		if (hapticList[i] == "HRing") {
			std::cout << "Found HRing in hapticList " << std::endl;

			proxyName = std::string("HapticProxy") + std::to_string(i + 1);
			this->proxies.insert({ proxyName, new HRingProxy() });
			std::cout << "Setting HRing Proxy class ... " << std::endl;
		}
#endif // WITH_HRING
#ifdef WITH_WEART
		if (hapticList[i] == "TouchDIVER") {
			std::cout << "Found TouchDIVER in hapticList " << std::endl;

			proxyName = std::string("HapticProxy") + std::to_string(i + 1);
			this->proxies.insert({ proxyName, new WeartProxy() });
			std::cout << "Setting Weart Proxy class ... " << std::endl;
		}
#endif // WITH_WEART
	}
	// ^^^ ++++ CASE WITH LIST OF HAPTIC DEVICES ++++++ ^^^ //


	// TELEOP DEVICE PROXY
#ifdef WITH_GEOMAGIC
	if (config->getTeleopDeviceType() == "Geomagic") {

		GeomagicTeleopProxy* geoTeleop = geoPtr->instantiateTeleop();
		geoPtr->enableTeleop(true);

		this->proxies.insert({ "TeleopProxy", geoTeleop });
		std::cout << "Setting Geomagic Teleop Proxy class ... " << std::endl;

	}
#endif //WITH_GEOMAGIC

	if (config->getControlFlag() == TELEOPERATION_MODE) {

		// Update SystemState accordingly
		this->systemstate->setControlMode(CONTROL::UNDEFINED_CONTROL);
		this->systemstate->setControlState(STATE::UNDEFINED_STATE);
		this->systemstate->addSenorName(config->getHapticDeviceType());
		std::cout << "Requested TELEOPERATION_MODE control from Configuration file." << std::endl;
	}
	
	// ROBOT MANIPULATOR PROXY
#ifdef WITH_FRI
	if (config->isRealRobotRequested()) {

		if (config->getRobotManipulatorType() == "KUKA") {
			this->proxies.insert({ "RobotProxy", new KukaProxy()} );
		}
		else {
			// Add analogous lines if you add other types of haptic devices
			// ...
		}

		// Update SystemState accordingly
		this->systemstate->addSenorName(config->getRobotManipulatorType());
		this->systemstate->setRobotTypeName(config->getRobotManipulatorType());
	}
#endif // WITH_FRI

	// SIMULATOR SOFTWARE PROXY
	if (config->isSimulatorRequested()) {
		if (config->getSimulatorType() == "V-REP") {
			this->proxies.insert({ "SimulatorProxy",new VREPProxy});
			vp = (VREPProxy*)(this->proxies["SimulatorProxy"]);
		}
	}

	// FORCE SENSOR PROXY
#ifdef WITH_FORCE_SENSOR
	if (config->isForceSensorRequested() && !config->isForceSensorSimulated()) {
		this->proxies.insert({ "ForceSensorProxy", new crlFTSensorWrapper() });
		this->systemstate->addSenorName("ForceSensorProxy");
	}
#endif // WITH_FORCE_SENSOR

	// Run the init() method for each instantiated proxy
	std::map< std::string, ExtSystemProxy*>::iterator it;
	if (proxies.size() > 0) {
		for (it = this->proxies.begin(); it != this->proxies.end(); ++it) {
			std::cout << "Initializing " << it->first << " proxy ..." << std::endl;
			it->second->init();
		}
	}

	// If V-REP is present, initialize the set of objects and the settings for the simulation.
	// Finally, run the simulation
	if (config->isSimulatorRequested()) {
		if (config->getSimulatorType() == "V-REP") {

			// Init the V-REP common structures for Scheduler, Task and Controller
			// Set the connection port for the scheduler
			this->setSimPort(19997);

			// Load V-REP object handles from input names
			std::vector < std::string > simNames;
			simNames = config->getVREPObjNames();
			vp->loadNames(simNames);

			// Set V-REP synchronous mode
			vp->setSynchMode(0);

			// Set the V-REP dynamics engine
			vp->enableDynamicEngine(config->isDynSimulationRequested());

			// Start the V-REP simulation
			vp->startSim();
		}
	}


}


/**
* @brief Instrument init function
* Initialize the instruments to be shared with Task and Controller objects
*/
void SystemManager::initInstruments() {

	// Define local pointers 
#ifdef WITH_FRI
	RobotProxy* rpPtr = nullptr;
#endif // WITH_FRI
	RobotInterface* robotPtr = nullptr;
	Instrument* geoPtr = nullptr;

	/// Create instances of Instruments
	// 1. Create a RobotInterface dynamic instance
	if (config->getRobotManipulatorType() == "KUKA") {
		this->instruments.insert({ "Robot", new KUKARobot() });
	}
	else {
		// Add analogous lines for different robots
		// ...
	}
	robotPtr = dynamic_cast<RobotInterface*>(this->instruments["Robot"]);
	if (robotPtr == nullptr) {
		std::cout << "robotPtr is a null pointer " << std::endl;
		std::cout << robotPtr << std::endl;
	}

	// 2. Create a Geomagic dynamic instance (if teleoperation is required)
	std::string hapticStringName = config->getHapticDeviceType();
	std::string teleopStringName = config->getTeleopDeviceType();


	// vvv ++++ CASE WITH ONE SINGLE HAPTIC DEVICE ++++++ vvvv //
	/*if (hapticStringName.find("Geomagic") != std::string::npos || teleopStringName.find("Geomagic") != std::string::npos) {
		this->instruments.insert({ "Geomagic" ,  new Geomagic()});
		geoPtr = this->instruments["Geomagic"];
		std::cout << "Instantiating Geomagic class as Haptic interface... " << std::endl;
	}
	if (hapticStringName.find("Armband") != std::string::npos) {
		this->instruments.insert({ hapticStringName.c_str() ,  new Armband() });
		std::cout << "Instantiating Armband class as Haptic interface... " << std::endl;
	}*/
	// ^^^ ++++ CASE WITH ONE SINGLE HAPTIC DEVICE ++++++ ^^^ //


	// vvv ++++ CASE WITH LIST OF HAPTIC DEVICES ++++++ vvvv //
	std::vector < std::string > hapticList = config->getHapticDeviceList();
	for (int i = 0; i < hapticList.size(); i++) {
		if (hapticList[i] == "Geomagic") {
			this->instruments.insert({ hapticList[i].c_str() ,  new Geomagic() });
			//geoPtr = this->instruments["Geomagic"];
			std::cout << "Instantiating Geomagic class as Haptic interface... " << std::endl;
		}
		else if (hapticList[i] == "Armband") {
			this->instruments.insert({ hapticList[i].c_str() ,  new Armband() });
			std::cout << "Instantiating Armband class as Haptic interface... " << std::endl;
		}
		else if (hapticList[i] == "HRing") {
			this->instruments.insert({ hapticList[i].c_str() ,  new HRing() });
			std::cout << "Instantiating HRing class as Haptic interface... " << std::endl;
		}
		else if (hapticList[i] == "TouchDIVER") {
			this->instruments.insert({ hapticList[i].c_str() ,  new WeartDevice() });
			std::cout << "Instantiating WeartDevice class as Haptic interface... " << std::endl;
		}
	}
	// ^^^ ++++ CASE WITH LIST OF HAPTIC DEVICES ++++++ ^^^ //

	// Create a dynamic instance for Teleoperation, only if the same has not added as Haptic device above
	if (this->instruments.find(teleopStringName.c_str()) == this->instruments.end()) {
		if (teleopStringName == "Geomagic") {
			this->instruments.insert({ teleopStringName.c_str() ,  new Geomagic() });
		}
	}

	// 3. Create a F/T sensor dynamic instance (if required)
	if (config->isForceSensorRequested()) {
		this->instruments.insert({ "FTSensor",  new FTSensor("FTSensor") });
	}

	// 4. Create a Needle dynamic instance
	this->instruments.insert({ "Needle",  new EndEffector("Needle") });

	// 5. Create a Phantom dynamic instance
	if (this->config->getPhantomType() == "Abdomen") {
		this->instruments.insert({ "Phantom",  new AbdomenPhantom() });
	}
	else if (this->config->getPhantomType() == "Multilayer gel") {
		this->instruments.insert({ "Phantom",  new MultiLayerGelPhantom() });
	}

	/// Initialize instances of Instruments
	// 1. Robot 
	std::cout << "config->getRobotConfig = " << config->getRobotConfig() << std::endl;
	robotPtr->setConfigFile(std::string("config/") + this->config->getRobotConfig());

	// Set the sample time on the Instrument KUKA object
	//robotPtr->setSampleTime((this->config->isRealRobotRequested() || (!this->config->isRealRobotRequested() && !this->config->isProcessingOnline()) ? CYCLE_TIME : SIM_TIMESTEP));
#ifdef WITH_FRI
	robotPtr->setSampleTime((this->config->isRealRobotRequested() || (!this->config->isRealRobotRequested() && !this->config->isProcessingOnline()) ? CYCLE_TIME : CYCLE_TIME));
#else
	robotPtr->setSampleTime((this->config->isRealRobotRequested() || (!this->config->isRealRobotRequested() && !this->config->isProcessingOnline()) ? SIM_TIMESTEP * 0.1 : SIM_TIMESTEP * 0.1));
#endif // WITH_FRI

	// 2. Geomagic -- TODO: generalize for any haptic interfaces
	//this->instruments[hapticStringName.c_str()]->setConfigFile(std::string("config/") + hapticStringName + std::string("_config.txt"));
	std::cout << "hapticList.size() = " << hapticList.size() << std::endl;
	for (int i = 0; i < hapticList.size(); i++) {
		std::cout << "hapticList[i] = " << hapticList[i] << std::endl;
		std::cout << "file = " << std::string("config/") + hapticList[i] + std::string("_config.txt") << std::endl;
		this->instruments[hapticList[i].c_str()]->setConfigFile(std::string("config/") + hapticList[i] + std::string("_config.txt"));
	}

	this->instruments[teleopStringName.c_str()]->setConfigFile(std::string("config/") + teleopStringName + std::string("_config.txt"));\
	/*if (teleopStringName != hapticStringName) {
		this->instruments[teleopStringName.c_str()]->setConfigFile(std::string("config/") + teleopStringName + std::string("_config.txt"));
	}*/
	//if (geoPtr != nullptr) {
	//	geoPtr->setConfigFile(std::string("config/") + this->config->getGeomagicConfig());
	//}


	// 3. F/T Sensor 
	if (this->config->isForceSensorRequested()) {
		this->instruments["FTSensor"]->setConfigFile(std::string("config/") + this->config->getFTSensorConfig());
	}

	// 4. Needle
	this->instruments["Needle"]->setConfigFile(std::string("config/") + this->config->getNeedleConfig());

	// 5. Phantom
	if (this->config->use3DLayerDepths()) {
		this->instruments["Phantom"]->setConfigFile(std::string("config/") + this->config->getPhantomConfig());
	}

	// Call initialization methods from configuration files
	std::map< std::string, Instrument* >::iterator it;
	//if (this->proxies.size() > 0) {
		for (it = this->instruments.begin(); it != this->instruments.end(); ++it) {
			it->second->initFromConfig();
		}
	//}

	// Add optional end-effector to the KUKA robot
	if (this->config->isForceSensorRequested()) {
		robotPtr->addEndEffector(*(EndEffector*)(this->instruments["FTSensor"]));
		debugPrint<char>("FT Sensor added as EE Tool of the Robot.", NULL);
	}

	if (this->config->isNeedleMounted()) {
		robotPtr->addEndEffector(*(EndEffector*)(this->instruments["Needle"]));
		debugPrint<char>("Needle added as EE Tool of the Robot.", NULL);
	}

}


/**
* @brief Robot initial pose function
* Set the robot (real or virtual) to the desired initial robot pose
*/
void SystemManager::setInitialRobotPose() {

	// Define local pointers 
#ifdef WITH_FRI
	RobotProxy* rpPtr = nullptr;
#endif // WITH_FRI
	VREPProxy* vrep = nullptr;
	RobotInterface* robotPtr = dynamic_cast<RobotInterface*>(this->instruments["Robot"]);;

#ifdef WITH_FRI
	if (this->config->isRealRobotRequested()) {
		rpPtr = dynamic_cast<KukaProxy*>(this->proxies["RobotProxy"]);;
	}
#endif // WITH_FRI

	if (this->config->isSimulatorRequested()) {
		vrep = dynamic_cast<VREPProxy*>(this->proxies["SimulatorProxy"]);
	}

	// Evaluate if KUKA robot starts on place or from an initial desired configuration
	//robotPtr->loadEstimatedFrictionParameters(config.getFrictionParamsFilename().c_str());
	if (robotPtr->robotStartsOnPlace() == false) {
		// Set the KUKA initial configuration (read from config file) to the FRI library
		Eigen::Vector7f qdes = robotPtr->getMsrJointPosition();

		if (this->config->isRealRobotRequested()) {
#ifdef WITH_FRI
			// Set the initial joint configuration read from file as FRICommand of the KukaProxy
			rpPtr->setJointRefs(qdes);
#endif // WITH_FRI
		}
		else if (vrep->isAvailable()) {
			std::cout << "qdes: " << qdes.transpose() << std::endl;
			for (int i = 0; i < KUKA_JOINT_NUM; i++) {
				std::string jointName;
				if (this->config->getRobotManipulatorType() == "KUKA") {
					jointName = std::string("LBR4p_joint") + std::to_string(i + 1);
					if (this->config->isDynSimulationRequested()) { // if dynamics engine is enabled, set commands as target position
						vrep->setJointTargetPosition(jointName.c_str(), qdes(i), this->simPort);
					}
					else {
						vrep->setJointPosition(jointName.c_str(), qdes(i), this->simPort);
					}
				}
			}
		}

	}

}


/**
* @brief BL class init function
* Initialize the BL instance given by the input pointer
* @param instance: the BL class to be initialized
*/
void SystemManager::initBLInstance(BusinessLogicInterface* instance) {

	// Copy the list of proxies
	instance->setProxyList(&this->proxies);

	// Copy the list of instruments
	instance->setInstrumentList(&this->instruments);

	// Call the init() function of the input BL class
	instance->init();
}


/**
* @brief Run function
* Run the BusinessLogic class threads (Task, Controller, Logger, SensorReader)  and
* to exchange data and keep updated the state of the program, based on
* the user inputs
*/
void SystemManager::startSystemThreads() {


	// Run the proxy threads
	std::map < std::string, ExtSystemProxy* >::iterator it;
	std::cout << "List of proxies that are starting a thread: " << std::endl;
	for (it = this->proxies.begin(); it != this->proxies.end(); it++) {
		std::cout << it->first << std::endl;
		it->second->startThread();
	}

	// Run the thread of SystemManager 
	this->run();

	// Run the Task thread
	this->task.run();

	// Run the Controller thread
	this->ctrl.run();

	// Run the Logger thread
	this->log.run();


}

/**
* @brief Join thread function
* Join the previously launched threads
*/
void SystemManager::joinSystemThreads() {


	// Join the proxy threads
	std::map < std::string, ExtSystemProxy* >::iterator it;
	for (it = this->proxies.begin(); it != this->proxies.end(); it++) {
		std::cout << "Joining " << it->first << " thread... " << std::endl;
		it->second->joinThread();
		std::cout << "Joined " << it->first << " proxy. " << std::endl;
	}

	// Join this thread
	this->join();
	std::cout << "Joining SysManager ...";
	std::cout << "OK." << std::endl;//*/

	// Join the Logger thread
	std::cout << "Joining Logger ...";
	this->log.join();
	std::cout << "OK." << std::endl;//*/

	// Join the Controller thread
	this->ctrl.join();

	// Join the Task thread
	std::cout << "Joining Task...";
	this->task.join();
	std::cout << "OK." << std::endl;//*/


}

/**
* @brief Main loop function
*
*/
void SystemManager::mainLoop() {

	
	// At the current stage, HapticState is a static struct because there is at the moment no need
	// of specializing it according to the HapticProxy. On the contrary, TeleopState is a base class
	// that can be specified to a given derived class according to the TeleopProxy. To keep it simple, 
	// HapticState is not converted to a class, but we could consider to do that in the future
	HapticState hs;
	TeleopState* ts; 
	RobotState rs;

	Eigen::Vector6f ftmeasf;
	Eigen::Vector6d ftmeasd;
	double ftmeas_arr[6];
	Eigen::VectorXf q, qdcmd, qcmd, qmsr, trq;
	int  jointNum, logNum, offIdx;
	bool online;
	bool computeResidualOnline = false;

	// Get the singleton Dataset instance (in case of offline processing)
	logNum = this->config->getOfflineLogNumber();
	Dataset* dataset = Dataset::GetInstance(logNum);

	// Local pointers to proxies
	std::map < std::string, ExtSystemProxy* >::iterator vpIt, rpIt, hpIt, tpIt, ftIt;

#ifdef WITH_FRI
	RobotProxy* robProxy = nullptr;
#endif // WITH_FRI

	VREPProxy* vrepProxy = nullptr;
	HapticProxy* haptProxy = nullptr;
	TeleopProxy* teleopProxy = nullptr;
#ifdef WITH_FORCE_SENSOR
	crlFTSensorWrapper* ftProxy = nullptr;
#endif // WITH_FORCE_SENSOR

	// Local pointers to instruments
	std::map < std::string, Instrument* >::iterator robIt;
	RobotInterface* robot = nullptr;

	// Time variables
	double tictoc, dt, rate, tic, toc, tac, Ts, des_rate;
	float t_curr, t_prev;
	Timer clock;
	des_rate = 200.0;
	clock.setRate(des_rate);
	Ts = 1.0 / des_rate;

	// Take local pointers of the required instances (if present)
	rpIt = this->proxies.find("RobotProxy");
	vpIt = this->proxies.find("SimulatorProxy");
	hpIt = this->proxies.find("HapticProxy");
	tpIt = this->proxies.find("TeleopProxy");
	ftIt = this->proxies.find("ForceSensorProxy");

#ifdef WITH_FRI
	if (rpIt != this->proxies.end()) {
		robProxy = (RobotProxy*)this->proxies["RobotProxy"];
	}
#endif // WITH_FRI
	if (vpIt != this->proxies.end()) {
		vrepProxy = (VREPProxy*)this->proxies["SimulatorProxy"];
	}
	if (hpIt != this->proxies.end()) {
		haptProxy = (HapticProxy*)this->proxies["HapticProxy"];
	}
	if (tpIt != this->proxies.end()) {
		teleopProxy = (TeleopProxy*)this->proxies["TeleopProxy"];
	}
	else {
		std::cout << "CANNOT FIND HAPTIC DEVICE!" << std::endl;
	}
#ifdef WITH_FORCE_SENSOR
	if (ftIt != this->proxies.end()) {
		ftProxy = (crlFTSensorWrapper*)this->proxies["ForceSensorProxy"];
	}
	else {
		std::cout << "CANNOT FIND FORCE SENSOR!!!" << std::endl;
	}
#endif // WITH_FORCE_SENSOR

	// Take the pointer of the RobotInterface instance
	robIt = this->instruments.find("Robot");
	if (robIt != this->instruments.end()) {
		robot = (RobotInterface*)this->instruments["Robot"];
	}

	//Ts = robot->getSampleTime();
	jointNum = robot->getJointNum();
	t_curr = robot->getDataTimestamp();
	t_prev = 0.0;

	// Initialize local variables
	q.setZero(jointNum);
	qdcmd.setZero(jointNum);
	qcmd.setZero(jointNum);
	qmsr.setZero(jointNum);
	trq.setZero(jointNum);
	rs.msrJointPosition.setZero(jointNum);
	rs.msrJointTorque.setZero(jointNum);
	rs.extEstJointTorque.setZero(jointNum);
	rs.extEstCartForce.setZero();
	rs.stamp = 0.0;

	for (int i = 0; i < 6; i++) { ftmeas_arr[i] = 0.0; }

#ifdef WITH_FRI
	rs.msrJointPosition = q;
	rs.msrJointTorque = trq;
	rs.extEstJointTorque.setZero(jointNum);
	rs.extEstCartForce.setZero();
	rs.stamp = 0.0;
#endif // WITH_FRI

	// If offline processing is requested, load the desired Dataset (start trigger is sent on task side)
	online = this->config->isProcessingOnline();
	if (!online) {
		dataset->load();
		offIdx = 0;
		//robot->setResidualVectorOffset(dataset->getResidualSample(offIdx));
		qmsr = dataset->getJointPostionSample(offIdx);
		trq = dataset->getJointTorqueSample(offIdx);
		t_curr = dataset->getTimeStamp(offIdx);
		t_prev = t_curr;
	}
	else {

		// If the real robot is connected, wait until the
		// RobotState from RobotProxy is filled.
		// Otherwise, if only the simulated robot is connected,
		// get the initial joint configuration from the virtual scene
		if (this->config->isRealRobotRequested()) {

#ifdef WITH_FRI
			while (!robProxy->isAvailable()) {
				//std::cout << "waiting for kukaProxy ... " << std::endl;
				clock.timeSleep(0.01);
			}

			// Get the current joint configuration of the real KUKA robot from FRI
			rs = robProxy->getRobotState();
			qmsr = rs.msrJointPosition;
			trq = rs.msrJointTorque;
			t_curr = rs.stamp;
			t_prev = t_curr;
#endif // WITH_FRI

		}
		else if (this->config->isSimulatorRequested() && vrepProxy->isAvailable()) {

			// Start joint position streaming
			for (int i = 0; i < 7; i++) {
				std::string qname = "LBR4p_joint" + std::to_string(i + 1);
				qmsr(i) = vrepProxy->getJointPosition(qname.c_str(), simx_opmode_blocking, this->simPort);
				vrepProxy->getJointPosition(qname.c_str(), simx_opmode_streaming, this->simPort);
			}

		}
	}

	// Initialize generalized momentum
	qcmd = qmsr;

	// Initialize the robot state
	Eigen::VectorXf resOff;
	resOff.setZero(jointNum);
	robot->setResidualVectorOffset(resOff);
	robot->setMsrJointPosition(qmsr);
	robot->setCmdJointPosition(qcmd);
	robot->setMsrJointVelocity(Eigen::VectorXf::Zero(KUKA_JOINT_NUM));
	robot->setCmdJointVelocity(Eigen::VectorXf::Zero(KUKA_JOINT_NUM));
	robot->setMsrJointAcceleration(Eigen::VectorXf::Zero(KUKA_JOINT_NUM));
	robot->setMsrJointTorques(trq);
	robot->computeKinematics();
	robot->computeDynamics();
	robot->initializeGeneralizedMomentum();
	robot->computeResidualFull(qmsr, Eigen::VectorXf::Zero(KUKA_JOINT_NUM), trq);
	resOff = robot->getModelDynParams().g - trq;
	robot->setResidualVectorOffset(resOff);

	// Notify the Controller to start
	this->ctrl.start();

	while (this->ok()) {

		// Measure starting time
		tic = clock.getCurTime();

		//----------------------------------------------------------------//
		// Do stuff here... 

		// Update time
		//this->time_ += dt;

		// Check Task state to start logging
		if (this->systemstate->getTaskInfo().taskState == STATE::RUNNING_STATE) {
			if (this->config->isLogEnabled()) {

				if(!this->log.isLogging()){
					// Create the log folder ... 
					std::string logPath = this->log.createMainFolder();

					// ... and pass the path string to the path
					this->systemstate->setLogPath(logPath);

					// Start the log
					this->log.start();
				}

				// Update Logger time
				this->log.setTimeStamp(this->time_);
			}

			// Update Task time
			this->task.setTimeStamp(this->time_);

		}

		if (this->systemstate->getTaskInfo().taskState == STATE::WAITING_STATE) {
		

			if (this->config->isLogEnabled()) {

				// Close log
				this->log.requestSaveAndWait();

			}

			this->systemstate->setTaskState(STATE::STOPPED_STATE);
		
		}

		if (online) {
			/// GET DATA FROM ...
			// ... Robot proxy, ...
			if (this->config->isRealRobotRequested()) {

#ifdef WITH_FRI
				if (robProxy->isAvailable()) {
					// Get the current joint configuration of the real KUKA robot from FRI
					rs = robProxy->getRobotState();
				}
#endif // WITH_FRI
			}
			else if (vrepProxy->isAvailable()) { // If real robot is not connected and V-REP is running, get the current configuration from the simulator

				for (int i = 0; i < jointNum; i++) {

					std::string qname;
					if (this->config->getRobotManipulatorType() == "KUKA") {
						qname = "LBR4p_joint" + std::to_string(i + 1);
					}
					rs.msrJointPosition(i) = vrepProxy->getJointPosition(qname.c_str(), simx_opmode_buffer, this->simPort);
					rs.msrJointTorque(i) = 0.0; //<--- TODO: TO BE FILLED
					rs.extEstJointTorque(i) = 0.0;
					rs.stamp = this->time_ + Ts;
				}

			}

			// ... Haptic device  ...
			// MF Remark: due to the nature of a generic haptic device, there are not real-time measurements
			// to be read by the corresponding device, as the haptic interface only receives input forces
			// to apply. For this reason, this function below has nothing to do here, since the input set
			// of the haptic device instrument is done in the Task context, while the set of the haptic
			// force on the real device occurs in the Control context. Consider to remove this call below
			// and its subsequent dependencies
 			//hs = haptProxy->getHapticState();

			// ... Teleoperation device and ...
			ts = teleopProxy->getTeleopState();

			// ... Force Sensor Proxy
			//ftProxy->getCurrentWrenchMeas(ftmeas_arr);
			std::memcpy(ftmeasd.data(), ftmeas_arr, sizeof(double)*6);
			ftmeasf = ftmeasd.cast<float>();
		}
		else {

			// Get current Dataset index
			offIdx = dataset->getCurrentIndex();

			// Get Robot Dataset data
			rs.stamp = dataset->getTimeStamp(offIdx);
			rs.msrJointPosition = dataset->getJointPostionSample(offIdx);
			rs.msrJointTorque = dataset->getJointTorqueSample(offIdx);
			rs.extEstJointTorque = dataset->getFRIResidualSample(offIdx);
			rs.extEstCartForce.setZero();

			// Get Haptic device Dataset data (TODO)
			//hs.force.setZero();
			//hs.asyncTriggerState = 0;
			//hs.buttonState = new bool[2];
			//hs.buttonState[0] = dataset->getHapticClutchSample(offIdx);
			//hs.buttonState[1] = false;

			// Get Teleop device Dataset data (TODO)
			ts = nullptr;

			// Get FTSensor Dataset data 
			ftmeasf = dataset->getFTSensorMeasSample(offIdx);

			if (this->systemstate->getTaskInfo().taskState == STATE::RUNNING_STATE) {			
				
				/// THIS IS COMMENTED ONLY TO TEST CLAUDIO'S MATLAB SCRIPT. DATASET EVOLVES IN TASKCONTEXT
				// Increase the dataset index
				dataset->nextStep();

				// Check if the Dataset has finished
				if (offIdx >= dataset->getSize()) {
					this->systemstate->setTaskState(STATE::STOPPING_STATE);
				}//*/
			}

			//delete hs.buttonState;

		}

		/// UPDATE INSTRUMENT CLASSES
		// Update RobotInterface
		this->updateRobot(rs, online);
		qcmd = robot->getCmdJointPosition();
		qdcmd = robot->getCmdJointVelocity();
		trq = robot->getMsrJointTorques();

		// Update HapticDevice
		//this->updateHapticDevice(hs);

		// Update TeleopDevice
		this->updateTeleopDevice(ts);

		// Update F/T Sensor
		this->updateFTSensor(ftmeasf);

		// Apply the new joint position commands on the simulated robot (if available) 
		if (vrepProxy->isAvailable() && this->systemstate->getCtrlInfo().ctrlState == RUNNING_STATE) {

			std::map < std::string, int > simNames = vrepProxy->getSimObjects();
			std::map < std::string, int >::iterator it;

			for (int i = 0; i < jointNum; i++) {
				std::string jointStr = "joint" + std::to_string(i + 1);
				for (it = simNames.begin(); it != simNames.end(); it++) {
					if (it->first.find(jointStr) != std::string::npos) {

						std::string jointName = it->first;
						std::string jointDotName = jointName + std::string("dot");
						std::string jointTorqueName = jointName + std::string("torque");

						// Send signals of q and qdot on V-REP (could I move all this setFloatSignal on SystemManager?)
						vrepProxy->setFloatSignal(jointName.c_str(), qcmd(i), simx_opmode_oneshot, this->simPort);
						vrepProxy->setFloatSignal(jointDotName.c_str(), qdcmd(i), simx_opmode_oneshot, this->simPort);
						vrepProxy->setFloatSignal(jointTorqueName.c_str(), trq(i), simx_opmode_oneshot, this->simPort);

						if (config->getRobotManipulatorType() == "KUKA") {
							std::string friResSigName_i = std::string("friRes_q") + std::to_string(i + 1);
							float friResidual_i = dynamic_cast<KUKARobot*>(robot)->getFRIResidual()(i);
							vrepProxy->setFloatSignal(friResSigName_i.c_str(), friResidual_i, simx_opmode_oneshot, this->simPort);
						}

					}
				}
			}
		}
	
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
		//std::cout << "[SysMan] Running rate:" << (1.0 / dt) << std::endl;
	}

	// If exit from here, you should exit from TaskContext, ControlContext and Logger ... 
	// Exit the Task
	this->task.exit();

	// Exit the Controller
	this->ctrl.exit();

	// Exit the Logger
	this->log.exit();

	// ... and all the proxies
	std::map<std::string, ExtSystemProxy*>::iterator it;
	std::cout << "Number of proxies = " << this->proxies.size() << std::endl;
	for (it = this->proxies.begin(); it != this->proxies.end(); it++) {
		std::cout << "Clearing proxy " << it->first << " ... ";
		it->second->clear();
		std::cout << "OK. " << std::endl;
	}

}

/**
* @brief Update function
* Update the state of the robot from data acquired from the
* corresponding proxy
* @param rs: the RobotState structure
* @param online: the RobotState structure
*/
void SystemManager::updateRobot(const RobotState& rs, const bool& online) {

	int joint_num, offIdx;
	double time_prev, Ts;
	Eigen::VectorXf qdmsr, qdcmd, qmsr, qprev, trq, residual;
	
	// Local pointers to instruments
	std::map < std::string, Instrument* >::iterator robIt;
	RobotInterface* robot = nullptr;

	// Take the pointer of the RobotInterface instance
	robIt = this->instruments.find("Robot");
	if (robIt != this->instruments.end()) {
		robot = (RobotInterface*)this->instruments["Robot"];
		joint_num = robot->getJointNum();
	}

	// Get Singleton instance of Dataset object
	Dataset* dataset = Dataset::GetInstance(0);
	offIdx = dataset->getCurrentIndex();

	// Update the RobotInterface class
	time_prev = this->time_;
	this->time_ = rs.stamp;
	qprev = robot->getMsrJointPosition();
	qmsr = rs.msrJointPosition;
	trq = rs.msrJointTorque;
	qdcmd = robot->getCmdJointVelocity();

	if (this->time_ != time_prev) { // <-- Important! Evaluate only if the robot configuration has changed!

		// Set the robot configuration on the local static instance of the Kuka robot
#ifdef WITH_FRI
		qdmsr = (qmsr - qprev) / CYCLE_TIME;
#else
		qdmsr = (qmsr - qprev) / (this->time_ - time_prev);
#endif // WITH_FRI

		// If requested during offline processing, use the logged residual signal instead of computing from updated kinematics and dynamics
		if (!online) {
			qdcmd = dataset->getJointVelocitySample(offIdx);
			residual = dataset->getResidualSample(offIdx);
			
			robot->setCmdJointPosition(qmsr);
			robot->setCmdJointVelocity(qdcmd);
			//robot->setResidualVector(residual);
		}

		// Update the full state of the robot with the current measurements
		robot->updateRobotState(qmsr, qdcmd, trq, online);

		// Set timestamp
		robot->setDataTimestamp(rs.stamp);

		// Set KUKA robot data
		if (this->config->getRobotManipulatorType() == "KUKA") {
			dynamic_cast<KUKARobot*>(robot)->setFRIResidual(rs.extEstJointTorque);
			Eigen::Vector6f recForceFRI = robot->staticForceReconstruction(rs.extEstJointTorque);
			dynamic_cast<KUKARobot*>(robot)->setFRICartesianForces(rs.extEstCartForce);
			dynamic_cast<KUKARobot*>(robot)->setFriResFbee(recForceFRI);
		}

	}

}

/**
* @brief Update function
* Update the state of the teleop device from data acquired from the
* corresponding proxy
* @param data: a void pointer to the acquired data (internal static cast is required based on the type of device)
*/
void SystemManager::updateTeleopDevice(TeleopState* ts) {

	// TODO: In the future, if additional teleoperation device will be added to the
	// framework, Geomagic entity classes should have a similar separation as done in
	// the Geomagic proxy classes, distinguishing a GeomagicTeleop and a GeomagicHaptic
	// classes, each of them derived from more general base classes (HapticInterface, 
	// that already exists, and a TeleopInterface)
	Eigen::Vector6f hipVel;

	// Local pointers to instruments
	std::map < std::string, Instrument* >::iterator teleIt; 
#ifdef WITH_GEOMAGIC
	if (config->getTeleopDeviceType() == "Geomagic") {

		Geomagic* teleopDevice = nullptr;
		GeoTeleopState* gts = (GeoTeleopState*)(ts);
		hipVel = gts->hipVelocity;
		teleIt = this->instruments.find("Geomagic");
		if (teleIt != this->instruments.end()) {
			teleopDevice = (Geomagic*)this->instruments["Geomagic"];
			dynamic_cast<Geomagic*>(teleopDevice)->setHIPVel(hipVel);
			dynamic_cast<Geomagic*>(teleopDevice)->setClutchState(gts->buttonState[0]);
		}

	}
#endif // WITH_GEOMAGIC

}


// TODO: Consider to remove this method. In fact, in SystemManagar only the entities 
// that should be updated based on measurements coming from external devices should
// be considered. The haptic feedback is a command, not a measurement, and therefore
// it is more likely that the responsibility to set this value is let to the Task
// that needs to manage such information according to the required specification of 
// the task itself
/**
* @brief Update function
* Update the state of the haptic device from data acquired from the
* corresponding proxy
* @param hs: the HapticState structure
*/
void SystemManager::updateHapticDevice(const HapticState& hs) {

	// Local pointers to instruments
	std::map < std::string, Instrument* >::iterator haptIt;
	std::string hapticName = "HapticProxy";
	HapticInterface* haptic = nullptr;

	haptIt = this->instruments.find(hapticName.c_str());
	if (haptIt != this->instruments.end()) {
		haptic = (HapticInterface*)this->instruments[hapticName.c_str()];
		//haptic->setForceFeedback(hs.force);
		//std::cout << "hs.force = " << hs.force.transpose() << std::endl;
	}

}


/**
* @brief Update function
* Update the state of the F/T sensor from data acquired from the
* corresponding proxy
* @param meas: the F/T sensor measurement
*/
void SystemManager::updateFTSensor(const Eigen::Vector6f& meas) {

	FTSensor* ftsensor = nullptr;
	std::map <std::string, Instrument*>::iterator ftIt;

	ftIt = this->instruments.find("FTSensor");
	if(ftIt != this->instruments.end()){
		ftsensor = dynamic_cast<FTSensor*>(this->instruments["FTSensor"]);
	}

	ftsensor->setWrench(meas.cast<double>());
}



/**
* @brief Reset function
* Send the request to reset the F/T Sensor bias
*/
void SystemManager::resetSystemFTSensorBias() {

#ifdef WITH_FORCE_SENSOR
	crlFTSensorWrapper* ftsensor = nullptr;
	std::map <std::string, ExtSystemProxy*>::iterator ftIt;

	ftIt = this->proxies.find("ForceSensorProxy");
	if (ftIt != this->proxies.end()) {
		ftsensor = dynamic_cast<crlFTSensorWrapper*>(this->proxies["ForceSensorProxy"]);
		ftsensor->requestBiasReset();

	}
#endif // WITH_FORCE_SENSOR
}


/**
* @brief Clear function
*/
void SystemManager::clear() {

}
