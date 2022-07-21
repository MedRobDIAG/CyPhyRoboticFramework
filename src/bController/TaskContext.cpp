// Project Header files
#include "TaskContext.hpp"
#include "VREPProxy.hpp"
#include "RegistrationStrategy.hpp"
#include "NeedleTissueIntEstStrategy.hpp"
#include "ForceFeedbackNeedleInsertionStrategy.hpp"
#include "Dataset.hpp"

/**
* @brief Init function
*/
void TaskContext::init() {

	this->exit_loop = false;
	this->started = false;
	this->accomplished = false;
	this->time_ = 0.0;

	// Initialize the singleton SystemState and Configuration classes
	// SystemState
	this->systemstate = SystemState::GetInstance();

	// Configuration
	this->config = Configuration::GetInstance(""); // Input filename string is useless here, as the singleton class is supposed to be already instantiated by SystemManager
												
	// Simulator port
	this->setSimPort(19996);

}


/**
* @brief Main loop function
*
*/
void TaskContext::mainLoop() {

	// Local pointers to proxies
	std::map < std::string, ExtSystemProxy* >::iterator vpIt;
	VREPProxy* vrepProxy = nullptr;

	// Local pointers to instruments
	std::map < std::string, Instrument* >::iterator robIt, phIt, ftIt, hIt;
	RobotInterface* robot = nullptr;
	Phantom* phantom = nullptr;
	FTSensor* ftsensor = nullptr;

	// vvv ++++ CASE WITH ONE SINGLE HAPTIC DEVICE ++++++ vvv //
	//HapticInterface* haptic = nullptr;
	// ^^^ ++++ CASE WITH ONE SINGLE HAPTIC DEVICE ++++++ ^^^ //

	// vvv ++++ CASE WITH LIST OF HAPTIC DEVICES ++++++ vvv //
	std::map < std::string, HapticInterface* > haptics;
	std::vector < std::string > hapticNameList = this->config->getHapticDeviceList();
		for (int i = 0; i < hapticNameList.size(); i++) {
			std::cout << "hapticNameList[i] = " << hapticNameList[i] << std::endl;
			haptics.insert({hapticNameList[i], nullptr});
		}
	// ^^^ ++++ CASE WITH LIST OF HAPTIC DEVICES ++++++ ^^^ //

	// Get Dataset Singleton instance
	Dataset* dataset = Dataset::GetInstance(0);

	// Time variables
	double tictoc, dt, rate, tic, toc, tac, Ts, des_rate;
	float t_curr, t_prev;
	Timer clock;
	dt = 0.0;
	des_rate = 20.0;  1.0 / 0.004; 200.0;
	des_rate = 1.0 / 0.004; 200.0;
	clock.setRate(des_rate);
	Ts = 1.0 / des_rate;

	// Take local pointers of the required instances (if present)
	vpIt = this->proxies.find("SimulatorProxy");
	if (vpIt != this->proxies.end()) {
		vrepProxy = (VREPProxy*)this->proxies["SimulatorProxy"];
	}

	// Take the pointer of the RobotInterface instance
	robIt = this->instruments.find("Robot");
	if (robIt != this->instruments.end()) {
		robot = (RobotInterface*)this->instruments["Robot"];
	}

	phIt = this->instruments.find("Phantom");
	if (phIt != this->instruments.end()) {
		phantom = (Phantom*)this->instruments["Phantom"];
	}

	ftIt = this->instruments.find("FTSensor");
	if (ftIt != this->instruments.end()) {
		ftsensor = (FTSensor*)this->instruments["FTSensor"];
	}

	// vvv ++++ CASE WITH ONE SINGLE HAPTIC DEVICE ++++++ vvvv //
	/*std::string hapticInstrument = this->config->getHapticDeviceType();
	hIt = this->instruments.find(hapticInstrument.c_str());
	if (hIt != this->instruments.end()) {
		if (hapticInstrument.find("Geomagic") != std::string::npos) {
			haptic = (Geomagic*)this->instruments[hapticInstrument.c_str()];
		}
		else if (hapticInstrument.find("Armband") != std::string::npos) {
			haptic = (Armband*)this->instruments[hapticInstrument.c_str()];
		}
	}//*/
	// ^^^ ++++ CASE WITH ONE SINGLE HAPTIC DEVICE ++++++ ^^^ //

	// vvv ++++ CASE WITH LIST OF HAPTIC DEVICES ++++++ vvv //
	for (int i = 0; i < hapticNameList.size(); i++) {
		std::string hapticName_i = hapticNameList[i];
		std::map < std::string, Instrument* >::iterator hIt = this->instruments.find(hapticName_i.c_str());
		if (hIt != this->instruments.end()) {
			if (hapticName_i.find("Geomagic") != std::string::npos) {
				haptics[hapticName_i] = (Geomagic*)this->instruments[hapticName_i.c_str()];
			}
			else if (hapticName_i.find("Armband") != std::string::npos) {
				haptics[hapticName_i] = (Armband*)this->instruments[hapticName_i.c_str()];
			}
			else if (hapticName_i.find("HRing") != std::string::npos) {
				haptics[hapticName_i] = (HRing*)this->instruments[hapticName_i.c_str()];
			}
			else if (hapticName_i.find("TouchDIVER") != std::string::npos) {
				haptics[hapticName_i] = (WeartDevice*)this->instruments[hapticName_i.c_str()];
			}
		}
	}
	// ^^^ ++++ CASE WITH LIST OF HAPTIC DEVICES ++++++ ^^^ //


	while (this->ok()) {

		// Measure starting time
		tic = clock.getCurTime();

		//----------------------------------------------------------------//
		// Do stuff here... 
		if (this->systemstate->getTaskInfo().taskState == STATE::REQUESTED_STATE) {


			if (this->systemstate->getTaskInfo().taskType == TASK::REGISTRATION_TASK) {

				if (this->config->getPhantomType() == "Abdomen") {
					AbdomenPhantom* abdphantom = dynamic_cast<AbdomenPhantom*>(phantom);
					this->setTaskStrategy(new RegistrationStrategy(abdphantom, vrepProxy, robot));
				}
				else {
					std::cout << "WARNING: The specified phantom in the Configuration file is not suitable for the requested Registration routine. \n"
						<< "Setting the Task state on waiting to allow new request." << std::endl;

					// Set the Task on WAITING_STATE
					this->systemstate->setTaskState(STATE::WAITING_STATE);

				}

			}
			else if (this->systemstate->getTaskInfo().taskType == TASK::PLANNING_TASK) {

				// TODO
				//this->setTaskStrategy(new PlanningStrategy(phantom, vrepProxy, robot));

			}
			else if (this->systemstate->getTaskInfo().taskType == TASK::NEEDLE_TISSUE_KV_ESTIMATION_TASK) {

				this->setTaskStrategy(new NeedleTissueIntEstStrategy(phantom, ftsensor, haptics, vrepProxy, robot));

			}
			else if (this->systemstate->getTaskInfo().taskType == TASK::FORCE_FEEDBACK_NEEDLE_INSERTION_TASK) {

				this->setTaskStrategy(new ForceFeedbackNeedleInsertionStrategy(phantom, ftsensor, haptics, vrepProxy, robot));

			}

			// Set the Task Strategy state on running
			this->systemstate->setTaskState(STATE::SET_STATE);

			// Clear the User action request
			this->systemstate->setAction(ACTION::CLEARED);

		}
		else if (this->systemstate->getTaskInfo().taskState == STATE::SET_STATE) {

			if (this->systemstate->getAction() == ACTION::REQUESTED) {
			
				// Run the initialization routine
				this->routine->init();

				// Set Task on running
				this->systemstate->setTaskState(STATE::RUNNING_STATE);
				this->start();
				//robot->resetDynParams();

				// Clear the User action request
				this->systemstate->setAction(ACTION::CLEARED);

				std::cout << "Task started. " << std::endl; 
			}

		}
		else if (this->systemstate->getTaskInfo().taskState == STATE::RUNNING_STATE) {

			// Run the routine of the currently active task
			this->routine->setTimeStamp(this->time_);
			int ret = this->routine->execTask();

			// Set Stopping state on Task state
			if (ret == TASK_RETURN_CODE::FINISHED) {
				this->systemstate->setTaskState(STATE::STOPPING_STATE);
			} 
		}
		else if (this->systemstate->getTaskInfo().taskState == STATE::STOPPING_STATE) {
		
			// Call the termination routine
			this->routine->terminate();

			// Finally, set the Task on STOPPED
			this->systemstate->setTaskState(STATE::WAITING_STATE);
		}


		vrepProxy->setFloatSignal("runningTask", this->isStarted(), simx_opmode_oneshot, this->simPort);
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
		//debugPrint<double>("\n[Controller] Running rate:", 1.0 / dt);

	}

}



/**
* @brief Update function
* Check for user requests of new changes on the running tasks
*/
void TaskContext::checkTaskRequests(){






}


/**
* @brief Clear function
*/
void TaskContext::clear() {



}
