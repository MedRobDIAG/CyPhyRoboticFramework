// Project Header files
#ifdef WITH_WEART
#include "WeartProxy.hpp"
#endif

#include "ControlContext.hpp"
#include "RobotProxy.hpp"
#include "KUKARobot.hpp"
#include "VREPProxy.hpp"
#include "HapticProxy.hpp"
#include "TeleopControlTask.hpp"
#include "VirtualFixtureControlTask.hpp"
#include "PositionLockControlTask.hpp"
#include "OrientationLockControlTask.hpp"
#include "AutonomousControlTask.hpp"
#include "ManualGuidanceControlTask.hpp"
#include "Armband.hpp"
#include "HRing.hpp"
#include "WeartDevice.hpp"

// Standard Header files
#include <algorithm>    // std::remove_if

/**
* @brief Init function
*/
void ControlContext::init() {

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
	this->setSimPort(19995);

	//Clear the stack
	this->stackOfTasks.clear();
}

/**
* @brief Main loop function
*
*/
void ControlContext::mainLoop() {

	Eigen::VectorXf q, qdcmd, qcmd, qsim, qmsr, trq;
	int  jointNum;

	// Time variables
	double tictoc, dt, rate, tic, toc, tac, Ts, des_rate;
	float t_curr, t_prev;
	Timer clock;

	// Local pointers to proxies
	std::map < std::string, ExtSystemProxy* >::iterator vpIt, rpIt;// , hpIt;
	RobotProxy* robProxy = nullptr;
	VREPProxy* vrepProxy = nullptr;

	// Local pointers to instruments
	std::map < std::string, Instrument* >::iterator robIt, hapDevIt;
	RobotInterface* robot = nullptr;

	// vvv ++++ CASE WITH ONE SINGLE HAPTIC DEVICE ++++++ vvv //
	//HapticInterface* haptic = nullptr;
	//HapticProxy* hapticProxy = nullptr;
	// ^^^ ++++ CASE WITH ONE SINGLE HAPTIC DEVICE ++++++ ^^^ //
	
	// vvv ++++ CASE WITH LIST OF HAPTIC DEVICES ++++++ vvv //
	std::map < std::string, HapticInterface* > haptics;
	std::map < std::string, HapticProxy* > hapticProxies;
	std::vector < std::string > hapticNameList = this->config->getHapticDeviceList();
	for (int i = 0; i < hapticNameList.size(); i++) {
		haptics.insert({ hapticNameList[i], nullptr });
		hapticProxies.insert({ hapticNameList[i], nullptr });
	}
	// ^^^ ++++ CASE WITH LIST OF HAPTIC DEVICES ++++++ ^^^ //

	// Take local pointers of the required instances (if present)
	rpIt = this->proxies.find("RobotProxy");
	vpIt = this->proxies.find("SimulatorProxy");
	if (rpIt != this->proxies.end()) {
		robProxy = (RobotProxy*)this->proxies["RobotProxy"];
	}
	if (vpIt != this->proxies.end()) {
		vrepProxy = (VREPProxy*)this->proxies["SimulatorProxy"];
	}	
	// vvv ++++ CASE WITH ONE SINGLE HAPTIC DEVICE ++++++ vvv //
	/*hpIt = this->proxies.find("HapticProxy");
	if (hpIt != this->proxies.end()) {
		hapticProxy = (HapticProxy*)this->proxies["HapticProxy"];
	}*/
	// Take the pointer of the HapticIntrument instance
	/*std::string hapticInstrument = this->config->getHapticDeviceType();
	hapDevIt = this->instruments.find(hapticInstrument.c_str());
	if (hapDevIt != this->instruments.end()) {
		if (hapticInstrument.find("Geomagic") != std::string::npos) {
			haptic = (Geomagic*)this->instruments["Geomagic"];
		}
		if (hapticInstrument.find("Armband") != std::string::npos) {
			haptic = (Armband*)this->instruments["Armband"];
		}
	}//*/
	// ^^^ ++++ CASE WITH ONE SINGLE HAPTIC DEVICE ++++++ ^^^ //

	// vvv ++++ CASE WITH LIST OF HAPTIC DEVICES ++++++ vvv //
	for (int i = 0; i < hapticNameList.size(); i++) {
		std::string hapticProxyName_i = std::string("HapticProxy") + std::to_string(i + 1);
		std::string hapticName_i = hapticNameList[i];
		std::map < std::string, ExtSystemProxy* >::iterator hpIt = this->proxies.find(hapticProxyName_i);
		std::map < std::string, Instrument* >::iterator hIt = this->instruments.find(hapticName_i.c_str());
		if (hpIt != this->proxies.end()) {
			hapticProxies[hapticNameList[i]] = (HapticProxy*)hpIt->second;
		}
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

	// Take the pointer of the RobotInterface instance
	robIt = this->instruments.find("Robot");


	if (robIt != this->instruments.end()) {
		robot = (RobotInterface*)this->instruments["Robot"];
		jointNum = robot->getJointNum();
	}


	// Initialize local variables
	q.setZero(jointNum);
	qdcmd.setZero(jointNum);
	qcmd.setZero(jointNum);
	qsim.setZero(jointNum);
	qmsr.setZero(jointNum);
	trq.setZero(jointNum);

	// At this point, the Controller should initialize structures related to the 
	// robot. Therefore, as long as the robot instance is not initialized by other
	// other threads (SystemManager), Controller must wait
	this->waitForStartRequest();

	des_rate = 1.0 / robot->getSampleTime();

	clock.setRate(des_rate);
	Ts = 1.0 / des_rate;

	// Initialize the command with the current configuration
	qcmd = robot->getMsrJointPosition();

	// Start the main loop
	while (this->ok()) {

		// Measure starting time
		tic = clock.getCurTime();

		//----------------------------------------------------------------//
		// Do stuff here... 

		if (this->config->isProcessingOnline()) {
			/// Check changes in SystemState 
			// 1. Check for request of Control strategies
			this->checkCtrlModeRequest();

			// 2. Check for requests of kinematic constraints
			this->checkKinConstraintRequest();

			// Check new data timestamp from RobotInterface
			t_curr = robot->getDataTimestamp();
			qmsr = robot->getMsrJointPosition();

			if (t_curr != t_prev) { // <-- Important! Evaluate only if the robot configuration has changed!

				// Call the control law method
				if (this->systemstate->getCtrlInfo().ctrlState == RUNNING_STATE) {

					// Update Control Tasks
					this->updateTasks();

					// Solve Stack of tasks
					qdcmd = this->solveControlStack();
					//qdcmd.setZero(jointNum);

				}
				else {
					qdcmd.setZero(jointNum);
				}

				// Integrate commanded joint velocities
				qcmd += Ts * qdcmd;

				// Set commanded values on the KUKA robot
				robot->setCmdJointVelocity(qdcmd);
				robot->setCmdJointPosition(qcmd);

				// update prev variables
				t_prev = t_curr;
			}
		}
		else {
		
			qdcmd = robot->getMsrJointVelocity();
			qcmd = robot->getMsrJointPosition();

		}

		// Apply the new joint position commands on the real robot (if enabled) 
		if (this->config->isRealRobotRequested() && robProxy->isAvailable()) {
			robProxy->setCommands(qcmd);
		}

		// vvv ++++ CASE WITH ONE SINGLE HAPTIC DEVICE ++++++ vvv //
		// Apply the feedback force to the master device in case of Teleoperation control strategy,
		// where the corresponding variable has been set in Geomagic instance
		// TEMP: Comment this if branch to verify the armband connection
		//if (this->systemstate->getCtrlInfo().ctrlMode == TELEOPERATION_MODE) {
			/*Eigen::Vector3f masterForce = haptic->getForceFeedback();
			hapticProxy->setHapticForce(masterForce);//*/
			//std::cout << "masterForce = " << masterForce.transpose() << std::endl;
		//}
		// ^^^ ++++ CASE WITH ONE SINGLE HAPTIC DEVICE ++++++ ^^^ //

	// vvv ++++ CASE WITH LIST OF HAPTIC DEVICES ++++++ vvv //
		for (int i = 0; i < hapticNameList.size(); i++) {
			std::string hapticName_i = hapticNameList[i];
			Eigen::VectorXf masterForce = haptics[hapticName_i]->getForceFeedback();
			hapticProxies[hapticName_i]->setHapticForce(masterForce);

			// Maybe a better way to integrate this can be found so as to make everything more homogeneous ... use this now for the sake of time
			if (hapticName_i.find("TouchDIVER") != std::string::npos) {
				//std::cout << "masterForce = " << masterForce.transpose() << std::endl;
				float vibVolume = dynamic_cast<WeartDevice*>(haptics[hapticName_i])->getVibrationVolume();
				hapticProxies[hapticName_i]->setVibrotactileIntensity(vibVolume);
			}
		}
		// ^^^ ++++ CASE WITH LIST OF HAPTIC DEVICES ++++++ ^^^ //

		
		// Apply the new joint position commands on the simulated robot (if available) 
		if (vrepProxy->isAvailable()) {

			std::map < std::string, int > simNames = vrepProxy->getSimObjects();
			std::map < std::string, int >::iterator it;

			for (int i = 0; i < jointNum; i++) {
				std::string jointStr = "joint" + std::to_string(i + 1);
				for (it = simNames.begin(); it != simNames.end(); it++) {
					if (it->first.find(jointStr) != std::string::npos) {
						std::string jointName = it->first;
						
						qsim = (this->config->isRealRobotRequested()) ? qmsr : qcmd;

						if (vrepProxy->isDynamicEngineEnabled(this->simPort)) { // if dynamics engine is enabled, set commands as target position
							vrepProxy->setJointTargetPosition(jointName.c_str(), qsim(i), this->simPort);
						}
						else { // ... otherwise, set commands as simple position
							vrepProxy->setJointPosition(jointName.c_str(), qsim(i), this->simPort);
						}
					}
				}
			}
		}//*/

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
* @brief Update function
* Update the control modality (among teleoperation, autonomous and manual guidance) based on user request
*/
void ControlContext::checkCtrlModeRequest() {

	// Local pointers to instruments
	std::map < std::string, Instrument* >::iterator robIt, hapDevIt, teleopDevIt;
	std::string hapticInstrument = this->config->getHapticDeviceType();
	std::string teleopInstrument = this->config->getTeleopDeviceType();
	RobotInterface* robot = nullptr;
	HapticInterface* haptic = nullptr;
	Geomagic* teleopDev = nullptr; // TODO: Need to generalize with arbitrary TeleopInterface

	// Take the pointer of the RobotInterface instance
	robIt = this->instruments.find("Robot");
	hapDevIt = this->instruments.find(hapticInstrument.c_str());
	teleopDevIt = this->instruments.find(teleopInstrument.c_str());
	if (robIt != this->instruments.end()) {
		robot = (RobotInterface*)this->instruments["Robot"];
	}
	if (hapDevIt != this->instruments.end()) {
		if (hapticInstrument.find("Geomagic") != std::string::npos) {
			haptic = (Geomagic*)this->instruments[hapticInstrument.c_str()];
		}
		if (hapticInstrument.find("Armband") != std::string::npos) {
			haptic = (Armband*)this->instruments[hapticInstrument.c_str()];
		}
	}
	if (teleopDevIt != this->instruments.end()) {
		if (teleopInstrument.find("Geomagic") != std::string::npos) {
			teleopDev = (Geomagic*)this->instruments[teleopInstrument.c_str()];
		}
	}

	if (this->systemstate->getCtrlInfo().ctrlState == REQUESTED_STATE) {

		// Every time you set a new controller, reset residual vectors and cumulative variables
		robot->resetDynParams();

		if (this->systemstate->getCtrlInfo().ctrlMode == AUTONOMOUS_MODE) {
			
			// Define the autonomous trajectory to be performed
			Trajectory* traj = nullptr;
			if (this->config->getAutonomousTrajectoryType() == SINUSOIDAL) {
				traj = new SinTrajectory(".\\config\\sinTrajectory.txt");
			}
			else if (this->config->getAutonomousTrajectoryType() == TRAJ_NUM) { // fake flag
				// ...
			}

			// Set the reference trajectory resolution from the robot sample time
			traj->setSampleTime(robot->getSampleTime());

			AutonomousControlTask* act = new AutonomousControlTask(POSITION_CONTROL, SPACE_DIM *2, robot, traj);
			Eigen::Matrix6f K = Eigen::Matrix6f::Identity() * this->config->getAutonomousCtrlGain();
			act->setGainMatrix(K);

			// Add new Task to the stack
			this->addNewTask(act, MIDDLE_LEVEL_PRIORITY, true);

			std::cout << "Control AUTONOMOUS_MODE set in ControlContext. " << std::endl << std::endl;
		}
		else if (this->systemstate->getCtrlInfo().ctrlMode == MANUAL_GUIDANCE_MODE) {

			// Get useful variables
			int jointNum = robot->getJointNum();
			float resThres;
			float fcut;
			Eigen::MatrixXf Kmg;
			std::vector<float> gainsvec;
			
			resThres = this->config->getMGResThresh();
			gainsvec = this->config->getMGGains();
			fcut = this->config->getMGCutoffFreq();
			Kmg.setZero(jointNum, jointNum);
			for (int i = 0; i < gainsvec.size(); i++) {
				Kmg(i,i) = gainsvec[i];
			}

			// Define the Manual Guidanc Control Task instance
			ManualGuidanceControlTask* mgct = new ManualGuidanceControlTask(JOINT_CONTROL, jointNum, robot);
			mgct->setGainMatrix(Kmg);
			mgct->setResThreshold(resThres);
			mgct->setTc(robot->getSampleTime());
			mgct->setCutF(fcut);

			// For the moment, clear all the tasks in the stack if you want to use manual guidance
			// TODO: Find a way to merge manual guidance with constraint and other control tasks
			this->stackOfTasks.clear();
			//this->clearTaskClassFromStack(POSITION_CONTROL);

			// Add new Task to the stack
			this->addNewTask(mgct, MIDDLE_LEVEL_PRIORITY, true);

			std::cout << "Control MANUAL_GUIDANCE_MODE set in ControlContext. " << std::endl << std::endl;
		}
		else if (this->systemstate->getCtrlInfo().ctrlMode == TELEOPERATION_MODE) {
			

			// For safety reason, set here the force feedback of the haptic device to 0,0,0
			
			//haptic->setForceFeedback(Eigen::Vector3f::Zero());
			TeleopControlTask* teleop = new TeleopControlTask(POSITION_CONTROL, SPACE_DIM * 2, robot, teleopDev);

			// Add new Task to the stack
			this->addNewTask(teleop, MIDDLE_LEVEL_PRIORITY, true);

			std::cout << "Control TELEOPERATION_MODE set in ControlContext. " << std::endl << std::endl;



		}

		// Print stack state
		this->printStackState();

		// Set the Control Strategy state on running
		this->systemstate->setControlState(RUNNING_STATE);
	}


}

/**
* @brief Update function
* Update the kinematic constraint (among linear motion, angular motion, virtual fixture, RCM and no constraints) based on user request
*/
void ControlContext::checkKinConstraintRequest() {

	// Local pointers to instruments
	std::map < std::string, Instrument* >::iterator robIt, hapDevIt;
	std::string hapticInterfaceName = this->config->getHapticDeviceType();
	RobotInterface* robot = nullptr;
	HapticInterface* haptic = nullptr;

	// Take the pointer of the RobotInterface instance
	robIt = this->instruments.find("Robot");
	hapDevIt = this->instruments.find(hapticInterfaceName.c_str());
	if (robIt != this->instruments.end()) {
		robot = (RobotInterface*)this->instruments["Robot"];
	}
	if (hapDevIt != this->instruments.end()) {
		if (hapticInterfaceName.find("Geomagic") != std::string::npos) {
			haptic = (Geomagic*)this->instruments[hapticInterfaceName.c_str()];
		}
		else if (hapticInterfaceName.find("Armband") != std::string::npos) {
			haptic = (Armband*)this->instruments[hapticInterfaceName.c_str()];
		}
	}

	if (this->systemstate->getConstraintInfo().constraintState == REQUESTED_STATE) {

		if (this->systemstate->getConstraintInfo().constraintType == LINEAR_MOTION_CONSTRAINT) {

			OrientationLockControlTask* olct = new OrientationLockControlTask(KINEMATIC_CONSTRAINT, SPACE_DIM, robot);
			olct->setGainMatrix(0.1);

			// Add new Task to the stack
			this->addNewTask(olct, HIGHEST_PRIORITY, true);

			std::cout << "Constraint LINEAR_MOTION set in ControlContext. " << std::endl << std::endl;
		}
		else if (this->systemstate->getConstraintInfo().constraintType == ANGULAR_MOTION_CONSTRAINT) {

			PositionLockControlTask* plct = new PositionLockControlTask(KINEMATIC_CONSTRAINT, SPACE_DIM, robot);
			plct->setGainMatrix(5.0);

			// Add new Task to the stack
			this->addNewTask(plct, HIGHEST_PRIORITY, true);

			std::cout << "Constraint ANGULAR_MOTION set in ControlContext. " << std::endl << std::endl;
		}
		else if (this->systemstate->getConstraintInfo().constraintType == VIRTUAL_FIXTURE_CONSTRAINT) {

			Eigen::Matrix6i S;
			Eigen::Matrix<float, 5, 5> K; // TODO: generalize for any direction and let user choose from file
			S.setZero(); S(Z_AXIS, Z_AXIS) = 1; // TODO: generalize for any direction and let user choose from file
			K = Eigen::Matrix<float, 5, 5>::Identity();
			K.bottomRows(SPACE_DIM) *= 0.05;

			VirtualFixtureControlTask* vfct = new VirtualFixtureControlTask(KINEMATIC_CONSTRAINT, S, robot);
			vfct->setGainMatrix(K);

			// Add new Task to the stack
			this->addNewTask(vfct, HIGHEST_PRIORITY, true);

			std::cout << "Constraint VIRTUAL_FIXTURE set in ControlContext. " << std::endl << std::endl;
		}
		else if (this->systemstate->getConstraintInfo().constraintType == RCM_CONSTRAINT) {

		}
		else if (this->systemstate->getConstraintInfo().constraintType == NO_CONSTRAINT) {
			this->clearTaskClassFromStack(KINEMATIC_CONSTRAINT); // TODO: This assumes only one constraint to be set. Consider multiple constraints and choose which oen to be cleared
			std::cout << "Kinematic constraints disabled in ControlContext. " << std::endl << std::endl;
		}

		// Print stack state
		this->printStackState();

		// Set the constraint
		//this->strategy->setKinematicConstraint(constraintMat);
		this->systemstate->setConstraintState(SET_STATE);
	}


}


/**
* @brief Add function
* Add a new task to the stack of tasks to be solved
* @param task: the pointer to the ControlTask object to be added to the stack
* @param priority: the desired priority of the task in the stack
* @param oveerideTask: if a task of the same type in the stack must be ovverridden by the new task
*/
void ControlContext::addNewTask(ControlTask* task, const int& priority, const bool& overrideTask){

	std::pair < int, ControlTask* > newTask;
	
	// Instantiate the pair for the new task with priority
	newTask.first = priority;
	newTask.second = task;

	// Check consistency with other tasks and evaluate ovveride
	if (overrideTask) { // If override is on ...
	
		// ... clear analogous tasks from the stack
		this->clearTaskClassFromStack(task->getTaskClass());

	}

	// Add the new task to the stack
	this->stackOfTasks.push_back(newTask);

	// Re-sort the stack according to the new priority
	this->sortTasks();

}

/**
* @brief Update fucntion
* Update the state of the tasks to be solved in the stack
*/
void ControlContext::updateTasks() {

	for (int i = 0; i < this->stackOfTasks.size(); i++) {
	
		this->stackOfTasks[i].second->updateTask();

	}
}

/**
* @brief Sort fucntion
* Sort the tasks to be solved in the stack according to their priority
*/
void ControlContext::sortTasks() {

	std::sort(this->stackOfTasks.begin(), this->stackOfTasks.end());

}

/**
* @brief Solve function
* Apply the task priority formulation to solve the stack of control tasks
* @return the final joint velocity solution vector
*/
Eigen::VectorXf ControlContext::solveControlStack() {

	Eigen::VectorXf qd_k, qd_km1, r1dot, rkdot, uff_k, rk, rkd;
	Eigen::MatrixXf In, J1, J1T, J1pinv, Jk, JkT, K_k, PA, PA_km1, JkPA, JkPAT, JkPApinv;
	float dampFactor; // Damping factor for the DLS solution
	float JPdet;
	int n; // input size
	int k; // task num size
	int td_aug;

	// Local pointers to instruments
	std::map < std::string, Instrument* >::iterator robIt;
	RobotInterface* robot = nullptr;

	// Take the pointer of the RobotInterface instance
	robIt = this->instruments.find("Robot"); // TODO: Abscence of robot should be handled with exception throw. Undefined behaviour is expected otherwise
	if (robIt != this->instruments.end()) {
		robot = (RobotInterface*)this->instruments["Robot"];
		n = robot->getJointNum();
	}

	// Initialize variables of the stack
	qd_k.setZero(n);
	qd_km1.setZero(n);
	In.setIdentity(n, n);
	PA.setIdentity(n, n);
	PA_km1.setIdentity(n, n);

	// Get the number of tasks
	k = this->stackOfTasks.size();
	td_aug = 0;

	for (int i = 0; i < k; i++) {

		//std::cout << "e[" << i << "] = " << this->stackOfTasks[i].second->computeTaskError().transpose() << std::endl;
		
		// Define the Identity matrix of task dim size
		int td_k = this->stackOfTasks[i].second->getTaskDim();
		td_aug += td_k;

		Eigen::MatrixXf Ik = Eigen::MatrixXf::Identity(td_k, td_k);

		// Get the i-th Jacobian matrix
		Jk = this->stackOfTasks[i].second->getTaskJacobian();
		JkT = Jk.transpose();
		
		// Compute the closed-loop input of the task
		rkdot = this->stackOfTasks[i].second->computeClosedLoopInput();

		// Compute the matrix to pseudo-invert
		JkPA = Jk * PA;
		JkPAT = JkPA.transpose();

		// Update the damping factor for the DLS pseudo-inverse
		JPdet = (JkPA * JkPAT).determinant();
		dampFactor = (JPdet <= 1e-10) ? ((JPdet + 1.0)*5e-1) : 0.0;

		// Compute the augmented pseudo-inverse
		if (td_aug < n) {
				JkPApinv = JkPAT * (JkPA * JkPAT + dampFactor*Ik).inverse();
		}
		else {
			JkPApinv = (JkPAT * JkPA + dampFactor*In).inverse() * JkPAT;
		}

		// Compute the i-th joint velocity vector solution
		qd_k = qd_km1 + JkPApinv * (rkdot - Jk * qd_km1);
		qd_km1 = qd_k;

		// Update PA
		PA = PA_km1 - JkPApinv * JkPA;
		PA_km1 = PA;
	}

	return qd_k;
}


/**
* @brief Clear function
* Clear from the stack of tasks all the tasks with given input class
* @param tc: the inpu task class to be cleared from the stack
*/
void ControlContext::clearTaskClassFromStack(const int& tc){

	std::vector < std::pair < int, ControlTask* > >::iterator it;

	for (int i = 0; i < this->stackOfTasks.size(); i++) {
	
		if (this->stackOfTasks[i].second->getTaskClass() == tc) {

			std::vector < std::pair < int, ControlTask* > >::iterator it_found = it;
			delete this->stackOfTasks[i].second;
			this->stackOfTasks.erase(this->stackOfTasks.begin() + i);
			i = 0;
		}

	}

	// Re-sort tasks
	this->sortTasks();

}


/**
* @brief Print function
* Print the status of the stack of tasks
*/
void ControlContext::printStackState() {


	int N = this->stackOfTasks.size();
	std::string className;



	std::cout << "Current Control Stack state: " << std::endl <<
		   		 "\t Control Tasks = " << N << std::endl;

	for (int i = 0; i < N; i++) {
	
		if (this->stackOfTasks[i].second->getTaskClass() == POSITION_CONTROL) {
			className = std::string("Position Control");
		}
		else if (this->stackOfTasks[i].second->getTaskClass() == FORCE_CONTROL) {
			className = std::string("Force Control");
		}
		else if (this->stackOfTasks[i].second->getTaskClass() == JOINT_CONTROL) {
			className = std::string("Joint Control");
		}
		else if (this->stackOfTasks[i].second->getTaskClass() == KINEMATIC_CONSTRAINT) {
			className = std::string("Kinematic Constraint");
		}


		std::cout << "\t Control Task #" << i << ": " << std::endl;
		std::cout << "\t\t Control Task priority: " << this->stackOfTasks[i].first << std::endl;
		std::cout << "\t\t Control Task class: " << className << std::endl;
		std::cout << std::endl;
	}

}


/**
* @brief Clear function
*/
void ControlContext::clear() {

}