// Project Header files 
#include "KeyboardProxy.hpp"

// Standard Header files
#include <iostream>

/**
* @brief Default contructor of KeyboardProxy class
*
*/
KeyboardProxy::KeyboardProxy() : UserProxy() {

	// Init the class
	this->init();

}

/**
* @brief Default destroyer of UserProxy class
*
*/
KeyboardProxy::~KeyboardProxy() {}


/**
* @brief Default init function
*/
void KeyboardProxy::init() {

	// Init variables
	this->kbdata.keyhit = false;
	this->kbdata.key = -1;

	// Set availability and running to true
	this->available = true;
	this->running = true;
}

/**
* @brief Default run function
*/
void KeyboardProxy::run() {

	// Local variables
	bool kbhitFlag;
	int key, userChoice;

	// Time variables
	double tictoc, dt, rate, tic, toc, tac, Ts;
	Timer clock;
	clock.setRate(20.0);
	rate = clock.getRate();
	Ts = 1.0 / rate;


	// Wait for some seconds to show the task menu on screen correctly (TODO: find a smart way and remove this horror)
	clock.timeSleep(1.0);

	// Let the user choose the task
	userChoice = this->chooseTask();

	while (this->isRunning()) {

		// Measure starting time
		tic = clock.getCurTime();

		//----------------------------------------------------------------//
		// Do stuff here... 

		// If a key is pressed ...
		kbhitFlag = this->kbhit();

		if (kbhitFlag) {

			// Get the current value
			key = this->getChar();

			//std::cout << "key: " << key << std::endl;

			// If the key is ESC, terminate
			if (key == ESCAPE) {
				this->sys.setSystemTaskState(STOPPING_STATE);
				this->sys.setSystemControlState(STOPPING_STATE);
			}
			else if (key == ENTER) { // Action request
				//...
				this->sys.setSystemAction(ACTION::REQUESTED);
			}
			else if (key == (int)'s') { // Request to reset F/T sensor bias
				//...
				this->sys.resetSystemFTSensorBias();
				std::cout << "F/T Sensor bias reset done. " << std::endl;
			}
			else if (key == (int)'v') { // Request to set Linear motion constraint
				//...
				this->sys.setSystemConstraintState(REQUESTED_STATE);
				this->sys.setSystemConstraintType(LINEAR_MOTION_CONSTRAINT);
				std::cout << "Requested LINEAR_MOTION kinematic constraint " << std::endl;
			}
			else if (key == (int)'w') { // Request to set Angular motion constraint
										//...
				this->sys.setSystemConstraintState(REQUESTED_STATE);
				this->sys.setSystemConstraintType(ANGULAR_MOTION_CONSTRAINT);
				std::cout << "Requested ANGULAR_MOTION kinematic constraint " << std::endl;
			}
			else if (key == (int)'f') { // Request to set virtual fixture constraint
				//...
				this->sys.setSystemConstraintState(REQUESTED_STATE);
				this->sys.setSystemConstraintType(VIRTUAL_FIXTURE_CONSTRAINT);
				std::cout << "Requested VIRTUAL_FIXTURE kinematic constraint " << std::endl;
			}
			else if (key == (int)'n') { // Request to disable motion constraint
				//...
				this->sys.setSystemConstraintState(REQUESTED_STATE);
				this->sys.setSystemConstraintType(NO_CONSTRAINT);
				std::cout << "Requested kinematic constraint disabling " << std::endl;

			}
			else if (key == (int)'t') { // Request to enable Teleoperation control
				//...
				this->sys.setSystemControlMode(TELEOPERATION_MODE);
				this->sys.setSystemControlState(REQUESTED_STATE);
				std::cout << "Requested TELEOPERATION_MODE control from Keyboard " << std::endl;
			}
			else if (key == (int)'a') { // Request to enable Autonomous control
				//...
				this->sys.setSystemControlMode(AUTONOMOUS_MODE);
				this->sys.setSystemControlState(REQUESTED_STATE);
				std::cout << "Requested AUTONOMOUS_MODE control from Keyboard " << std::endl;
			}
			else if (key == (int)'m') { // Request to enable Manual Guidance control
				//...
				this->sys.setSystemControlMode(MANUAL_GUIDANCE_MODE);
				this->sys.setSystemControlState(REQUESTED_STATE);
				std::cout << "Requested MANUAL_GUIDANCE_MODE control from Keyboard " << std::endl;
			}
			else if (key == (int)'q') { // Request to close logging
				//...
			}
			else if (key == (int)'l') { // Request to draw virtual line in the simulator
				//...
				this->sys.setSystemShowFcn(ACTION::REQUESTED);
			}
			else if (key == (int)'e') { // Request to enable/disable force feedback enhancement in teleoperation control
				//...
			}
			else if (key == (int)'h') { // Request to print the set of instructions
				//...
				this->printInstructions();
			}


		}

		// If task has stopped, ask user for a new task to launch
		if (this->sys.getSystemTaskState() == STATE::STOPPED_STATE) {

			// Let the user choose the next task
			userChoice = this->chooseTask();

		}


		// Write keyboard status on the shared memory
		{
			std::lock_guard<std::mutex> lock(kbmtx);
			this->kbdata.key = (kbhitFlag) ? key : -1;
			this->kbdata.keyhit = kbhitFlag;
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
		//this->simTime += Ts;
		//std::cout << "[Key] Running rate:" << (1.0 / dt) << std::endl;

	}

}

/**
* @brief Default clear function
*/
void KeyboardProxy::clear() {}

/**
* @brief Check if a key has been hit or not
* @return boolean value
*/
bool KeyboardProxy::kbhit() {

	bool kbhitFlag;

#ifdef _WIN32
	kbhitFlag = (_kbhit() == 1) ? true : false;
#elif unix
	kbhitFlag = false;
#endif

	return kbhitFlag;

}

/**
* @brief Get the current char key upon pressure
* @return the pressed key
*/
int KeyboardProxy::getChar() {

	int key;

#ifdef _WIN32
	// Acquire hit character
	key = _getch();
#elif unix
	kbhitFlag = false;
#endif

	return key;
}


/**
* @brief Selection function
* Let the user choose the next task to run
*/
int KeyboardProxy::chooseTask() {

	int choice;

	this->sys.lockKBMutexInSysState();

	// Show Menu
	this->showMenu();

	// Capture user input
	std::cin >> choice;

	while (choice >= TOTAL_INPUTS) { // Wrong input

		std::cout << "Wrong input." << std::endl;
		this->showMenu();

		// Capture user input
		std::cin >> choice;
	}

	this->sys.unlockKBMutexInSysState();


	if (choice == QUIT_INPUT) {

		std::cout << "Exit program requeted. Closing the application... " << std::endl;

		// Request SystemManager to close everything and exit
		this->sys.exit();

		// Exit from this thread
		this->setRunning(false);
	}
	else {

		if (choice == USER_INPUTS::REGISTRATION_INPUT) {
			std::cout << "Registration Task requested. Press ENTER to start the Task ... " << std::endl;
		}
		else if (choice == USER_INPUTS::PLANNING_INPUT) {
			std::cout << "Planning Task requested. Press ENTER to start the Task ... " << std::endl;
		}
		else if (choice == USER_INPUTS::NEEDLE_TISSUE_KV_ESTIMATION_INPUT) {
			std::cout << "Needle Tissue Interaction Estimation (Kelvin-Voigt) Task requested. Press ENTER to start the Task ... " << std::endl;
		}
		else if (choice == USER_INPUTS::NEEDLE_TISSUE_GEROVICH_ESTIMATION_INPUT) {
			std::cout << "Needle Tissue Interaction Estimation (Gerovich) Task requested. Press ENTER to start the Task ... " << std::endl;
		}
		else if (choice == USER_INPUTS::FORCE_FEEDBACK_NEEDLE_INSERTION_INPUT) {
			std::cout << "Force Feedback Needle Insertion Task requested. Press ENTER to start the Task ... " << std::endl;
		}

		this->sys.setSystemTaskType(choice);
		this->sys.setSystemTaskState(STATE::REQUESTED_STATE);

	}

	return choice;
}

void KeyboardProxy::showMenu() {


	std::cout << "Select the desired application to run: \n"
		<< "1. Registration \n"
		<< "2. Interactive Surgical Planning \n"
		<< "3. Needle-Tissue interaction parameter identification \n"
		<< "4. Friction Estimation Task \n"
		<< "5. Force Feedback Needle Insertion Task \n\n"
		<< "0. Quit \n\n";

}

/**
* @brief print function
* Show the instructions on the screen that the user can choose during the experimental session
*/
void KeyboardProxy::printInstructions() {

	std::cout << "Instructions: \n"
		<< "\t's'   = " << "F/T Sensor bias reset " << std::endl
		<< "\t'l'   = " << "Start Log" << std::endl
		<< "\t'q'   = " << "Quit log and ask for log comment" << std::endl
		<< "\t'v'   = " << "Linear motion constraint" << std::endl
		<< "\t'w'   = " << "Angular motion constraint" << std::endl
		<< "\t'f'   = " << "Virtual fixture constraint" << std::endl
		<< "\t't'   = " << "Enable/Disable the Teleoperation mode" << std::endl
		<< "\t'a'   = " << "Enable/Disable Autonomous insertion mode" << std::endl
		<< "\t'm'   = " << "Enable/Disable Manual Guidance mode" << std::endl
		<< "\t'e'   = " << "Enable/Disable force feedback in Teleoperation mode" << std::endl
		<< "\t'h'   = " << "Print this menu" << std::endl << std::endl
		<< "\tENTER = " << "Start/Stop the chosen Task" << std::endl
		<< "\tSPACE = " << "Generic Trigger input for Task" << std::endl << std::endl
		<< "\tESC   = " << "Quit the program" << std::endl << std::endl;


}
