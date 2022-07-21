// Project Header files
#include "GeomagicHapticProxy.hpp"
#include "Timer.hpp"

/**
* @brief Default contructor of GeomagicHapticProxy class
*
*/
GeomagicHapticProxy::GeomagicHapticProxy() : HapticProxy() {


	this->setHapticDOF(GEOMAGIC_HAPTIC_DOF);

}

/**
* @brief Default destroyer of GeomagicHapticProxy class
*
*/
GeomagicHapticProxy::~GeomagicHapticProxy() {}

/**
* @brief Default init function
*/
void GeomagicHapticProxy::init() {

	// Debug print
	debugPrint<char>("[GeomagicHapticProxy] This is a debug print", NULL);

	/*** Because of the inner working conditions of the haptic device and its OpenHaptics SDK, 
	 *** this init() function here cannot initialize the device, because the main loop execution 
	 *** occurs in an another thread. It seems that initialization, utilization and clear of the device
	 *** has to occur in the same thread. Therefore this function, that is called by Scheduler in the main thread,
	 *** loses its use, and it should just initialize local structures, but not the device. 
	 *** Device initialization and clear and then left to the same function of the main loop (i.e. before and after
	 *** the while() loop). 
	 ***/

	this->hapticDOF = GEOMAGIC_HAPTIC_DOF;
	this->hapticState.force.setZero(this->hapticDOF);
	this->availability(true);

}




/**
* @brief Get function
* Retrieves the 6D velocity vector from the Geomagic Proxy
* @param v [out] the 6D velocity vector
*/
/*void GeomagicHapticProxy::getHIPVelocity(float v[SPACE_DIM * 2]) {

	for (int i = 0; i < SPACE_DIM; i++) {
		v[i] = (float)this->geoStatus.stylusLinearVelocity[i];
	}
	for (int i = 0; i < SPACE_DIM; i++) {
		v[i + 3] = (float)this->geoStatus.stylusAngularVelocity[i];
	}

}//*/


/**
* @brief Default run function
*/
void GeomagicHapticProxy::run() {

	/**** vvvvv maybe not needed here vvv  ****/
	// Time variables
	/*double tictoc, dt, rate, tic, toc, tac, Ts;

	// Get the clock rate
	Timer clock;
	clock.setRate(20.0);
	rate = clock.getRate();
	Ts = 1.0 / rate;
	dt = 0.0;*/
	/**** ^^^^^ maybe not needed here ^^^^^ ****/

	// Required structures
	/*HDErrorInfo error;

	// Init the device
	this->dvcHandle = hdInitDevice(HD_DEFAULT_DEVICE);
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		debugPrint<char>("Failed to initialize haptic device. Code: 0x0", NULL);
		this->availability(false);
		return;
	}

	// Calibrate the device
	int ok = calibrate();

	// If calibration fails ...
	if (!ok) {
		debugPrint<char>("Calibration failed!", NULL);
	}
	//hdDisable(HD_SOFTWARE_VELOCITY_LIMIT);

	// Set running on true
	this->setRunning(true);

	std::cout << "Master control loop started!\n";

	// Set the force feedback callback in the scheduler
	this->schHandle = hdScheduleAsynchronous(forceFeedbackCallback, this, HD_MAX_SCHEDULER_PRIORITY);
	
	// Enable force feedback
	hdEnable(HD_FORCE_OUTPUT);

	// Start the scheduler
	//hdSetSchedulerRate(800);
	hdStartScheduler();

	if (HD_DEVICE_ERROR(error = hdGetError())){
	debugPrint<char>("Failed to start scheduler",NULL);
	}

	// Run the main loop
	while (this->isRunning())
	{
		// Measure starting time
		tic = clock.getCurTime();

		

		//----------------------------------------------------------------//
		// Do stuff here... 


		while (!hdWaitForCompletion(schHandle, HD_WAIT_CHECK_STATUS));
		hdScheduleSynchronous(updateGeoStateCallback, this, HD_DEFAULT_SCHEDULER_PRIORITY);

		//----------------------------------------------------------------//

		// Measure the ending time and the elapsed time
		toc = clock.getCurTime();
		tictoc = clock.elapsedTime(tic, toc);

		// Wait until Ts
		if (tictoc < Ts) {
			//clock.timeSleep(Ts - tictoc); //<----- DO NOT UNCOMMENT THIS! CONFLICT WITH THE INNER DEVICE SERVO LOOP! 
		}

		// Measure the final time after sleep to check the actual rate of the thread
		tac = clock.getCurTime();
		dt = clock.elapsedTime(tic, tac);

		//debugPrint<double>("[GeomagicHapticProxy] Running rate:", 1.0 / dt);
	}

	// Stop the scheduler
	debugPrint<char>("Stopping the device scheduler...", NULL);
	hdStopScheduler();
	debugPrint<char>("OK", NULL);

	// Unschedule the tasks
	hdUnschedule(schHandle);

	// Disable the device
	debugPrint<char>("Disabling the device...", NULL);
	hdDisableDevice(dvcHandle);
	debugPrint<char>("OK", NULL);*/

}

/**
* @brief Default clear function
*/
void GeomagicHapticProxy::clear() {

	// Set running on false
	this->setRunning(false);

}





