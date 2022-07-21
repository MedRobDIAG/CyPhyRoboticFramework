// Project Header files
#include "GeomagicProxy.hpp"

/**
* @brief Default contructor of GeomagicProxy class
*
*/
GeomagicProxy::GeomagicProxy() {

	this->geoHapticProxy = nullptr;
	this->geoTeleopProxy = nullptr;
	this->useHaptics = false;
	this->useTeleop = false;

}

/**
* @brief Contructor of GeomagicProxy class with pointers
* @param hp: the pointer to the HapticProxy component
* @param tp: the pointer to the TeleopProxy component
*
*/
GeomagicProxy::GeomagicProxy(GeomagicHapticProxy* hp, GeomagicTeleopProxy* tp) : geoHapticProxy(hp), geoTeleopProxy(tp) {

}

/**
* @brief Default destroyer of GeomagicProxy class
*
*/
GeomagicProxy::~GeomagicProxy() {

	delete this->geoHapticProxy;
	delete this->geoTeleopProxy;

}


/**
* @brief Default init function
*/
void GeomagicProxy::init(){

	/*if (this->geoHapticProxy != nullptr) {
		this->geoHapticProxy->init();
	}

	if (this->geoTeleopProxy != nullptr) {
		this->geoTeleopProxy->init();
	}//*/

}


/**
* @brief Dynamic instance function
* Create a dynamic instance of the GeomagicHapticProxy class
* @return the pointer to a dynamic instance of GeomagicHapticProxy
*/
GeomagicHapticProxy* GeomagicProxy::instantiateHaptic() {
	
	this->geoHapticProxy = new GeomagicHapticProxy();

	return this->geoHapticProxy;

}

/**
* @brief Dynamic instance function
* Create a dynamic instance of the GeomagicTeleopProxy class
* @return the pointer to a dynamic instance of GeomagicTeleopProxy
*/
GeomagicTeleopProxy* GeomagicProxy::instantiateTeleop() {
	
	this->geoTeleopProxy = new GeomagicTeleopProxy();

	return this->geoTeleopProxy;

}


/**
* @brief Calibrate function
* Calibrate the Geomagic device
* @return true if the device has been successfully calibrated
*/
bool GeomagicProxy::calibrate() {

	if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
		int calibrationStyle;
		hdGetIntegerv(HD_CALIBRATION_STYLE, &calibrationStyle);
		if (calibrationStyle & HD_CALIBRATION_AUTO || calibrationStyle & HD_CALIBRATION_INKWELL) {
			debugPrint<char>("Please prepare for starting the demo by\nplacing the device at its reset position.\n", NULL);
			while (hdCheckCalibration() != HD_CALIBRATION_OK) {}
			return false;
		}
		return true;
	}
	return true;

}


/**
* @brief Default run function
*/
void GeomagicProxy::run() {

	// Required structures
	HDErrorInfo error;

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

	if (HD_DEVICE_ERROR(error = hdGetError())) {
		debugPrint<char>("Failed to start scheduler", NULL);
	}

	// Run the main loop
	while (this->isRunning())
	{

		//----------------------------------------------------------------//
		// Do stuff here... 
		if (this->useTeleop) {
			while (!hdWaitForCompletion(schHandle, HD_WAIT_CHECK_STATUS));
			hdScheduleSynchronous(updateGeoStateCallback, this, HD_DEFAULT_SCHEDULER_PRIORITY);
		}

		//----------------------------------------------------------------//

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
	debugPrint<char>("OK", NULL);



}

/**
* @brief Callback function
* Force feedback callback of the Geomagic device
* @param data the data containing the force feedback data
*/
HDCallbackCode HDCALLBACK forceFeedbackCallback(void* data) {

	GeomagicProxy* device = (GeomagicProxy*)data;
	GeomagicHapticProxy* haptic = device->getHapticHandler();
	hduVector3Dd hduForce;
	HDErrorInfo error;
	hduForce.set(0.0, 0.0, 0.0);
	device->dvcHandle = hdGetCurrentDevice();
	if (haptic != nullptr) {
	

		// Convert the force vector in the appropriate data type of the Geomagic class
		Eigen::VectorXf eigForce = haptic->getHapticForce();
		int hapticDOF = haptic->getHapticDOF();
		for (int i = 0; i < haptic->getHapticDOF(); i++) {
			hduForce[i] = eigForce(i);
		}

		if (device->isHapticsRequested()) {
			hdBeginFrame(device->dvcHandle);
			hdSetDoublev(HD_CURRENT_FORCE, hduForce);
			hdEndFrame(device->dvcHandle);
		}

	}
	else {

		hdBeginFrame(device->dvcHandle);
		hdSetDoublev(HD_CURRENT_FORCE, hduForce);
		hdEndFrame(device->dvcHandle);

	}

	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		hduPrintError(stderr, &error, "Failed");
		device->availability(false);
		if (hduIsSchedulerError(&error))
			return HD_CALLBACK_DONE;
	}
	else { device->availability(true); }
	return HD_CALLBACK_CONTINUE;



}

/**
* @brief Callback function
* Update the state of the Geomagic device
* @param data the data containing the updated status
*/
HDCallbackCode HDCALLBACK updateGeoStateCallback(void* data) {


	GeomagicProxy* device = (GeomagicProxy*)data;
	GeomagicTeleopProxy* teleop = device->getTeleopHandler();
	GeoTeleopState* geoTeleopState = (GeoTeleopState*)(teleop->state);
	Eigen::Matrix<double, (SPACE_DIM * 2), GEOMAGIC_JOINTS> J;
	Eigen::Matrix<double, (SPACE_DIM * 2), 1> vel;
	Eigen::Matrix<double, GEOMAGIC_JOINTS, 1> q, qdot, qprev;
	bool curButton[GEOMAGIC_BUTTONS_NUM];
	double hdrate;

	HDErrorInfo error;
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		hduPrintError(stderr, &error, "Failed");
		if (hduIsSchedulerError(&error))
			return HD_CALLBACK_DONE;
	}
	else {

		// Get data from the device
		hduVector3Dd pos, abg;
		double jacobian[SPACE_DIM * 2 * GEOMAGIC_JOINTS];
		double jointPosition[GEOMAGIC_JOINTS];
		double jointVelocity[GEOMAGIC_JOINTS];

		hdGetDoublev(HD_CURRENT_POSITION, pos);
		hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, abg);
		hdGetDoublev(HD_CURRENT_JACOBIAN, jacobian);
		hdGetDoublev(HD_CURRENT_JOINT_ANGLES, jointPosition);
		//hdGetDoublev(HD_UPDATE_RATE, &hdrate);

		/*for (int i = 0; i < 6; i++) {
			for (int j = 0; j < 6; j++) {
				J(i, j) = device->geoStatus.jacobian[i * 6 + j];
			}
		}
		std::memcpy(q.data(), device->geoStatus.jointPosition, sizeof(double) * GEOMAGIC_JOINTS);
		std::memcpy(qprev.data(), device->hdUtils.jointPosPrev, sizeof(double) * GEOMAGIC_JOINTS);//*/

		//std::cout << std::endl;

		// Convert from [mm] to [m]
		pos *= 1e-3;


		// Fill internally the TeleopState structure
		teleop->setStateHIPPosition(hdu2Eigen3D(pos).cast<float>());
		teleop->setStateHIPGimbalAngles(hdu2Eigen3D(abg).cast<float>());
		teleop->updateVelocities(); // Set internally the velocity vectors in TeleopState

		/*vel = J * (q - qprev)*(1.0/hdrate);
		if (vel.norm() > 1e3) vel.setZero();
		std::cout << "vel = " << vel.transpose() << std::endl;
		hduVector3Dd linvel(vel(0), vel(1), vel(2));
		hduVector3Dd angvel(vel(3), vel(4), vel(5));
		device->geoStatus.stylusLinearVelocity = linvel;
		device->geoStatus.stylusAngularVelocity = angvel;
		std::memcpy(device->hdUtils.jointPosPrev,device->geoStatus.jointPosition,sizeof(double)*HAPTIC_JOINTS);//*/

		// Get the state of the button
		int stylusButtons;
		hdGetIntegerv(HD_CURRENT_BUTTONS, &stylusButtons);

		for (int i = 0; i < GEOMAGIC_BUTTONS_NUM; i++) {
			curButton[i] = (stylusButtons == i + 1 || stylusButtons == PRESSED_BOTH) ? true : false;

			// Catch the single pressure event
			teleop->catchButtonPressEvent(curButton[i], geoTeleopState->evHoldButton[i],
				geoTeleopState->evRaiseEdge[i],
				geoTeleopState->evTrailEdge[i],
				geoTeleopState->action[i]);

			geoTeleopState->buttonState[i] = geoTeleopState->action[i];
		}

		
	}


	return HD_CALLBACK_DONE;


}



/**
* @brief Default clear function
*/
void GeomagicProxy::clear() {

	// Set running on false
	this->setRunning(false);


}
