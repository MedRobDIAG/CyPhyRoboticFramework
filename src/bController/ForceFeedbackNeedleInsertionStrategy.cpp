// Project Header files
#include "ForceFeedbackNeedleInsertionStrategy.hpp"
#include "Configuration.hpp"

/*
* @brief Constructor of ForceFeedbackNeedleInsertionStrategy
* @param p: the pointer to Phantom
* @param ft: the pointer to FTSesnor
* @param hs: a map list of haptic interfaces
* @param v: the pointer to VREPProxy
* @param r: the pointer to RobotInterface
*/
ForceFeedbackNeedleInsertionStrategy::ForceFeedbackNeedleInsertionStrategy(Phantom* p, FTSensor* ft, std::map <std::string, HapticInterface* > hs, VREPProxy* v, RobotInterface* r) : TaskStrategy(v, r) {

	// Initialize variables
	this->phantom = p;
	this->ftsensor = ft;
	
	std::map <std::string, HapticInterface* >::iterator hIt;
	for (hIt = hs.begin(); hIt != hs.end(); ++hIt) {
		this->haptics.insert({ hIt->first, hIt->second });
	}
	

}

/*
* @brief Default destroyer
*
*/
ForceFeedbackNeedleInsertionStrategy::~ForceFeedbackNeedleInsertionStrategy() {}

/**
* @brief Init function
* Initialize the TaskStrategy class
*/
void ForceFeedbackNeedleInsertionStrategy::init() {

	Eigen::Vector3f pwr;
	Eigen::Quaternionf qwr;
	float pwr_v[SPACE_DIM], qwr_v[SPACE_DIM + 1];

	// Get Singleton Configuration object
	Configuration* config = Configuration::GetInstance("");

	// Load the Phantom pose, if requested, and set it in the simulator, if available
	if (config->useRegisteredPhantom()) {

		// Get the Transformation filename
		std::string regFileName = config->getRegTransfFile();

		// Load the transformation robot->phantom obtained from a registration procedure
		this->phantom->loadTransformation(regFileName.c_str());

		// Set the pose of the phantom in the simulator
		Eigen::Matrix4d Trp = this->phantom->getTransformation();
		this->vrep->setObjectPose("_Phantom", Trp, "KukaOrigin", simx_opmode_blocking, this->simPort);
	}

	// Initialize the streaming for the layer counter signal from the simulated scene
	this->vrep->getFloatSignal("layerID", this->simPuncturedLayerCounter, simx_opmode_streaming, this->simPort);

	// Draw line
	this->drawNeedleLine = false;

	// Set Twr
	this->vrep->getObjectPosition("KukaOrigin", pwr_v, "world", simx_opmode_blocking, this->simPort);
	this->vrep->getObjectQuaternion("KukaOrigin", qwr_v, "world", simx_opmode_blocking, this->simPort);
	std::memcpy(pwr.data(), pwr_v, sizeof(float) * SPACE_DIM);
	std::memcpy(qwr.coeffs().data(), qwr_v, sizeof(float) * (SPACE_DIM + 1));
	this->Twr.topLeftCorner(SPACE_DIM, SPACE_DIM) = qwr.toRotationMatrix();
	this->Twr.block<3, 1>(0, 3) = pwr;

	this->forcefeedback.setZero();
	this->anchor.setZero();
	this->cumulativeFriction = 0.0;
	this->simPuncturedLayerCounter = 0;
	this->currentLayerID = 0;

	// Initialize the phantom
	int numLayers = 5+1; // first is empty space
	for (int i = 0; i < numLayers; i++) {
		this->staticPhantom.addNewLayer(new TissueLayer());
	}

	// Fill the interaction parameters of the layers
	// Stiffness
	this->staticPhantom.setLayerKcoeff(0, 0);
	this->staticPhantom.setLayerKcoeff(1, 200);
	this->staticPhantom.setLayerKcoeff(2, 300);
	this->staticPhantom.setLayerKcoeff(3, 500);
	this->staticPhantom.setLayerKcoeff(4, 400);
	this->staticPhantom.setLayerKcoeff(5, 300);

	// Friction
	this->staticPhantom.setLayerDcoeff(0, 0);
	this->staticPhantom.setLayerDcoeff(1, 700);
	this->staticPhantom.setLayerDcoeff(2, 800);
	this->staticPhantom.setLayerDcoeff(3, 1000);
	this->staticPhantom.setLayerDcoeff(4, 900);
	this->staticPhantom.setLayerDcoeff(5, 1000);

	// Rupture threshold
	this->staticPhantom.setLayerRuptureThreshold(0, 0);
	this->staticPhantom.setLayerRuptureThreshold(1, 0.8);
	this->staticPhantom.setLayerRuptureThreshold(2, 1.0);
	this->staticPhantom.setLayerRuptureThreshold(3, 1.5);
	this->staticPhantom.setLayerRuptureThreshold(4, 1.2);
	this->staticPhantom.setLayerRuptureThreshold(5, 1.5);

	// Depths (default -1)
	this->staticPhantom.setLayerDepth(0, 0);
	this->staticPhantom.setLayerDepth(1, 0);
	this->staticPhantom.setLayerDepth(2, 0);
	this->staticPhantom.setLayerDepth(3, 0);
	this->staticPhantom.setLayerDepth(4, 0);
	this->staticPhantom.setLayerDepth(5, 0);

	this->staticPhantom.setLayerPunctured(0, false);
	this->staticPhantom.setLayerPunctured(1, false);
	this->staticPhantom.setLayerPunctured(2, false);
	this->staticPhantom.setLayerPunctured(3, false);
	this->staticPhantom.setLayerPunctured(4, false);
	this->staticPhantom.setLayerPunctured(5, false);

}

/**
* @brief Main function
* Implements the task of the NeedleTissueIntEstStrategy
* @return the return state of the routine
*/
int ForceFeedbackNeedleInsertionStrategy::execTask() {

	int ret;
	float simLayerCounter_prev;
	Eigen::Vector3f forceFb_cur;

	// Get Singleton Configuration object
	Configuration* config = Configuration::GetInstance("");
	std::vector < std::string > hapticListNames = config->getHapticDeviceList();

	// Get Singleton SystemState object
	SystemState* sys = SystemState::GetInstance();


	//--------------------- vvv Main core of the task vvv ------------------------//

	// Until the first layer is detected, update z0
	if (this->simPuncturedLayerCounter == 0) {
		this->staticPhantom.setZ0(this->robot->getEEPosition()(2));
	}

	// Get the layer transition detection signal from the simulated scene
	simLayerCounter_prev = this->simPuncturedLayerCounter;
	this->vrep->getFloatSignal("layerID", this->simPuncturedLayerCounter, simx_opmode_buffer, this->simPort);
	
	// if on the raising edge of the signal...
	if (this->simPuncturedLayerCounter != simLayerCounter_prev) {

		// Update the layer ID
		this->currentLayerID++;

		// Update the anchor point
		Eigen::Vector3f anchor_prev = this->anchor;
		this->anchor = this->robot->getEEPosition();
		this->z_anchors.push_back(this->anchor(Z_AXIS));

		// Fill the vector from the second layer on
		if (this->simPuncturedLayerCounter > 1) { // I need this because I don't want to set the depth for the empty space, so I skip the first depth set
			this->staticPhantom.setLayerDepth(this->simPuncturedLayerCounter - 1, -(this->anchor(Z_AXIS) - anchor_prev(Z_AXIS)));
		}


	}

	if (this->currentLayerID > 0 && this->robot->getEEPosition()(Z_AXIS) > this->anchor(Z_AXIS)) {
				

		// Update the layer ID
		this->currentLayerID--;
		if (currentLayerID > 0) {
			this->anchor = Eigen::Vector3f(0, 0, this->z_anchors[this->currentLayerID-1]);
		}

	}

	std::vector < Eigen::VectorXf > f_feedback;
	float frictionFeedbackValue;
	// Compute the force feedback signal
	//// Different options are possible at this point (only one is kept intentionally uncommented):
	//// 
	//// +++ Example 1: use as haptic device the first one specified in the list (choseHapticIdx = 0)

	/*int chosenHapticIdx = 0;
	f_feedback = this->computeForceFeedback(chosenHapticIdx);
	this->forcefeedback = f_feedback[0];
	this->haptics[hapticListNames[chosenHapticIdx]]->setForceFeedback(this->forcefeedback);//*/
	////
	//// +++ Example 2: if available, use the Geomagic device
	/*std::vector < std::string >::iterator hIt = std::find(hapticListNames.begin(), hapticListNames.end(), "Geomagic");
	if (hIt != hapticListNames.end()) {
		int i = std::distance(hapticListNames.begin(), hIt);
		this->forcefeedback = this->computeForceFeedback(i);
		this->haptics["Geomagic"]->setForceFeedback(this->forcefeedback);
	}//*/
	////
	//// +++ Example 3: if available, use the Armband device
	/*std::vector < std::string >::iterator hIt = std::find(hapticListNames.begin(), hapticListNames.end(), "Armband");
	if (hIt != hapticListNames.end()) {
		int i = std::distance(hapticListNames.begin(), hIt);
		this->forcefeedback = this->computeForceFeedback(i);
		this->haptics["Armband"]->setForceFeedback(this->forcefeedback);
	}//*/
	//// +++ Example 4: apply the same force feedback information to all the haptic devices
	/*for (int i = 0; i < hapticListNames.size(); i++) {
		this->forcefeedback = this->computeForceFeedback(i);
		this->haptics[hapticListNames[i]]->setForceFeedback(this->forcefeedback);
	}//*/
	//// +++ Example 5: if available, use the Weart device
	/*std::vector < std::string >::iterator hIt = std::find(hapticListNames.begin(), hapticListNames.end(), "TouchDIVER");
	if (hIt != hapticListNames.end()) {
		int i = std::distance(hapticListNames.begin(), hIt);
		f_feedback = this->computeForceFeedback(i);
		this->forcefeedback = f_feedback[2];
		frictionFeedbackValue = f_feedback[1](0);
		this->haptics["TouchDIVER"]->setForceFeedback(f_feedback[2]);
		dynamic_cast<WeartDevice*>(this->haptics["TouchDIVER"])->setVibrationVolume(frictionFeedbackValue);
	}//*/

	int chosenHapticIdx = 0;
	f_feedback = this->computeForceFeedback(chosenHapticIdx);
	if (hapticListNames[chosenHapticIdx] == "TouchDIVER") {
		this->forcefeedback = f_feedback[2];
		frictionFeedbackValue = f_feedback[1](0);
		//this->haptics["TouchDIVER"]->setForceFeedback(f_feedback[2]);
		dynamic_cast<WeartDevice*>(this->haptics["TouchDIVER"])->setVibrationVolume(frictionFeedbackValue);

		// Send the force feedback signal on CoppeliaSim/V-REP
		this->vrep->setFloatSignal("forceFeedback", this->forcefeedback(X_AXIS), simx_opmode_oneshot, this->simPort);
	}
	else if (hapticListNames[chosenHapticIdx] == "Geomagic") {
		this->forcefeedback = f_feedback[0]; // total force

		// Send the force feedback signal on CoppeliaSim/V-REP
		this->vrep->setFloatSignal("forceFeedback", this->forcefeedback(Y_AXIS), simx_opmode_oneshot, this->simPort);
	}
	this->haptics[hapticListNames[chosenHapticIdx]]->setForceFeedback(this->forcefeedback);//*/


	//--------------------- ^^^ Main core of the task ^^^ ------------------------//

	// Check for visualization of the needle direction in the simulator
	if (sys->getShow() == ACTION::REQUESTED) {
		this->showNeedleDirection();
		sys->setShow(ACTION::CLEARED);
	}

	// Check for termination request and set flag
	if (sys->getAction() == ACTION::REQUESTED) {
		
		ret = TASK_RETURN_CODE::FINISHED;
		sys->setAction(ACTION::CLEARED);

	}
	else {

		ret = TASK_RETURN_CODE::PROCESSING;
	}


	return ret;
}

/**
* @brief Force feedback computation function
* Compute the force feedback signal to be sent back to the haptic device
* @param: integer id specifying the haptic device to be accounted on which the feedback is expected to be sent (default is 0, i.e., the first
* haptic device of the specified list
* @return the 3D vector of the linear force feedback signal
*/
std::vector < Eigen::VectorXf > ForceFeedbackNeedleInsertionStrategy::computeForceFeedback(const int& hapticIdx){

	std::vector < Eigen::VectorXf > f_out;
	Eigen::Vector3f fb_s;
	Eigen::VectorXf fb_m;
	Eigen::MatrixXf hapticRms;
	Eigen::Vector3f pbee, vbee;
	TissueLayer layer_i;
	float Ki, Bi, rupture_thresh_i;
	float fEl, fFric, fSumFric;
	float fscale;

	// Select the Haptic device from which retrieve the rotation matrix and the scaling factor
	Configuration* config = Configuration::GetInstance("");
	std::vector < std::string > hapticList = config->getHapticDeviceList();
	HapticInterface* chosenHaptic = this->haptics[hapticList[hapticIdx]];

	// Initialize variables
	fb_s.setZero();

	// Get the slave-to-master rotation matrix
	hapticRms = chosenHaptic->getRms();

	// Get the haptic force scale factor
	fscale = chosenHaptic->getForceScale();

	// Get the current needle tip position
	pbee = this->robot->getEEPosition();

	// Get the current needle tip velocity
	vbee = this->robot->getEEVelocity().topRows(SPACE_DIM);

	// Get paramters of the current layer
	layer_i = this->staticPhantom.getLayerInfo(this->currentLayerID);
	Ki = layer_i.getKcoeff();
	Bi = layer_i.getDcoeff();
	rupture_thresh_i = layer_i.getRuptThresh();

	// Compute the force contributions
	if (!this->staticPhantom.getLayerInfo(this->currentLayerID).isPunctured()) {
		fEl = -Ki * (pbee(Z_AXIS) - anchor(Z_AXIS));
	}
	else {
		fEl = 0.0;
	}
	fFric = Bi * (pbee(Z_AXIS) - anchor(Z_AXIS)) * vbee(Z_AXIS);
	fSumFric = 0.0;
	for (int j = 0; j < this->currentLayerID; j++) {
		float Bj = this->staticPhantom.getLayerInfo(j).getDcoeff();
		float dj = this->staticPhantom.getLayerInfo(j).getDepth();
		fSumFric -= Bj * dj;
	}
	fSumFric *= vbee(Z_AXIS);

	// Compute the simulated force
	if (std::fabs(fEl) > rupture_thresh_i) {
		fEl = 0;
		this->staticPhantom.setLayerPunctured(this->currentLayerID, true);
	}

	fb_s(Z_AXIS) = fEl +fFric + fSumFric;

	fb_m = hapticRms * fb_s;
	if (std::fabs(fb_m(Y_AXIS)) > 2) {
		fb_m(Y_AXIS) = 2;
	}

	Eigen::VectorXf fFriction(3), fElastic(3);
	fFriction << 0.0, 0.0, fFric + fSumFric;
	fElastic << 0.0, 0.0, fEl;
	f_out.push_back(fb_m * fscale);
	f_out.push_back(hapticRms * fFriction * fscale);
	f_out.push_back(hapticRms * fElastic * fscale);

	//std::cout << "fFriction = " << fFriction(2) << std::endl;

	return f_out;// (fb_m * fscale);

}



/**
* @brief Terminate function
* Terminate the ForceFeedbackNeedleInsertionStrategy class
*/
void ForceFeedbackNeedleInsertionStrategy::terminate() {}



/**
* @brief Visualization function
* Request to the VREP simulator to render the line of the needle direction in the virtual scene
*/
void ForceFeedbackNeedleInsertionStrategy::showNeedleDirection() {

	Eigen::Matrix3f Rwr;
	Eigen::Vector3f pwr;

	Rwr = this->Twr.topLeftCorner(SPACE_DIM, SPACE_DIM);
	pwr = this->Twr.block<3, 1>(0, 3);

	// Draw insertion line in V-REP
	this->drawNeedleLine = !this->drawNeedleLine;
	std::cout << ((this->drawNeedleLine) ? "Drawing" : "Clearing") << " needle direction line" << std::endl;
	Eigen::Vector3f needleDir_k = robot->getEERotMat().col(Z_AXIS);
	Eigen::Vector3f needlePos_k = robot->getEEPosition();
	Eigen::Vector3f needleDir_w = Rwr * needleDir_k;
	Eigen::Vector3f needlePos_w = Rwr * needlePos_k + pwr;

	this->vrep->setFloatSignal("drawLineSig", this->drawNeedleLine, simx_opmode_oneshot, this->simPort);
	this->vrep->setFloatSignal("needleDir_x", needleDir_w(X_AXIS), simx_opmode_oneshot, this->simPort);
	this->vrep->setFloatSignal("needleDir_y", needleDir_w(Y_AXIS), simx_opmode_oneshot, this->simPort);
	this->vrep->setFloatSignal("needleDir_z", needleDir_w(Z_AXIS), simx_opmode_oneshot, this->simPort);
	this->vrep->setFloatSignal("needlePos_x", needlePos_w(X_AXIS), simx_opmode_oneshot, this->simPort);
	this->vrep->setFloatSignal("needlePos_y", needlePos_w(Y_AXIS), simx_opmode_oneshot, this->simPort);
	this->vrep->setFloatSignal("needlePos_z", needlePos_w(Z_AXIS), simx_opmode_oneshot, this->simPort);

}
