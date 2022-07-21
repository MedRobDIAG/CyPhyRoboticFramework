// Project Header Files
#include "NeedleTissueIntEstStrategy.hpp"
#include "Configuration.hpp"
#include "SystemState.hpp"
#include "Dataset.hpp"
#include "KUKARobot.hpp"
#include "MultiLayerGelPhantom.hpp"

// Standard Header files
#include <numeric>
#include <algorithm>

// System Header file
#include <Windows.h>

/*
* @brief Constructor of NeedleTissueIntEstStrategy
* @param p: the pointer to Phantom
* @param ft: the pointer to FTSesnor
* @param hs: a map list of haptic interfaces
* @param v: the pointer to VREPProxy
* @param r: the pointer to RobotInterface
*/
NeedleTissueIntEstStrategy::NeedleTissueIntEstStrategy(Phantom* p, FTSensor* ft, std::map< std::string , HapticInterface* > hs, VREPProxy* v, RobotInterface* r) : TaskStrategy(v, r) {

	// Initialize variables
	this->phantom = p;
	this->ftsensor = ft;

	std::map <std::string, HapticInterface* >::iterator hIt;
	for (hIt = hs.begin(); hIt != hs.end(); ++hIt) {
		this->haptics.insert({ hIt->first, hIt->second });
	}
}

/*
* @brief Destroyer of NeedleTissueIntEstStrategy
*/
NeedleTissueIntEstStrategy::~NeedleTissueIntEstStrategy() {}

/**
* @brief Init function
* Initialize the TaskStrategy class
*/
void NeedleTissueIntEstStrategy::init() {

	std::vector <float> varGains;
	Eigen::VectorXf Psi0diag;
	Eigen::Vector3f pwr;
	Eigen::Quaternionf qwr;
	Eigen::VectorXf theta0;
	float lambda, sigma0, sigma1, gamma;
	float pwr_v[SPACE_DIM], qwr_v[SPACE_DIM + 1], covResetRatio;
	int intModelType;
	float crFrequency;
	int ruptureWinSize;
	bool with3DdepthInfo;
	
	// Get Singleton Configuration object
	Configuration* config = Configuration::GetInstance("");

	// Get Singleton Configuration object
	Dataset* dataset = Dataset::GetInstance(0); //input number here has no relevance since the class is a Singleton

	// Load the Phantom pose, if requested, and set it in the simulator, if available
	if (config->useRegisteredPhantom()) {

		std::string regFileName = config->getRegTransfFile();
		this->phantom->loadTransformation(regFileName.c_str());
		Eigen::Matrix4d Trp = this->phantom->getTransformation();
		// Set the pose of the phantom in the simulator
		this->vrep->setObjectPose("_Phantom", Trp, "KukaOrigin", simx_opmode_blocking, this->simPort);
	}

	float curSimTransitionSample;
	this->vrep->getFloatSignal("true_transitions", curSimTransitionSample, simx_opmode_streaming, this->simPort);

	// Load standard deviations for the transition detection algorithm
	varGains = config->getRNIInitVariances();
	Psi0diag.setZero(varGains.size());
	for (int i = 0; i < varGains.size(); i++) {
		Psi0diag(i) = varGains[i];
	}
	theta0.setZero(varGains.size());

	// Initialize the RLS algorithm
	lambda = config->getRNIlambda();
	sigma0 = config->getRNISigma0();
	sigma1 = config->getRNISigma1();
	intModelType = config->getRNIinteractionModel();
	with3DdepthInfo = config->use3DLayerDepths();
	covResetRatio = config->getCovResetRatio();
	crFrequency = config->getRNICovResetFrequency();
	ruptureWinSize = config->getRNIRuptureWinSize();

	this->rlsAlgorithm.setCRratio(covResetRatio);
	this->rlsAlgorithm.setCRfrequency(config->getRNICovResetFrequency());
	this->rlsAlgorithm.initData(intModelType, lambda, Psi0diag, sigma0, sigma1);

	// Set the initial transformation matrix of the robot EE pose
	this->Tbee0 = this->robot->getTbee();

	// Draw line
	this->drawNeedleLine = false;

	// Set Twr
	this->vrep->getObjectPosition("KukaOrigin", pwr_v, "world", simx_opmode_blocking, this->simPort);
	this->vrep->getObjectQuaternion("KukaOrigin", qwr_v, "world", simx_opmode_blocking, this->simPort);
	std::memcpy(pwr.data(), pwr_v, sizeof(float)*SPACE_DIM);
	std::memcpy(qwr.coeffs().data(), qwr_v, sizeof(float)*(SPACE_DIM+1));
	this->Twr.topLeftCorner(SPACE_DIM, SPACE_DIM) = qwr.toRotationMatrix();
	this->Twr.block<3, 1>(0, 3) = pwr;

	this->resOffsetSet = false;
	this->posOffsetSet = false;
	this->firstItInLayer = true;
	this->z0 = 0.0;
	this->z_init = 0.0;
	this->res0.setZero(this->robot->getJointNum());
	this->f0.setZero();
	this->offsetSign = 1;
	this->fzHold = 0.0;

	this->offIdx_prev = 0;

	this->gtTransitionNum = -1;
	this->positionBasedLayerID = -1;
	this->layerDepthSet = false;
	this->insertionAccomplished = false;
	this->crossedLayersNum = 0;
	this->Bcomp = 0.0;
	this->BcompPrev = 0.0;
	this->Kcomp = 0.0;
	this->Bzsum.push_back(0.0);

	this->simTransition = 0;
	this->cusumTransition = false;
	this->cusumTransitionPrev = false;

	this->z_i_ = 0.0;
	this->z_i_prev = 0.0;
	this->depth_i_ = 0.0;
	this->fzPrev = 0.0;
	this->fzDot = 0.0;
	this->fzDotFilt = 0.0;
	this->fzFilt = 0.0;
	
	this->estimatedTransitionNum = 0;
	this->estimatedTransitionSig = 0;
	this->estimatedRuptureNum = 0;
	this->estimatedRuptureSig = false;

	this->task_idx = 0;
	this->rupture_idx = 0;
	this->setFzRuptureWinSize(ruptureWinSize);
	this->localVarianceWinSize = 25;

	this->localVarianceWindow.clear();
	this->localVarianceWindow.resize(this->localVarianceWinSize);
	this->fzFullBuffer.clear();
	this->ruptureIt_begin = this->fzFullBuffer.begin();
	this->ruptureIt_end = this->fzFullBuffer.begin();
	this->waitingForRuptureWindowEnd = false;
	this->layer_compensated = false;
	this->ignoreCUSUMdetection = false;
	this->first_detection = false;

	this->fzLayerBuffer.clear();
	this->vLayerBuffer.clear();
	this->phiLayerBuffer.clear();
	this->timeLayerBuffer.clear();
	this->fzLayerBuffer.push_back(0.0);
	this->vLayerBuffer.push_back(0.0);
	this->phiLayerBuffer.push_back(Eigen::VectorXf::Zero(varGains.size()));
	this->timeLayerBuffer.push_back(0.0);
	this->fzLayerBuffer_startIdx = 0;
	this->vLayerBuffer_startIdx = 0;
	this->phiLayerBuffer_startIdx = 0;
	this->timeLayerBuffer_startIdx = 0;
	this->fzLayerBuffer_endIdx = 0;
	this->vLayerBuffer_endIdx = 0;
	this->phiLayerBuffer_endIdx = 0;
	this->timeLayerBuffer_endIdx = 0;

	this->insertionState = INSERTION_STATE::UNDEFINED;
	this->max_force_val = 0.0;
	this->t_inv = 0.0;
	this->autonomousMotionInverted = false;
}


/**
* @brief Main function
* Implements the task of the RegistrationStrategy
* @return the return state of the routine
*/
int NeedleTissueIntEstStrategy::execTask() {
	// Define utility variables
	MultiLayerGelPhantom* mgphantom;
	int ret, jointNum, modelSize, zindex, intModelType;
	float pee_v[3], velNormThresh, covMatTrace, sigma0, sigma1, gamma_r, gamma_p, ni, gk_rupture, gk_rupture_arg, gk_puncture, gk_puncture_arg, err, fz_hat, fz_el, fz_vis, fz_comp, Klast, Blast;
	float curSimTransitionSample, z_i, depth_i, sumdepth, Bcomp_cur, z0, dt, fzLPFalpha, fzDotLPFalpha, fzDotRaw, lambda, needle_length;
	float* needleSize;
	bool detection, transition, isClutchButtonActive, layerDetectionEnabled, contactDetection, use3DModelDepths;
	std::vector <float> varGains;
	Eigen::VectorXf Psi0diag;
	Eigen::VectorXf phi_i, params, tau_c, tau_cFRI, qdot, theta0;
	Eigen::MatrixXf resGain;
	Eigen::Vector3f vbeet, pe0et, ve0et, pee;
	Eigen::Matrix3f Rbee, Rbee0, Re0et;
	Eigen::Vector6f ftmeasf, resF0ee, resFRIF0ee, sensorFe0et, resFriFee, resFee, gtFee;
	Eigen::Matrix4f Tbeet, Te0et;
	std::vector <float> sumDepths;
	std::string ftmeasSigNames[TWIST_DIM] = { "fee_x", "fee_y", "fee_z",
											  "tee_x", "tee_y", "tee_z", };

	// Get Singleton Configuration object
	Configuration* config = Configuration::GetInstance("");

	// Get Singleton SystemState object
	SystemState* sys = SystemState::GetInstance();

	// Get Singleton Dataset object
	Dataset* dataset = Dataset::GetInstance(0);


	/// +++ PRE-ESTIMATION PHASE 

	// Set local variables
	jointNum = this->robot->getJointNum();
	modelSize = this->rlsAlgorithm.getInputSize();
	velNormThresh = 1e-5;
	sigma0 = config->getRNISigma0();
	sigma1 = config->getRNISigma1();
	gamma_r = (sigma1 * sigma1 - sigma0 * sigma0) * 0.5;
	ni = (sigma1 * sigma1 + sigma0 * sigma0) * 0.5;
	sumdepth = 0.0;
	layerDetectionEnabled = false;
	tau_cFRI.setZero(jointNum);
	resFRIF0ee.setZero();
	phi_i.setZero(modelSize);
	mgphantom = nullptr;
	z_i = 0.0;
	depth_i = 0.0;
	Bcomp_cur = 0.0;
	curSimTransitionSample = 0;
	z0 = 1e3; // dummy high value
	use3DModelDepths = config->use3DLayerDepths();
	contactDetection = false;
	fzDotRaw = 0.0;
	Klast = 0.0;
	Blast = 0.0;
	needleSize = new float[SPACE_DIM];

	// Get Needle length
	this->robot->getEndEffectorFromName("Needle").getSize(needleSize);
	needle_length = needleSize[Z_AXIS];

	if (config->getPhantomType() == "Multilayer gel") {
		mgphantom = dynamic_cast<MultiLayerGelPhantom*>(this->phantom);
	}

	// 2. Get the flag about clutch signal (to deactivate transition detection during teleoperation)
	// TODO: This has been commented since I'm plannning to separate the functionalities of teleoperation (responsible of enabling/disabling the clutch)
	// and of the haptic feedback system (responsible of setting the force feedback). Therefore, two distinct classes should be considered in the future.
	// In the meanwhile, remember to uncomment these lines below if you want to run the task with the Geomagic device (that accomplishes both teleoperation and
	// force feedback features) and comment the line with single setForceFeedback() call (to be set in case of simulation with Armband).
	//isClutchButtonActive = ((!config->isProcessingOnline() && config->getControlFlag() == CONTROL::TELEOPERATION_MODE) ||
	//	(config->isProcessingOnline() && sys->getCtrlInfo().ctrlMode == CONTROL::TELEOPERATION_MODE))
	//	? (haptic->isClutchActive()) : (true);
	isClutchButtonActive = true;


	// 3. Get force measurement from the different sources (the selected one is taken later)
	// F/T sensor (if available)
	if (config->isForceSensorRequested()) {
		ftmeasf = ftsensor->getWrench().cast<float>();
	}

	// Residual-based contact torques and Cartesian froce
	tau_c = this->robot->getResidualVector();
	tau_c -= res0 * (this->insertionAccomplished ? -1.0 : 1.0);	// Compensation of the local friction
	resF0ee = -this->robot->staticForceReconstruction(tau_c);


	// KUKA residual-based contact torques (if available)
	if (config->getRobotManipulatorType() == "KUKA") {
		tau_cFRI = dynamic_cast<KUKARobot*>(this->robot)->getFRIResidual();
		resFRIF0ee = -this->robot->staticForceReconstruction(tau_cFRI);
	}

	// 4. Get Robot kinematics data to rotate estimation input data in the robot EE,0 frame
	// Robot pose
	Tbeet = this->robot->getTbee();

	// Robot EE velocity
	vbeet = (this->robot->getEEVelocity().topRows(SPACE_DIM));

	// Robot EE rotation matrix wrt base frame
	Rbee = Tbeet.topLeftCorner(SPACE_DIM, SPACE_DIM);

	// Compute the incremental transformation (ee,t wrt ee,0)
	Te0et = this->Tbee0.inverse() * Tbeet;
	Re0et = Te0et.topLeftCorner(SPACE_DIM, SPACE_DIM);

	// 4.1 EE,t position vector in the robot EE,0 frame
	pe0et = Te0et.block<3, 1>(0, 3);

	// 4.2 EE,t velocity vector in the robot EE,0 frame
	Rbee0 = this->Tbee0.topLeftCorner(SPACE_DIM, SPACE_DIM);
	ve0et = Rbee0.transpose() * vbeet;

	// 4.3 Rotate each force measurement in the robot EE,0 frame 
	// 4.3.1 F/T sensor measurement 
	sensorFe0et.topRows(SPACE_DIM) = Re0et * (ftmeasf.topRows(SPACE_DIM).cast<float>());
	sensorFe0et.bottomRows(SPACE_DIM) = Re0et * (ftmeasf.bottomRows(SPACE_DIM).cast<float>());

	// 4.3.2 residual-based force measurement 
	resFee.topRows(SPACE_DIM) = Rbee0.transpose() * resF0ee.topRows(SPACE_DIM);
	resFee.bottomRows(SPACE_DIM) = Rbee0.transpose() * resF0ee.bottomRows(SPACE_DIM);

	// 4.3.3 FRI KUKA residual-based force measurement 
	if (config->getRobotManipulatorType() == "KUKA") {
		resFriFee.topRows(SPACE_DIM) = Rbee0.transpose() * resFRIF0ee.topRows(SPACE_DIM);
		resFriFee.bottomRows(SPACE_DIM) = Rbee0.transpose() * resFRIF0ee.bottomRows(SPACE_DIM);
	}

	// 5. Select the appropriate force measurement based on user's choice and system settings
	float Tinv = 0.5;
	float ramp_scale = (time_ - this->t_inv >= Tinv) ? Tinv : time_ - this->t_inv;
	float sat_ramp =  1.0 - ramp_scale / Tinv * 2.0;
	resFee -= this->f0 * sat_ramp;


	if (config->getRNIgtForceData() == FT_SENSOR_SOURCE && config->isForceSensorRequested()) {
		gtFee = -sensorFe0et;
	}
	else if (config->getRNIgtForceData() == FRI_RESIDUAL_SOURCE) {
		gtFee = resFriFee;
	}
	else {
		gtFee = resFee;
	}

	// Evaluate variance of the force interval
	//float forceStdDev = this->evaluateRuptureVariance(this->task_idx, gtFee(Z_AXIS));
	float errStdDev = this->evaluateLocalVariance(this->task_idx, this->rlsAlgorithm.getLatestErrorSample());
	this->task_idx++;
	this->fzFullBuffer.push_back(gtFee(Z_AXIS));
	this->timeFullBuffer.push_back(time_);

	// Check for first contact detection from force derivative signal
	contactDetection = this->detectNeedleTissueContacts(gtFee(Z_AXIS), ve0et(Z_AXIS));

	// 6. Get the simulation transition signal to detect nominal transitions from 3D model reconstruction
	if (this->vrep->isAvailable()) { // If the simulator is available ...

		// Get the transition signal
		this->vrep->getFloatSignal("true_transitions", curSimTransitionSample, simx_opmode_buffer, this->simPort);

		// Check from configuration file if the knowledge of the layer depths must be taken into account for the interaction model
		// and the corresponding parameter identification. 
		// If nominal depths known from the target 3D model are used, the identification algorithm accounts them to scale the friction
		// force component to be estimated by employing the Gerovich model from the beginning. 
		// Otherwise, the Kelvin-Voigt model is used first until CUSUM + force derivative signal analysis shows detection of
		// contacts or ruptures. In that case, the depth of the layer is evaluated and the coefficients are re-estimated with 
		// the Gerovich model and the estimated depth
		if (use3DModelDepths) {

			// First, look for the first transition to initialize the position offset
			// On the raising edge of the transition signal, ...
			if (curSimTransitionSample == 1 && !this->simTransition) {


				// Increase the current layer index
				this->gtTransitionNum++;

				// If first hit, set z0
				if (this->gtTransitionNum == 0) {
					if (mgphantom != nullptr) {
						mgphantom->setZ0(pe0et(Z_AXIS)); // set z0
						this->f0 = resFee;
						this->cusumTransition = true;
					}

				}

			}
			// Update latest transition signal sample
			this->simTransition = curSimTransitionSample;

			if (mgphantom != nullptr) {

				int layerID = -1;
				if (this->gtTransitionNum >= 0) {

					z0 = mgphantom->getZ0();
					sumDepths = mgphantom->getCumulativeDepthsVector();

					for (int i = 0; i < mgphantom->getNumLayers(); i++) {

						if (pe0et(Z_AXIS) >= z0 + sumDepths[i] && pe0et(Z_AXIS) < z0 + sumDepths[i + 1]) {
							layerID = i;
						}
					}

					if (layerID >= 0) { // an index has been found and the needle is in one of the layers
						z_i = z0 + sumDepths[layerID];
						depth_i = mgphantom->getLayerInfo(layerID).getDepth();
					}
					else {
						layerID = mgphantom->getNumLayers();
						z_i = 0.0;
						depth_i = 0.0;
					}


					params = rlsAlgorithm.getCurrentEstParams();

					if (layerID > 0 && layerID != this->positionBasedLayerID && ve0et(Z_AXIS) > 0) {

						mgphantom->setLayerKcoeff(this->positionBasedLayerID, params(0)); 
						mgphantom->setLayerDcoeff(this->positionBasedLayerID, params(1)); 

						std::cout << "Layer # " << this->positionBasedLayerID << " data set: \n"
							<< "\t K coeff = " << mgphantom->getLayerInfo(this->positionBasedLayerID).getKcoeff() << std::endl
							<< "\t D coeff = " << mgphantom->getLayerInfo(this->positionBasedLayerID).getDcoeff() << std::endl << std::endl;

						this->Bcomp = 0.0;
						for (int i = 0; i <= this->positionBasedLayerID; i++) {
							if (config->getRNIinteractionModel() == KELVIN_VOIGT || config->getRNIinteractionModel() == NORMALIZED_GEROVICH || config->getRNIinteractionModel() == NORMALIZED_GEROVICH_WITH_CONST) {
								this->Bcomp += (mgphantom->getLayerInfo(i).getDcoeff() * mgphantom->getLayerInfo(i).getDepth()); // - for l-norm
							}
							else if (config->getRNIinteractionModel() == GEROVICH) {
								this->Bcomp += (mgphantom->getLayerInfo(i).getDcoeff() * mgphantom->getLayerInfo(i).getDepth());
							}
						}
						std::cout << "Cumulative B = " << this->Bcomp << std::endl;

						this->rlsAlgorithm.popLatestThetaSample();
						Eigen::VectorXf th0(modelSize);
						th0.setZero(modelSize);
						th0(2) = this->Bcomp;
						if (config->getRNIinteractionModel() == NORMALIZED_GEROVICH_WITH_CONST) {
							th0(2) = this->Bcomp;
						}
						std::cout << "th0 = " << th0.transpose() << std::endl;
						this->rlsAlgorithm.pushNewThetaSample(th0);

					}

					this->positionBasedLayerID = layerID; 			

				}
				else {
					z0 = 1e3; // dummy high value
				}
			}

		}
		else { // use3DModelDepths == false


			// This first if branch is required if you want to detect only the first contact
			// with the simulator. Otherwise, this must be commented and the one in the next if branch
			// decommented
			if (curSimTransitionSample == 1 && !this->simTransition) {
				this->gtTransitionNum++;
				this->z_i_ = pe0et(Z_AXIS);
				z0 = this->z_i_;
				mgphantom->setZ0(z0); // set z0
				this->f0 = resFee;
				this->simTransition = curSimTransitionSample;

				// SET RLS State on Running
				this->insertionState = INSERTION_STATE::NO_RUPTURE;
				std::cout << "Setting NO RUPTURE" << std::endl;
			}

			if (ve0et(Z_AXIS) > 0){
				if (this->insertionState == INSERTION_STATE::NO_RUPTURE) {
					int offIdx = dataset->getCurrentIndex();
					if (contactDetection) {
						this->insertionState = INSERTION_STATE::EXPECTED_RUPTURE;
						std::cout << "Setting EXPECTED_RUPTURE" << std::endl;
					}
					else if (this->cusumTransition) {
					//else if (offIdx == 2609 || offIdx == 3450 || offIdx == 3850){ // 3L+glove_sinvel
					//else if (offIdx == 4169 || offIdx == 6344){ // liver
						this->insertionState = INSERTION_STATE::DETECTED_RUPTURE;
						std::cout << "Setting DETECTED_RUPTURE" << std::endl;
					}
				}

				this->t_inv = time_;
			}
			else if(!this->autonomousMotionInverted) {
			
				this->t_inv = time_;
				this->autonomousMotionInverted = true;

			}
			if (this->insertionState == INSERTION_STATE::EXPECTED_RUPTURE) {
				//if (this->cusumTransition || this->estimatedTransitionTrail) { // <-- this for gel phantoms
				if (this->cusumTransition) { // <-- this for abdomen?
					int offIdx = dataset->getCurrentIndex();
				//if (offIdx == 2609 || offIdx == 3450 || offIdx == 3850) { // 3L+glove_sinvel
				//if (offIdx == 4169 || offIdx == 6344){ // liver
					float fdot_thresh = config->getForceDerContactThreshold();
					fdot_thresh += 0.025;
					config->setForceDerContactThreshold(fdot_thresh);
					this->insertionState = INSERTION_STATE::DETECTED_RUPTURE;
					std::cout << "Setting DETECTED_RUPTURE" << std::endl;
				}
			}
			if (this->insertionState == INSERTION_STATE::DETECTED_RUPTURE) {

				std::cout << "index of the dataset --> " << dataset->getCurrentIndex() << std::endl;
				std::cout << "New rupture detected. Number of ruptures = " << this->estimatedRuptureNum << std::endl;
				this->estimatedRuptureNum++;
				this->estimatedRuptureSig = true;
				this->first_detection = true;

				// ... Update depth_i
				depth_i = pe0et(Z_AXIS) - this->z_i_;

				// Get the latest estimated paramters
				params = this->rlsAlgorithm.getCurrentEstParams();
				Klast = params(0);
				Blast = params(1);
				mgphantom->addNewLayer(new TissueLayer(depth_i, Klast, Blast));
				std::cout << "depth = " << depth_i << std::endl;
				std::cout << "current estimated K = " << Klast << std::endl;
				std::cout << "current estimated B = " << Blast << std::endl;

				if (!config->rniUsePostCompensation()) {
				
					std::cout << "Post-compensation not requested. Here I should use the parameters found above... " << std::endl;
					if (config->getRNIinteractionModel() == KELVIN_VOIGT) {

						this->Bcomp = 0.0;
						std::cout << "After weighted re-estimation, the following parameters have been found:" << std::endl;
						std::cout << "Adopted Weighted Covariance Phi = " << Psi0diag.transpose() << std::endl;
						std::cout << "Rupture num: " << this->estimatedRuptureNum << std::endl;
						std::cout << "(Equivalently, phantom layers number = " << mgphantom->getNumLayers() << ") " << std::endl;
						float cumdepth = mgphantom->getCumulativeDepthsVector()[mgphantom->getCumulativeDepthsVector().size() - 1];
						for (int i = 0; i < mgphantom->getNumLayers(); i++) {
							float Bd_i = (mgphantom->getLayerInfo(i).getDcoeff());
							//float Bd_i = (mgphantom->getLayerInfo(i).getDcoeff() * mgphantom->getLayerInfo(i).getDepth());
							//float Bd_i = (mgphantom->getLayerInfo(i).getDcoeff()) * (0.153) / mgphantom->getLayerInfo(i).getDepth();
							//float Bd_i = (mgphantom->getLayerInfo(i).getDcoeff()) / mgphantom->getLayerInfo(i).getDepth();
							this->Bcomp += Bd_i;
							std::cout << "Cumulative depth = " << cumdepth << std::endl;
							std::cout << " --- Layer n. " << i + 1 << std::endl;
							std::cout << "\tLayer depth = " << mgphantom->getLayerInfo(i).getDepth() << std::endl;
							std::cout << "\tLayer K = " << mgphantom->getLayerInfo(i).getKcoeff() << std::endl;
							std::cout << "\tLayer B = " << mgphantom->getLayerInfo(i).getDcoeff() << std::endl;
							std::cout << "\tLayer B*d (damping) = " << Bd_i << std::endl;
							std::cout << "\t(temp) Sum_i(Bi*di) = " << this->Bcomp << std::endl;
						}

						this->rlsAlgorithm.popLatestThetaSample();
						Eigen::VectorXf th0(modelSize);
						th0.setZero(modelSize);
						th0 << 0.0, 0.0, this->Bcomp;
						if (config->getRNIinteractionModel() == NORMALIZED_GEROVICH_WITH_CONST) {
							th0 << 0.0, 0.0, this->Bcomp;
						}
						this->rlsAlgorithm.pushNewThetaSample(th0);
					}
					else if (config->getRNIinteractionModel() == NORMALIZED_GEROVICH_WITH_CONST) {

						// Recompute the compensating term
						this->Bcomp = 0.0;
						std::cout << "Rupture num: " << this->estimatedRuptureNum << std::endl;
						std::cout << "(Equivalently, phantom layers number = " << mgphantom->getNumLayers() << ") " << std::endl;
						for (int i = 0; i < mgphantom->getNumLayers(); i++) {
							float Bd_i = (mgphantom->getLayerInfo(i).getDcoeff() * mgphantom->getLayerInfo(i).getDepth());
							this->Bcomp += Bd_i;
							std::cout << " --- Layer n. " << i + 1 << std::endl;
							std::cout << "\tLayer depth = " << mgphantom->getLayerInfo(i).getDepth() << std::endl;
							std::cout << "\tLayer K = " << mgphantom->getLayerInfo(i).getKcoeff() << std::endl;
							std::cout << "\tLayer B = " << mgphantom->getLayerInfo(i).getDcoeff() << std::endl;
							std::cout << "\tLayer B*d (damping) = " << Bd_i << std::endl;
							std::cout << "\t(temp) Sum_i(Bi*di) = " << this->Bcomp << std::endl;
						}

						this->rlsAlgorithm.popLatestThetaSample();
						Eigen::VectorXf th0(modelSize);
						th0.setZero(modelSize);
						th0 << 0.0, 0.0, this->Bcomp;
						this->rlsAlgorithm.pushNewThetaSample(th0);
					}
					else {
						this->insertionState = INSERTION_STATE::FORCE_DROP_MEASURING;
					}	

					// Create the force interval to evaluate the force drop after rupture
					if (std::fabs(ve0et(Z_AXIS)) > 1e-6 && ve0et(Z_AXIS) > 0) {
						this->fzRuptureWinSize = 1.0 / ve0et(Z_AXIS);
					}
					else {
						this->fzRuptureWinSize = config->getRNIRuptureWinSize(); // default value
					}

					this->z_i_ = pe0et(Z_AXIS);
					this->rlsAlgorithm.resetCovMat();

					this->rupture_idx = this->task_idx;
					this->insertionState = INSERTION_STATE::COMPENSATED_FORCE;

				} 
				else {

					// Get current z value
					this->z_i_ = pe0et(Z_AXIS);
					// Reset Covariance matrix
					//this->rlsAlgorithm.resetCovMat();

					// Create the force interval to evaluate the force drop after rupture
					if (std::fabs(ve0et(Z_AXIS)) > 1e-6 && ve0et(Z_AXIS) > 0) {
						this->fzRuptureWinSize = 1.0 / ve0et(Z_AXIS);
					}
					else {
						this->fzRuptureWinSize = config->getRNIRuptureWinSize(); // default value
					}

					this->ruptureIt_begin = this->fzFullBuffer.end() - this->fzRuptureWinSize;
					this->waitingForRuptureWindowEnd = true;
					this->rupture_idx = this->task_idx;

					// Save the iterators of the layer buffers at the current position
					this->fzLayerBuffer_endIdx = this->fzLayerBuffer.size();
					this->vLayerBuffer_endIdx = this->vLayerBuffer.size();
					this->phiLayerBuffer_endIdx = this->phiLayerBuffer.size();
					this->timeLayerBuffer_endIdx = this->timeLayerBuffer.size();

					this->insertionState = INSERTION_STATE::FORCE_DROP_MEASURING;

				}

			}
			else { this->estimatedRuptureSig = false; }
			if (this->insertionState == INSERTION_STATE::FORCE_DROP_MEASURING) {
				if ((this->task_idx - this->rupture_idx) >= this->fzRuptureWinSize) {	
					this->ruptureIt_end = this->fzFullBuffer.end() - 1;
					this->ruptureIt_begin = this->fzFullBuffer.begin() + (this->rupture_idx - this->fzRuptureWinSize);
					std::vector<float>::iterator rupIt = this->ruptureIt_begin;
					std::vector<float>::iterator tminIt = std::min_element(rupIt + this->fzRuptureWinSize, this->ruptureIt_end);
					std::vector<float>::iterator tmaxIt = std::max_element(rupIt, this->ruptureIt_end);
					float fmin_ = *tminIt;
					float fmin = std::accumulate(rupIt + this->fzRuptureWinSize, this->ruptureIt_end, 0.0) / std::distance(rupIt + this->fzRuptureWinSize, this->ruptureIt_end);
					float fmax = *tmaxIt;
					fmin = fmin_;
					int tminIdx = std::distance(this->fzFullBuffer.begin(), tminIt);
					int tmaxIdx = std::distance(this->fzFullBuffer.begin(), tmaxIt);
					float tmin = this->timeFullBuffer[tminIdx];
					float tmax = this->timeFullBuffer[tmaxIdx];

					float deltaforce = fmax - fmin;
					float Kratio = deltaforce / (fmax - this->Bcomp * ve0et(Z_AXIS));
					float Bratio = 1.0 - Kratio;

					// ... Re-estimate the dynamic parameters of the previous layer
					float vavg = std::accumulate(this->vLayerBuffer.begin(), this->vLayerBuffer.end(), 0.0) / this->vLayerBuffer.size();

					// Get the average velocity value during insertion
					lambda = config->getRNIlambda();
					sigma0 = config->getRNISigma0();
					sigma1 = config->getRNISigma1();
					intModelType = config->getRNIinteractionModel();
					varGains = config->getRNIInitVariances();
					if (config->getRNIinteractionModel() == KELVIN_VOIGT) {
						float davg = mgphantom->getLayerInfo(mgphantom->getNumLayers() - 1).getDepth();
						davg *= 0.5;
						varGains[1] *=  (davg * davg) / (vavg * vavg) * (Bratio / Kratio);
					}
					else if (config->getRNIinteractionModel() == NORMALIZED_GEROVICH_WITH_CONST) {
						varGains[1] *= 1.0 / (vavg * vavg) * (Bratio / Kratio);
					}

					Psi0diag.setZero(varGains.size());
					for (int i = 0; i < varGains.size(); i++) {
						Psi0diag(i) = varGains[i];
					}

					this->layerRLS.push_back(new RLS);
					RLS* RLS_i = this->layerRLS.back();
					RLS_i->initData(intModelType, lambda, Psi0diag, sigma0, sigma1);
					theta0.setZero(3);
					theta0 << 0.0, 0.0, this->Bcomp;
					if (config->getRNIinteractionModel() == NORMALIZED_GEROVICH_WITH_CONST) {
						theta0.setZero(3);
						theta0 << 0.0, 0.0, this->Bcomp;
					}
					RLS_i->setInitialState(theta0);
					RLS_i->setCRfrequency(config->getRNICovResetFrequency());

					for (int j = this->fzLayerBuffer_startIdx; j < this->fzLayerBuffer_endIdx; j++) {
						float t_j = this->timeLayerBuffer[j];
						float fz_j = this->fzLayerBuffer[j];
						Eigen::VectorXf phi_j = this->phiLayerBuffer[j];
						RLS_i->run(t_j,fz_j,phi_j);
					}

					std::cout << "New parameter estimated after rupture detection and re-estimation: " << std::endl;
					std::cout << "K_new = " << RLS_i->getCurrentEstParams()(0) << std::endl;
					std::cout << "B_new = " << RLS_i->getCurrentEstParams()(1) << std::endl;
					std::cout << std::endl;

					this->fzLayerBuffer_startIdx = this->fzLayerBuffer_endIdx;
					this->vLayerBuffer_startIdx = this->vLayerBuffer_endIdx;
					this->phiLayerBuffer_startIdx = this->phiLayerBuffer_endIdx;
					this->timeLayerBuffer_startIdx = this->timeLayerBuffer_endIdx;

					// Reset Covariance matrix

					float Knew_2 = RLS_i->getCurrentEstParams()(0);
					float Bnew_2 = RLS_i->getCurrentEstParams()(1);

					// Update the coefficients in the layer phantom
					mgphantom->setLayerKcoeff(mgphantom->getNumLayers() - 1, Knew_2);
					mgphantom->setLayerDcoeff(mgphantom->getNumLayers() - 1, Bnew_2);

					// Recompute the compensating term
					this->Bcomp = 0.0;
					std::cout << "After weighted re-estimation, the following parameters have been found:" << std::endl;
					std::cout << "Adopted Weighted Covariance Phi = " << Psi0diag.transpose() << std::endl;
					std::cout << "Rupture num: " << this->estimatedRuptureNum << std::endl;
					std::cout << "(Equivalently, phantom layers number = " << mgphantom->getNumLayers() << ") " << std::endl;
					for (int i = 0; i < mgphantom->getNumLayers(); i++) {
						float Bd_i;
						if (config->getRNIinteractionModel() == KELVIN_VOIGT) {
							Bd_i = (mgphantom->getLayerInfo(i).getDcoeff());
							Bd_i = (Bd_i < 0) ? 0.0 : Bd_i;
						}
						else if (config->getRNIinteractionModel() == NORMALIZED_GEROVICH_WITH_CONST) {
							Bd_i = (mgphantom->getLayerInfo(i).getDcoeff() * mgphantom->getLayerInfo(i).getDepth());
						}
						this->Bcomp += Bd_i ;
						std::cout << " --- Layer n. " << i + 1 << std::endl;
						std::cout << "\tLayer depth = " << mgphantom->getLayerInfo(i).getDepth() << std::endl;
						std::cout << "\tLayer K = " << mgphantom->getLayerInfo(i).getKcoeff() << std::endl;
						std::cout << "\tLayer B = " << mgphantom->getLayerInfo(i).getDcoeff() << std::endl;
						std::cout << "\tLayer B*d (damping) = " << Bd_i << std::endl;
						std::cout << "\t(temp) Sum_i(Bi*di) = " << this->Bcomp << std::endl;
					}

					this->rlsAlgorithm.popLatestThetaSample();
					Eigen::VectorXf th0(modelSize);
					th0.setZero(modelSize);
					th0 << 0.0, 0.0, this->Bcomp;
					if (config->getRNIinteractionModel() == NORMALIZED_GEROVICH_WITH_CONST) {
						th0 << 0.0, 0.0, this->Bcomp;
					}
					this->rlsAlgorithm.pushNewThetaSample(th0);


					this->waitingForRuptureWindowEnd = false;
					this->rupture_idx = this->task_idx;

					this->insertionState = INSERTION_STATE::COMPENSATED_FORCE;

				}
			}
			if (this->insertionState == INSERTION_STATE::COMPENSATED_FORCE) {
				if ((this->task_idx - this->rupture_idx) >= this->fzRuptureWinSize / 2) {
					this->insertionState = INSERTION_STATE::NO_RUPTURE;
				}
			}

			// Set on the local variable
			z_i = this->z_i_;
			z0 = (this->gtTransitionNum >= 0) ? (mgphantom->getZ0()) : (1e3);
			depth_i = mgphantom->getCumulativeDepthsVector()[mgphantom->getNumLayers()];

		}

	}


	// 8. Update the RLS input vector phi based on the current position of the needle tip
	// At this point of the algorithm, we should know some z_i:
	// - If the 3D model is used, it should be read as the sum of depths of the previous layers
	// - If the 3D model is not used, it comes from the latest contact detection (see code above - TODO)
	if (pe0et(Z_AXIS) < z0) {
		phi_i.setZero(modelSize);
	}
	else {
		if ((use3DModelDepths && z_i != 0.0) || !use3DModelDepths) {
			phi_i(0) = (pe0et(Z_AXIS) - z_i);
			if (config->getRNIinteractionModel() == KELVIN_VOIGT) {
				phi_i(1) = ve0et(Z_AXIS); 
				phi_i(2) = ve0et(Z_AXIS); 

			}
			else if (config->getRNIinteractionModel() == GEROVICH) {
				phi_i(1) = (pe0et(Z_AXIS) - z_i) * ve0et(Z_AXIS); // Gerovich
			}
			else if (config->getRNIinteractionModel() == NORMALIZED_GEROVICH || config->getRNIinteractionModel() == NORMALIZED_GEROVICH_WITH_CONST) {

				phi_i(1) = (pe0et(Z_AXIS) - z_i) * ve0et(Z_AXIS); 

				if (config->getRNIinteractionModel() == NORMALIZED_GEROVICH_WITH_CONST) {
					phi_i(2) = ve0et(Z_AXIS) ; 
				}
			}

		}
	}


	fz_comp = gtFee(2) - this->Bcomp * ve0et(2); 
	this->rlsAlgorithm.pushCompensatedForceSample(fz_comp);

	/// ESTIMATION PHASE
	this->rlsAlgorithm.run(time_, gtFee(2), phi_i);

	// Store on buffer
	this->fzLayerBuffer.push_back(gtFee(Z_AXIS));
	this->phiLayerBuffer.push_back(phi_i);
	this->vLayerBuffer.push_back(ve0et(Z_AXIS));
	this->timeLayerBuffer.push_back(time_);

	// Run the current iteration of the RLS algorithm
	if (isClutchButtonActive) {
		this->rlsAlgorithm.pushGTTransitionSample(curSimTransitionSample);
	}

	// Get the current estimation of the contact force
	fz_hat = rlsAlgorithm.getCurrentRecFz();

	// Get the current error of the force reconstruction
	err = rlsAlgorithm.getLatestErrorSample();

	// Run the layer detection algorithm if conditions are met
	layerDetectionEnabled = ((config->getRNIgtForceData() == MODEL_RESIDUAL_SOURCE) && this->gtTransitionNum >= 0 ||
		(config->getRNIgtForceData() != MODEL_RESIDUAL_SOURCE));

	layerDetectionEnabled = layerDetectionEnabled && 
		(this->insertionState != INSERTION_STATE::FORCE_DROP_MEASURING && 
		 this->insertionState != INSERTION_STATE::COMPENSATED_FORCE);
	
	this->cusumTransitionPrev = this->cusumTransition;
	if (layerDetectionEnabled) {
		transition = this->rlsAlgorithm.cusumRuptureDetection(err, fz_hat);
	}
	else {
		transition = false;
	}

	transition = transition || (!this->first_detection && this->estimatedTransitionTrail);
	if (this->estimatedTransitionTrail) { this->first_detection = false; }

	this->rlsAlgorithm.pushTransitionDetectionSample(this->estimatedRuptureSig);
	this->cusumTransition = transition;




	
	fz_el = rlsAlgorithm.getLatestElasticForceSample();
	fz_vis = rlsAlgorithm.getLatestViscousForceSample();

	/// +++ POST-ESTIMATION PHASE 
	// Get latest estimated parameters Khat and Bhat
	params.setZero(phi_i.size());
	if (pe0et(2) > z_init) {
		params = rlsAlgorithm.getCurrentEstParams();
	}

	// Get the latest covariance matrix trace value
	covMatTrace = rlsAlgorithm.getLatestCovMat().trace();

	// Get the current value of the transitionDetection signal
	gk_rupture = rlsAlgorithm.getLatestGRuptureFcnSample();
	gk_rupture_arg = rlsAlgorithm.getLatestGRuptureArg();
	gk_puncture = rlsAlgorithm.getLatestGPunctureFcnSample();
	gk_puncture_arg = rlsAlgorithm.getLatestGPunctureArg();
	detection = this->rlsAlgorithm.getLatestTransitionDetectionSample();


	// If the simulator is available, send all data to be monitored online in live graphs
	if (this->vrep->isAvailable()) {

		// Residual-based contact torques
		for (int i = 0; i < jointNum; i++) {
			std::string modResSigName_i = std::string("modRes_q") + std::to_string(i + 1);
			this->vrep->setFloatSignal(modResSigName_i.c_str(), tau_c(i), simx_opmode_oneshot, this->simPort);
		}

		// F/T Sensor measurements
		for (int i = 0; i < SPACE_DIM * 2; i++) {
			this->vrep->setFloatSignal(ftmeasSigNames[i].c_str(), (float)ftmeasf(i), simx_opmode_oneshot, this->simPort);
		}
		
		gamma_p = 0.0;
		this->vrep->setFloatSignal("fz_rls", fz_hat, simx_opmode_oneshot, this->simPort); // RLS Reconstructed force
		this->vrep->setFloatSignal("fz_el", fz_el, simx_opmode_oneshot, this->simPort); // RLS Elastic contribution from reconstructed force
		this->vrep->setFloatSignal("fz_comp", fz_comp, simx_opmode_oneshot, this->simPort); // RLS Elastic contribution from reconstructed force
		this->vrep->setFloatSignal("fz_vis", fz_vis, simx_opmode_oneshot, this->simPort); // RLS Viscous contribution from reconstructed force
		this->vrep->setFloatSignal("fz_cumvis", this->Bcomp * ve0et(Z_AXIS), simx_opmode_oneshot, this->simPort); // RLS cumulative Viscous contribution from reconstructed force
		this->vrep->setFloatSignal("fz_res", resFee(Z_AXIS), simx_opmode_oneshot, this->simPort); // Residual-based Cartesian contact force
		this->vrep->setFloatSignal("fz_fri", resFriFee(Z_AXIS), simx_opmode_oneshot, this->simPort); // FRI Residual-based Cartesian contact force (only for KUKA robot. TODO: Add a check on the robot type here)
		this->vrep->setFloatSignal("fz_sensor", -sensorFe0et(Z_AXIS), simx_opmode_oneshot, this->simPort); // F/T Sensor measurement along z-axis in the robot EE,0 frame
		this->vrep->setFloatSignal("K_hat", params(0), simx_opmode_oneshot, this->simPort); // RLS estimation of the elastic coefficient K
		this->vrep->setFloatSignal("D_hat", params(1), simx_opmode_oneshot, this->simPort); // RLS estimation of the viscous coefficient D
		this->vrep->setFloatSignal("NL_hat", params(2), simx_opmode_oneshot, this->simPort);
		this->vrep->setFloatSignal("forceDer", this->fzDotFilt, simx_opmode_oneshot, this->simPort); // RLS estimation error
		this->vrep->setFloatSignal("errStdDev", errStdDev, simx_opmode_oneshot, this->simPort); // RLS estimation error
		this->vrep->setFloatSignal("RLS_err", err*err, simx_opmode_oneshot, this->simPort); // RLS estimation error
		this->vrep->setFloatSignal("pz", phi_i(0), simx_opmode_oneshot, this->simPort); // RLS input contribution of the elastic force component
		this->vrep->setFloatSignal("vz", phi_i(1), simx_opmode_oneshot, this->simPort); // RLS input contribution of the viscous force component
		this->vrep->setFloatSignal("covMatTrace", covMatTrace, simx_opmode_oneshot, this->simPort);  // RLS covariance matrix trace
		this->vrep->setFloatSignal("gRupture", gk_rupture, simx_opmode_oneshot, this->simPort);  // CUSUM decision function argument
		this->vrep->setFloatSignal("gRupture_arg", gk_rupture_arg, simx_opmode_oneshot, this->simPort); // CUSUM g function argument
		this->vrep->setFloatSignal("gPuncture", gk_puncture, simx_opmode_oneshot, this->simPort);  // CUSUM decision function argument
		this->vrep->setFloatSignal("gPuncture_arg", gk_puncture_arg, simx_opmode_oneshot, this->simPort); // CUSUM g function argument
		this->vrep->setFloatSignal("rlsGamma_rupture", gamma_r, simx_opmode_oneshot, this->simPort);  // CUSUM gamma value 
		this->vrep->setFloatSignal("rlsGamma_puncture", gamma_p, simx_opmode_oneshot, this->simPort);  // CUSUM gamma value 
		if (detection) {
			this->vrep->setFloatSignal("rlsTransition", detection, simx_opmode_blocking, this->simPort); // CUSUM detection signal
		}
		else {
			this->vrep->setFloatSignal("rlsTransition", detection, simx_opmode_oneshot, this->simPort); // CUSUM detection signal
		}
		this->vrep->setFloatSignal("contact", contactDetection, simx_opmode_oneshot, this->simPort); // CUSUM detection signal
		this->vrep->setFloatSignal("DZF_delta", this->rlsAlgorithm.getDeadZoneFcnDelta(), simx_opmode_oneshot, this->simPort); // RLS dead-zone function value
		this->vrep->setFloatSignal("clutch", float(isClutchButtonActive) * 0.5, simx_opmode_oneshot, this->simPort); // clutch signal
	}


	// Check for visualization of the needle direction in the simulator
	if (sys->getShow() == ACTION::REQUESTED) {
		this->showNeedleDirection();
		sys->setShow(ACTION::CLEARED);
	}

	// Check for termination request and set flag
	if (sys->getAction() == ACTION::REQUESTED) {

		staticPhantom.reset();

		ret = TASK_RETURN_CODE::FINISHED;
		sys->setAction(ACTION::CLEARED);

	}
	else {

		ret = TASK_RETURN_CODE::PROCESSING;
	}


	return ret;

}


/**
* @brief Update function
* Compute the variance of the input signal over the specified window with size localVarianceWindow 
* @param idx: the current index
* @param fz: the current input signal sample
* @return stddev: the output local standard deviation of the input signal
*/
float NeedleTissueIntEstStrategy::evaluateLocalVariance(const int& idx, const float& in) {

	int cursize = this->localVarianceWindow.size();
	int circular_idx = idx % cursize;
	this->localVarianceWindow[circular_idx] = in;

	// Compute the mean in the interval
	float mean = 0.0;
	for (int i = 0; i < cursize; i++) {
		mean += this->localVarianceWindow[i];
	}
	mean /= cursize;

	// Compute the variance
	float stddev = 00.0;
	for (int i = 0; i < cursize; i++) {
		stddev += pow(this->localVarianceWindow[i] - mean,2);
	}
	stddev = sqrt(stddev / cursize);
	//stddev = (stddev / cursize);

	return stddev;
}


/**
* @brief Estimation function
* Compute a batch estimation of the previous layer coefficients based on the input buffers
* @param fzBuff: the buffer of the force measurement
* @param phiBuff: the buffer of the input vector phi
* @param old_depth: the previous depth considered for the layer
* @param new_depth: the previous depth evaluated for the layer
* @return the vector of estimated coefficients
*/
Eigen::VectorXf NeedleTissueIntEstStrategy::layerBatchEstimation(const std::vector <float>& fzBuff, const std::vector <Eigen::VectorXf>& phiBuff, const float& old_depth, const float& new_depth) {

	Eigen::VectorXf KB;
	Eigen::VectorXf F;
	Eigen::MatrixXf PHI, PHIT, PHIpinv;
	int nsize, fsize;

	nsize = this->rlsAlgorithm.getInputSize();
	fsize = fzBuff.size();
	KB.setZero(nsize);
	F.setZero(fsize);
	PHI.setZero(fsize, nsize);
	PHIT.setZero(nsize, fsize);

	for (int i = 0; i < fsize; i++) {
		F(i) = fzBuff[i];
		PHI.row(i) = phiBuff[i];
		PHI(i, 1) = PHI(i, 1) * PHI(i, 0) / new_depth;
	}

	// Compute the pseudoinverse
	PHIT = PHI.transpose();
	PHIpinv = (PHIT * PHI).inverse() * PHIT;

	KB = PHIpinv * F;

	return KB;
}


/**
* @brief Detection function
* Detect detection of contacts between needle and tissues, based on the analysis of the force derivative signal
* @param fraw: the raw force measurement
* @param v: the insertion velocity 
* @return the boolean signal of the contact event
*/
bool NeedleTissueIntEstStrategy::detectNeedleTissueContacts(const float& fraw, const float& v) {

	bool contactDetection;
	float input, fzDotRaw, fzLPFalpha, fzDotLPFalpha, dt, contactThreshold;

	// Initialize the return value
	contactDetection = false;

	// Get singleton instance of Configuration object
	Configuration* config = Configuration::GetInstance("");

	// Get configuration parameters
	fzLPFalpha = config->getForceLowpassFilterCoeff();
	fzDotLPFalpha = config->getForceDerLowpassFilterCoeff();
	contactThreshold = config->getForceDerContactThreshold();
	dt = robot->getSampleTime();

	// Low-pass filter the input raw force measurement
	//input = (std::fabs(v) > 1e-6) ? (fraw / v) : 0.0;
	input = fraw;
	//std::cout << "input = " << input << std::endl;

	this->fzFilt = fzLPFalpha * input + (1.0 - fzLPFalpha) * this->fzPrev;

	// Compute the raw measurement of the force derivative signal
	fzDotRaw = (this->fzFilt - this->fzPrev) / dt;
	//fzDotRaw = (fraw - this->fzPrev) / dt;

	// Low-pass filter the input raw force derivative measurement
	this->fzDotFilt = fzDotLPFalpha * fzDotRaw + (1.0 - fzDotLPFalpha) * this->fzDotFilt;

	// Update the filtered force measurement
	this->fzPrev = this->fzFilt;
	//this->fzPrev = fraw;

	// Evaluate the contact event based on the force derivative threshold
	contactDetection = this->fzDotFilt > contactThreshold;

	if (!this->estimatedTransitionSig) {
		contactDetection = this->fzDotFilt > contactThreshold;
	}
	else{
		contactDetection = this->fzDotFilt > -3e-2; contactThreshold * 0.0; 0.5;
	}

	// Increase on the raising edge
	if (contactDetection && !this->estimatedTransitionSig) {
		this->estimatedTransitionNum++;
		this->estimatedTransitionRaise = true;
	}
	else {
		this->estimatedTransitionRaise = false;
	}

	// Detect trailing edge
	if (!contactDetection && this->estimatedTransitionSig) {
		this->estimatedTransitionTrail = true;
	}
	else {
		this->estimatedTransitionTrail = false;
	}

	// Set in the RLS object
	this->rlsAlgorithm.pushContactDetectionSample(contactDetection);

	// Returb the boolean signal
	this->estimatedTransitionSig = contactDetection;
	return contactDetection;
	//return this->estimatedTransitionRaise;

}


/**
* @brief Terminate function
* Terminate the NeedleTissueIntEstStrategy class
*/
void NeedleTissueIntEstStrategy::terminate() {

	// Get Singleton SystemState object
	SystemState* sys = SystemState::GetInstance();

	// Get Singleton Configuration object
	Configuration* config = Configuration::GetInstance("");

	// Get MultiLayerGelPhantom
	MultiLayerGelPhantom* mgphantom = dynamic_cast<MultiLayerGelPhantom*>(this->phantom);

	// Save stats
	if (config->isLogEnabled()) {
	
		std::string logPath = sys->getLogPath() + "\\RLS";
		std::cout << "logPath = " << logPath << std::endl;
		rlsAlgorithm.saveStats(logPath.c_str());

		if (config->rniUsePostCompensation()) {

			// Save stats for all the layer RLS instances
			for (int i = 0; i < this->layerRLS.size(); i++) {

				std::string logPath = sys->getLogPath() + "\\RLS_" + std::to_string(i + 1);
				if (!CreateDirectory(logPath.c_str(), NULL)) {
					std::cout << "Error: impossible to create folder at path: " << logPath << std::endl;
				}

				std::cout << "logPath = " << logPath << std::endl;
				std::cout << "numLayers = " << mgphantom->getNumLayers() << std::endl;
				this->layerRLS[i]->saveStats(logPath.c_str());
			}

		}

	}

	for (int i = 0; i < mgphantom->getNumLayers(); i++) {
		// Delete dynamic instance
		//delete this->layerRLS[i];
	}

	// Print out the final robot configuration
	std::cout << "The robot is stopping in the following configuration: \n" << this->robot->getMsrJointPosition().transpose() * 180.0 / M_PI << std::endl;


}


/**
* @brief Visualization function
* Request to the VREP simulator to render the line of the needle direction in the virtual scene
*/
void NeedleTissueIntEstStrategy::showNeedleDirection() {

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

