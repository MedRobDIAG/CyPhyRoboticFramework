// Project Header files
#include "RLS.hpp"
#include "DBWrapper.hpp"

// Windows Header files
#include <Windows.h>

// Standard Header files
#include <iostream>

/**
* @brief Init function
* Initiialize the data of the RLS algorithm
* @param interactionType_: an integer specifying the adopted interaction model
*/
void RLS::initData(const int& interactionType_, const float& lambda_, const Eigen::VectorXf& Psi0diag, const float& sig0, const float& sig1){

	// Set the interaction model type
	this->interactionModelType = interactionType_;

	// Set lambda
	this->lambda = lambda_;

	//this->inputSize = 2;
	this->inputSize = 3;
	if (this->interactionModelType == NORMALIZED_GEROVICH_WITH_CONST) {
		this->inputSize = 3;
	}

	// Init the theta and Psi variables
	Eigen::VectorXf theta0(this->inputSize);
	theta0.setZero(this->inputSize);
	this->setPsi0(Psi0diag.block(0,0,this->inputSize,1));

	std::cout << "Psi0 =\n " << Psi0 << std::endl;

	// Init the Covariance Reset condition threshold
	this->covResetThreshold = Psi0diag.sum() * this->crRatio; 

	// Init the decision variables
	this->setDecisionSigmas(sig0, sig1);

	// Initialize the number of transitions to zero
	this->transitionNum = 0;
	this->ruptureEvent = false;
	this->punctureEvent = false;

	// Push back initial data on the corresponding containers
	this->setInitialState(theta0);
	this->covMat.push_back(this->Psi0);
	this->recFz.push_back(0.0);
	this->gtFz.push_back(0.0);
	this->err.push_back(0.0);
	this->layerErr.push_back(0.0);
	this->d_f.push_back(0.0);
	this->timestamps.push_back(0.0);
	this->gRuptureFcn.push_back(0.0);
	this->gRuptureArg.push_back(0.0);
	this->gPunctureFcn.push_back(0.0);
	this->gPunctureArg.push_back(0.0);
	this->transitionDetection.push_back(false);
	this->phi.push_back(Eigen::VectorXf::Zero(this->inputSize));
	this->fEl.push_back(0.0);
	this->fVis.push_back(0.0);
	this->compFz.push_back(0.0);

	// Initialize stringstream object
	recFzSS.str("");							//!< Stringstream object of the reconstructed force
	gtFzSS.str("");								//!< Stringstream object of the ground truth force
	thetaSS.str("");							//!< Stringstream object of the theta vector
	phiSS.str("");								//!< Stringstream object of the phi vector
	covMatSS.str("");							//!< Stringstream object of the covariance Psi matrix
	detTransSS.str("");							//!< Stringstream object of the detected transitions
	detContactSS.str("");						//!< Stringstream object of the detected contacts
	gtTransSS.str("");							//!< Stringstream object of the ground truth transitions
	errSS.str("");								//!< Stringstream object of the force error
	derrSS.str("");								//!< Stringstream object of the force error derivative
	dfSS.str("");								//!< Stringstream object of the reconstructed force derivative
	fcompSS.str("");							//!< Stringstream object of the compensated force


	this->S = 0.0;
	this->S2 = 0.0;
	this->S11 = 0.0;
	this->fzMean = 0.0;
	this->fzStdDev = 0.0;

	this->errSum = 0.0;
	this->errSum2 = 0.0;
	this->errMean = 0.0;
	this->errStdDev = 0.0;

	this->estimateK = true;
	this->stdvarWinSize = 200;
	this->treset = 0.0;
	this->ttrue = clock.getCurTime();
}

/**
* @brief Load function
* Load the datasets from previous sessions to run the algorithm offline
* @param logNum: the number identifying the previous log session
* @param [out] q: the dataset of joint position
* @param [out] qd: the dataset of joint velocity
* @param [out] r: the dataset of residuals
* @param [out] tau: the dataset of torques
* @param [out] f: the dataset of forces
*/
void RLS::loadData(const int& logNum, std::vector<Eigen::VectorXf>& q,
									  std::vector<Eigen::VectorXf>& qd,
									  std::vector<Eigen::VectorXf>& r,
									  std::vector<Eigen::VectorXf>& tau,
									  std::vector<Eigen::VectorXf>& f){


	std::string logPath, filePath;
	std::vector < std::string > qcontent, qdcontent, rcontent, taucontent, fcontent;
	std::string qFile("qmsr.txt");
	std::string qdFile("qdotcmd.txt");
	std::string rFile("modelResidual.txt");
	std::string tauFile("taumsr.txt");
	std::string fFile("ftmeas.txt");
	DBWrapper db;
	char buf[256];

	// Get the current program directory
	GetCurrentDirectoryA(256, buf);
	logPath = std::string(buf) + "\\LogSession_" + std::string(std::to_string(logNum));

	// Joint positionss
	filePath = logPath + "\\KUKARobot\\" + qFile;
	q = db.loadCSVLog(filePath);

	// Joint velocities
	filePath = logPath + "\\KUKARobot\\" + qdFile;
	qd = db.loadCSVLog(filePath);

	// Residuals
	filePath = logPath + "\\KUKARobot\\" + rFile;
	r = db.loadCSVLog(filePath);

	// Joint Torques
	filePath = logPath + "\\KUKARobot\\" + tauFile;
	tau = db.loadCSVLog(filePath);

	// Force
	filePath = logPath + "\\FTSensor\\" + fFile;
	f = db.loadCSVLog(filePath);

}

/**
* @brief Push function
* Push the new acquired sample data in the corresponding buffers
* @param t: the timestamp
* @param gtfz: the ground truth force data on z-axis
* @param phi: the phi input vector
* @param runningCondition: true if the conditions to run the current iteration of the algorithm are satisfied. 
*        If so, do not forward recFz, theta and Psi former data in the buffer
*/
void RLS::pushData(const double& t, const float& gtfz, const Eigen::VectorXf& phi, const bool& runningCondition) {

	int N = this->inputSize;

	// Push the timestamp in the corresponding container
	this->timestamps.push_back(t);

	// Push the input ground truth force data on the corresponding vector
	this->gtFz.push_back(gtfz);
	std::vector<float>::const_iterator first = (this->gtFz.size() < this->stdvarWinSize) ? (this->gtFz.begin()) : (this->gtFz.end() - this->stdvarWinSize);
	std::vector<float>::const_iterator last = this->gtFz.end();
	std::vector<float> fzVarBuffer(first,last);
	
	Eigen::VectorXd fzVarBufferArr(fzVarBuffer.size());

	for (int i = 0; i < fzVarBuffer.size(); i++) {
		fzVarBufferArr[i] = fzVarBuffer[i];
	}
	this->fzStdDev = std::sqrt((fzVarBufferArr.array() - fzVarBufferArr.array().mean()).square().sum() / (fzVarBufferArr.array().size() - 1));

	// Push the input phi data on the corresponding vector
	this->phi.push_back(phi);

	if (!runningCondition) { // If the algorithm is not supposed to be run in this iteration, forward former data of recFz, theta and Psi data in their buffer
	
		Eigen::VectorXf theta_i = this->theta.back();
		Eigen::MatrixXf Psi_i = this->covMat.back();
		float recFz_i = this->recFz.back();
		float err_i = this->err.back();

		// Set the next samples of theta and Psi in the corresponding containers
		this->theta.push_back(theta_i);
		this->covMat.push_back(Psi_i);
		this->recFz.push_back(recFz_i);
		this->err.push_back(err_i);
		this->transitionDetection.push_back(false);

	}

}


/**
* @brief Initialization function
* Compute the exact initialization of the RLS algorithm with a set of m initial measurements
* @param fBuff: the buffer of the first m force measurements
* @param phiBuff: the buffer of the fir
*/
void RLS::exactInitialization(const std::vector<float>& fBuff, const std::vector <Eigen::VectorXf>& phiBuff) {



	// Initialize the covariance matrix

	// initialize the theta vector

}



/**
* @brief Run function
* Run the i-th iteration of the RLS algorithm
* @param t: the i-th timestamp
* @param gtFz_i: the i-th ground truth force data sample
* @param phi_i: the i-th input data sample
*/
void RLS::run(const double& t, const float& gtFz_i, const Eigen::VectorXf& phi_i) {

	// Extract the size of the problem
	int N;
	Eigen::VectorXf theta_i;
	Eigen::VectorXf phi_iT;
	Eigen::VectorXf theta_prev;
	Eigen::MatrixXf Psi_i, Psi_prev;
	float recFz_i, fEl_i, fVis_i;
	float err_i;
	bool detectTransition = false;

	// Get the input size and initialize local data
	N = this->inputSize;
	theta_i.setZero(N);
	phi_iT.setZero(N);
	theta_prev.setZero(N);
	Psi_i.setZero(N, N);
	Psi_prev.setZero(N, N);

	// Push the input data in the corresponding container
	this->pushData(t, gtFz_i, phi_i);

	// Compute the transpose of the phi vector
	phi_iT = phi_i.transpose();

	// Get the i-1-th theta vector
	theta_prev = this->theta.back();

	// Get the i-1-th covariance Psi matrix
	Psi_prev = this->covMat.back();

	// Compute the latest reconstructed force estimation
	recFz_i = phi_i.dot(theta_prev);
	fEl_i = phi_i(0) * theta_prev(0);
	fVis_i = phi_i(1) * theta_prev(1);
	//fVis_i = phi_i(1) * theta_prev(1) + phi_i(2) * theta_prev(2);

	// Evaluate the current error
	err_i = gtFz_i - recFz_i;

	// Compute statistical properties of the error
	this->layerErr.push_back(err_i);
	int errSamples = this->layerErr.size();
	this->errSum += err_i;
	this->errSum2 += err_i * err_i;
	this->errStdDev = (this->errSum2 - this->errSum * this->errSum/ ((float)errSamples) ) / ((float)errSamples);
	this->errStdDev = sqrt(this->errStdDev);

	// Update the estimation of the theta vector for the next iteration
	float lambda_i = 1.0 + phi_i.norm() * this->lambda;
	float N0 = 1e-2;
	float abserr = std::fabs(err_i);
	this->delta_dzf = ((abserr >= 2.0 * N0 ? (1.0) : ((abserr >= N0 && abserr < 2.0 * N0) ? (abserr / N0 - 1.0) : (0.0)))); // dead-zone function
	//theta_i = theta_prev + (Psi_prev * phi_i * err_i) / (lambda_k + phi_i.transpose() * Psi_prev * phi_i);
	theta_i = theta_prev + (Psi_prev * phi_i * err_i) / (this->lambda + phi_i.transpose() * Psi_prev * phi_i);

	// Update the estimation of the covariance matrix Psi for the next iteration
	//Psi_i = Psi_prev - (Psi_prev * phi_i * phi_i.transpose() * Psi_prev) / (lambda_k + phi_i.transpose() * Psi_prev * phi_i);
	Psi_i = Psi_prev - (Psi_prev * phi_i * phi_i.transpose() * Psi_prev) / (this->lambda + phi_i.transpose() * Psi_prev * phi_i);
	//Psi_i *= this->delta_dzf;

	/*std::cout << "err = " << err_i << std::endl;
	std::cout << "gtFz_i = " << gtFz_i << std::endl;
	std::cout << "recFz_i = " << recFz_i << std::endl;
	std::cout << "Phi = \n" << Psi_prev << std::endl;
	std::cout << "Psi update = \n " << (Psi_prev * phi_i * phi_i.transpose() * Psi_prev) / (this->lambda + phi_i.transpose() * Psi_prev * phi_i) << std::endl;
	std::cout << std::endl;//*/



	/*if (this->started && std::fabs(phi_i(1)) < 1e-5) {
		this->estimateK = true;
		std::cout << "Stopping estimating K." << std::endl;
	}//*/

	//if (t - this->treset > 0.15) {//0.15 for single layer datasets - const an sinusoidal , multi layer dataset - const
	//if (t - this->treset > 0.15) {//0.02 for multi layer dataset - sinusoidal
	if (t - this->treset > 1.0 / this->crFrequency) {//0.02 for multi layer dataset - sinusoidal
		Psi_i = Psi0;
		this->treset = t;
	}//*/


	// Test: this should be uncommented
	/*if (Psi_i.trace() < this->covResetThreshold) {
		Psi_i = Psi0;
	}//*/

	if (!this->estimateK) {
		theta_i(0) = 0.0;
		Psi_i(0, 0) = 0.0;
		//this->covResetThreshold = Psi_i(1, 1) * 0.1;
	}



	// Run the layer transition detection algorithm
	//detectTransition = this->layerDetection(err_i, recFz_i);

	// Set the next samples in the corresponding containers
	this->theta.push_back(theta_i);
	this->covMat.push_back(Psi_i);
	this->err.push_back(err_i);
	this->recFz.push_back(recFz_i);
	this->fEl.push_back(fEl_i);
	this->fVis.push_back(fVis_i);

	// Test: this should be commented
	/*if (!detectTransition) {  // If true, they have been already pushed back in layerDetection function, so push must not be repeated
		this->covMat.push_back(Psi_i);
	}//*/

}


/**
* @brief Detection function
* Tissue Layer transition detection algorithm based on statistical evaluation of the error signal
* @param e_k the k-th sample of the error signal
* @param e_k the k-th sample of the reconstructed force signal
* @return the transition detection flag
*/
bool RLS::cusumRuptureDetection(const float& ek, const float& fk) {

	// Define the required decision variables
	float sk, gamma, ni, gk1, gk_rupture, g_rupture_arg;
	bool detection;

	// Assign the required decision variables
	sk = ek * ek;

	gamma = (this->sigma1 * this->sigma1 - this->sigma0 * this->sigma0) * 0.5;
	ni = (this->sigma1 * this->sigma1 + this->sigma0 * this->sigma0) * 0.5;

	// Get the latest sample of the decision function
	gk1 = this->gRuptureFcn.back();

	// Compute the new value of the decision function
	//float sk_weight = (this->ruptureEvent) ? 0.01 : 1.0;
	float sk_weight = (this->ruptureEvent) ? 1.0 : 1.0;
	g_rupture_arg = gk1 + sk_weight * sk - ni;
	gk_rupture = max(g_rupture_arg, 0.0);

	// Evaluate if the tissue layer has been traversed or not
	detection = (gk_rupture > gamma) ? true : false;

	bool detRaising = detection && !this->ruptureEvent;
	this->ruptureEvent = detection;

	if (detRaising){
		this->transitionNum++;
		this->layerErr.clear();
	}

	// Update the decision function buffer
	this->gRuptureFcn.push_back(gk_rupture);
	this->gRuptureArg.push_back(g_rupture_arg);

	//this->transitionDetection.push_back(detection);


	return detRaising;
}

//// OBSOLETE
/**
* @brief Layer detection function (overload with sigma parameters)
* Implement the layer detection passing the CUSUM sigma parameters as input to the function
* @param ek: the current sample of the reconstruction error
* @return the boolean detection signal
*/
bool RLS::cusumPuncturingDetection(const float& ek) {

	bool detRaising, puncturing;
	float sk, sk_weight, sig0, sig1, gamma, ni, g_punct_k, g_punct_km1, g_punct_arg;

	sig0 = 0.01;
	sig1 = 0.03;
	gamma = (sig1 * sig1 - sig0 * sig0) * 0.5;
	ni = (sig1 * sig1 + sig0 * sig0) * 0.5;
	sk = ek * ek;

	// Get the latest sample of the puncture decision function
	g_punct_km1 = this->gPunctureFcn.back();

	// Evaluate the CUSUM function for puncturing
	sk_weight = (this->punctureEvent) ? 0.01 : 1.0;
	g_punct_arg = g_punct_km1 + sk_weight * sk - ni;
	g_punct_k = max(g_punct_arg, 0.0);

	// Evaluate puncture event at the current time
	puncturing = (g_punct_k > gamma) ? true : false;

	// Update the raising boolean variable to be returned
	detRaising = puncturing && !this->punctureEvent;
	this->punctureEvent = puncturing;

	// Update the decision function buffer
	this->gPunctureFcn.push_back(g_punct_k);
	this->gPunctureArg.push_back(g_punct_arg);

	return detRaising;

}


/**
* @brief Log save function
* Store the acquired data in stringstream objects for logging
* @param logPath: the name of the folder in which data have to be saved
*/
void RLS::saveStats(const char* logPath) {

	// Create the DBWrapper object
	DBWrapper db;

	int N = this->inputSize;

	// Store the acquired data 
	// Phi
	for (int i = 0; i < this->phi.size(); i++) {
		this->phiSS << this->timestamps[i];
		for (int j = 0; j < N; j++) {
			this->phiSS << ", " << this->phi[i](j);
		}
		this->phiSS << "; " << std::endl;
	}

	// Ground truth force
	for (int i = 0; i < this->gtFz.size(); i++) {
		this->gtFzSS << this->timestamps[i] << ", " << this->gtFz[i] << "; " << std::endl;
	}

	// Reconstructed force
	for (int i = 0; i < this->recFz.size(); i++) {
		this->recFzSS << this->timestamps[i] << ", " << this->recFz[i] << "; " << std::endl;
	}

	// Elastic force
	for (int i = 0; i < this->fEl.size(); i++) {
		this->fElSS << this->timestamps[i] << ", " << this->fEl[i] << "; " << std::endl;
	}

	// Viscous force
	for (int i = 0; i < this->fVis.size(); i++) {
		this->fVisSS << this->timestamps[i] << ", " << this->fVis[i] << "; " << std::endl;
	}

	// Reconstructed force derivative
	for (int i = 0; i < this->d_f.size(); i++) {
		this->dfSS << this->timestamps[i] << ", " << this->d_f[i] << "; " << std::endl;
	}

	// Force error
	for (int i = 0; i < this->err.size(); i++) {
		this->errSS << this->timestamps[i] << ", " << this->err[i] << "; " << std::endl;
	}

	// Force error derivative
	for (int i = 0; i < this->layerErr.size(); i++) {
		this->derrSS << this->timestamps[i] << ", " << this->layerErr[i] << "; " << std::endl;
	}

	// Theta
	for (int i = 0; i < this->theta.size(); i++) {
		this->thetaSS << this->timestamps[i];
		for (int j = 0; j < N; j++) {
			this->thetaSS << ", " << this->theta[i](j);
		}
		this->thetaSS << "; " << std::endl;
	}

	// Psi matrix
	for (int i = 0; i < this->covMat.size(); i++) {
		this->covMatSS << this->timestamps[i];
		for (int j = 0; j < N; j++) {
			for (int k = 0; k < N; k++) {
				this->covMatSS << ", " << this->covMat[i](j,k);
			}
		}
		this->covMatSS << "; " << std::endl;
	}

	// Detected transitions
	for (int i = 0; i < this->transitionDetection.size(); i++) {
		this->detTransSS << this->timestamps[i] << ", " << this->transitionDetection[i] << ";" << std::endl;
	}

	// Detected contacts
	for (int i = 0; i < this->contactDetection.size(); i++) {
		this->detContactSS << this->timestamps[i] << ", " << this->contactDetection[i] << ";" << std::endl;
	}

	// GT transitions
	for (int i = 0; i < this->gtTransitions.size(); i++) {
		this->gtTransSS << this->timestamps[i] << ", " << this->gtTransitions[i] << ";" << std::endl;
	}

	// Compensated force
	for (int i = 0; i < this->compFz.size(); i++) {
		this->fcompSS << this->timestamps[i] << ", " << this->compFz[i] << ";" << std::endl;
	}

	// Set db file and write data for each object
	std::string fullpath = std::string(logPath) + "\\";
	std::cout << "fullpath = " << fullpath << std::endl;

	// Write the input content on the given file
	// Theta
	std::string filename = fullpath + "theta.txt";
	db.setDBFile(filename.c_str());
	db.write(this->thetaSS);

	// Phi
	filename = fullpath + "phi.txt";
	db.setDBFile(filename.c_str());
	db.write(this->phiSS);

	// Ground truth force
	filename = fullpath + "gtFz.txt";
	db.setDBFile(filename.c_str());
	db.write(this->gtFzSS);

	// Reconstructed force
	filename = fullpath + "recFz.txt";
	db.setDBFile(filename.c_str());
	db.write(this->recFzSS);

	// Elastic component of the reconstructed force
	filename = fullpath + "elFz.txt";
	db.setDBFile(filename.c_str());
	db.write(this->fElSS);

	// Viscous component of the reconstructed force
	filename = fullpath + "visFz.txt";
	db.setDBFile(filename.c_str());
	db.write(this->fVisSS);

	// Reconstructed force derivative
	filename = fullpath + "recFzDerivative.txt";
	db.setDBFile(filename.c_str());
	db.write(this->dfSS);

	// Force error
	filename = fullpath + "FzError.txt";
	db.setDBFile(filename.c_str());
	db.write(this->errSS);

	// Force error derivative
	filename = fullpath + "FzErrorDerivative.txt";
	db.setDBFile(filename.c_str());
	db.write(this->derrSS);

	// Psi matrix
	filename = fullpath + "Psi.txt";
	db.setDBFile(filename.c_str());
	db.write(this->covMatSS);

	// Detected transitions
	filename = fullpath + "detected_transitions.txt";
	db.setDBFile(filename.c_str());
	db.write(this->detTransSS);

	// Detected transitions
	filename = fullpath + "detected_contacts.txt";
	db.setDBFile(filename.c_str());
	db.write(this->detContactSS);

	// GT transitions
	filename = fullpath + "true_transitions.txt";
	db.setDBFile(filename.c_str());
	db.write(this->gtTransSS);

	// Compensated force
	filename = fullpath + "compensated_force.txt";
	db.setDBFile(filename.c_str());
	db.write(this->fcompSS);

}

