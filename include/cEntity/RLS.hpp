#ifndef RLS_HPP_
#define RLS_HPP_

// Standard Header files
#include <vector>
#include <sstream>

// Eigen Header files
#include <Eigen\Dense>

// Project Header files
#include "utils.hpp"
#include "Timer.hpp" // only for DEBUG, TBR

enum INTERACTION_MODEL {KELVIN_VOIGT, GEROVICH, NORMALIZED_GEROVICH, NORMALIZED_GEROVICH_WITH_CONST};

class RLS {

public:

	/**
	* @brief Default Construcor of the RLS object
	*/
	RLS() {}

	/**
	* @brief Default Destroyer of the RLS object
	*/
	~RLS() {}

	/**
	* @brief Set function
	* Set the input data size for the RLS algorithm
	* @param s: the input data size
	*/
	inline void setInputSize(const int& s) { this->inputSize = s; }

	/**
	* @brief Get function
	* Get the input data size for the RLS algorithm
	* @return the input data size
	*/
	inline int getInputSize() { return this->inputSize; }

	/**
	* @brief Get function
	* Retrieves the force interaction model adopted for the RLS algorithm
	*/
	inline int getInteractionmodel() { return this->interactionModelType; }

	/**
	* @brief Set function
	* Set the forgetting factor for the RLS algorithm
	* @param s: the forgetting factor
	*/
	inline void setLambda(const float& l) { this->lambda = l; }


	/**
	* @brief Set function
	* Set the vector of the diagonal of the initial covariance matrix Psi0
	* @param s: the vector of the diagonal of the initial covariance matrix Psi0
	*/
	inline void setPsi0(const Eigen::VectorXf& Psi0diag) { this->Psi0.setZero(Psi0diag.size(), Psi0diag.size()); this->Psi0 = Psi0diag.matrix().asDiagonal(); }

	/**
	* @brief Set function
	* Set the covariance matrix reset threshold value
	* @param th: the threshold value
	*
	*/
	inline void setCovMatThreshold(const float& th) { this->covResetThreshold = th; }

	/**
	* @brief Init function
	* Initiialize the data of the RLS algorithm
	* @param interactionType_: an integer specifying the adopted interaction model
	* @param lambda_: the forgetting factor
	* @param Psi0diag: vector of the diagonal of the initial covariance matrix Psi0
	* @param sig0 the singular value denoting the default variation of the error signal
	* @param sig1 the singular value denoting the abrupt variation of the error signal
	*/
	void initData(const int& interactionType_, const float& lambda_, const Eigen::VectorXf& Psi0diag, const float& sig0, const float& sig1);

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
	void loadData(const int& logNum, std::vector<Eigen::VectorXf>& q,
		std::vector<Eigen::VectorXf>& qd,
		std::vector<Eigen::VectorXf>& r,
		std::vector<Eigen::VectorXf>& tau,
		std::vector<Eigen::VectorXf>& f);

	/**
	* @brief Run function
	* Run the i-th iteration of the RLS algorithm
	* @param t: the i-th timestamp
	* @param gtFz_i: the i-th ground truth force data sample
	* @param phi_i: the i-th input data sample
	*/
	void run(const double& t, const float& gtFz_i, const Eigen::VectorXf& phi_i);

	/**
	* @brief Log save function
	* Store the acquired data in stringstream objects for logging
	* @param logPath: the name of the folder in which data have to be saved
	*/
	void saveStats(const char* logPath);

	/**
	* @brief Get function
	* Get the latest sample of the reconstructed force on the needle tip
	* @return the value of the reconstructed force
	*/
	inline float getCurrentRecFz() { return this->recFz.back(); }

	/**
	* @brief Get function
	* Get the latest sample of the force error
	* @return the value of the force error
	*/
	inline float getLatestErrorSample() { return this->err.back(); }

	/**
	* @brief Get function
	* Get the latest sample of the estimated parameters theta
	* @return the value of the estimated parameters
	*/
	inline Eigen::VectorXf getCurrentEstParams() { return this->theta.back(); }

	/**
	* @brief Get function
	* Get the latest sample of the covariance matrix
	* @return the value of the latest sample of the covariance matrix
	*/
	inline Eigen::MatrixXf getLatestCovMat() { return this->covMat.back(); }

	/**
	* @brief Set function
	* Set the singular values sigma0 and sigma1 to distinguish default and abrupt changes
	* in the implementation of the tissue layer transition detection algorithm
	* @param s0 the singular value denoting the default variation of the error signal
	* @param s1 the singular value denoting the abrupt variation of the error signal
	*/
	inline void setDecisionSigmas(const float& s0, const float& s1) { this->sigma0 = s0; this->sigma1 = s1; }

	/**
	* @brief Detection function
	* Tissue Layer transition detection algorithm based on statistical evaluation of the error signal
	* @param e_k the k-th sample of the error signal
	* @param e_k the k-th sample of the reconstructed force signal
	* @return the transition detection flag
	*/
	bool cusumRuptureDetection(const float& ek, const float& fk);


	/**
	* @brief Layer detection function (overload with sigma parameters)
	* Implement the layer detection passing the CUSUM sigma parameters as input to the function
	* @param ek: the current sample of the reconstruction error
	* @return the boolean detection signal
	*/
	bool cusumPuncturingDetection(const float& ek);

	/**
	* @brief Get function
	* Retrieve the latest sample of the boolean transition function
	* @return the boolean value of the latest sample of the transition function
	*/
	inline bool getLatestTransitionDetectionSample() { return this->transitionDetection.back(); }

	/**
	* @brief Get function
	* Retrieve the latest sample of the rupture decision function
	* @return the value of the rupture decision function
	*/
	inline float getLatestGRuptureFcnSample() { return this->gRuptureFcn.back(); }

	/**
	* @brief Get function
	* Retrieve the latest sample of the puncture decision function
	* @return the value of the puncture decision function
	*/
	inline float getLatestGPunctureFcnSample() { return this->gPunctureFcn.back(); }

	/**
	* @brief Get function [DEBUG purposes]
	* Retrieve the latest sample of the rupture decision function entry
	* @return the value of the decision function entry
	*/
	inline float getLatestGRuptureArg() { return this->gRuptureArg.back(); }

	/**
	* @brief Get function [DEBUG purposes]
	* Retrieve the latest sample of the puncture decision function entry
	* @return the value of the decision function entry
	*/
	inline float getLatestGPunctureArg() { return this->gPunctureArg.back(); }

	/**
	* @brief Get function [DEBUG purposes]
	* Retrieve the latest sample of the force derivative signal
	* @return the value of the force derivative signal
	*/
	inline float getLatestForceDerivativeSample() { return this->d_f.back(); }

	/**
	* @brief Get function [DEBUG purposes]
	* Retrieve the latest sample of the elastic component of the reconstructed force signal
	* @return the elastic component
	*/
	inline float getLatestElasticForceSample() { return this->fEl.back(); }

	/**
	* @brief Get function [DEBUG purposes]
	* Retrieve the latest sample of the viscous component of the reconstructed force signal
	* @return the viscous component
	*/
	inline float getLatestViscousForceSample() { return this->fVis.back(); }

	/**
	* @brief Set function
	* Set the latest GT transition sample acquired from external application
	* @parameter ts the GT transition sample to be set
	*/
	inline void pushGTTransitionSample(const float& ts) { this->gtTransitions.push_back(ts); }

	/**
	* @brief Set function
	* Set the starting time of the RLS algorithm
	* @param  st: the starting time
	*/
	inline void setStartTime(const float& st) { this->start_t = st; }

	/**
	* @brief Set function
	* Set the started flag
	* @param s: the started flag
	*/
	inline void setStarted(const bool& s) { this->started = s; }

	/**
	* @brief Set function
	* Set the restored flag
	* @param r: the restpred flag
	*/
	inline void setRestored(const bool& r) { this->restored = r; }

	/**
	* @brief Check function
	* Check the value of the started flag
	* @return true if the algorithm has started
	*/
	inline bool checkStarted() { return this->started; }

	/**
	* @brief Push function
	* Push the new acquired sample data in the corresponding buffers
	* @param t: the timestamp
	* @param gtfz: the ground truth force data on z-axis
	* @param phi: the phi input vector
	* @param runningCondition: true if the conditions to run the current iteration of the algorithm are satisfied.
	*	                       If so, do not forward recFz, theta and Psi former data in the buffer (defaul is true(
	*/
	void pushData(const double& t, const float& gtfz, const Eigen::VectorXf& phi, const bool& runningCondition = true);

	/**
	* @brief Pop function
	* Remove the last element from the theta buffer
	*/
	void popLatestThetaSample() { this->theta.pop_back(); }

	/**
	* @brief Push function
	* Push the input thetaSample element in the theta buffer
	* @param thetaSample: thet new theta sample to add
	*/
	void pushNewThetaSample(const Eigen::VectorXf& thetaSample) { this->theta.push_back(thetaSample); }

	/**
	* @brief Reset function
	* Reset the covariance matrix and store the value in the buffer
	*/
	inline void resetCovMat() { this->covMat.pop_back(); this->covMat.push_back(Psi0); }

	/**
	* @brief Reset function
	* Reset the covariance matrix with an input matrix and store the value in the buffer
	* @param Psi: initial covariance matrix to be set
	*/
	inline void resetCovMat(const Eigen::MatrixXf& Psi) { this->covMat.pop_back(); this->covMat.push_back(Psi); this->Psi0 = Psi; }

	/**
	* @brief Get function
	* Get the delta parameter of the dead zone function for the covariance matrix
	* @return the delta parameter
	*/
	inline float getDeadZoneFcnDelta() { return this->delta_dzf; }

	/**
	* @brief Get function
	* Get the current timestamp
	* @return the current timestamp
	*/
	inline float getCurrentTimeStamp() { return this->timestamps.back(); }

	/**
	* @brief Set function
	* Set the flag estabilishing if the K coefficient must be estimated
	* @param inc: the boolean value
	*
	*/
	inline void includeKCoeffInEstimation(const bool& inc) { this->estimateK = inc; }

	/*
	* @brief Get function
	* Get the last sample of the force standard deviation
	* @return the sample of standard deviation
	*/
	inline float getFzStdDevSample() { return this->fzStdDev; }

	/**
	* @brief Initialization function
	* Compute the exact initialization of the RLS algorithm with a set of m initial measurements
	* @param fBuff: the buffer of the first m force measurements
	* @param phiBuff: the buffer of the fir
	*/
	void exactInitialization(const std::vector<float>& fBuff, const std::vector <Eigen::VectorXf>& phiBuff);

	/**
	* @brief Set function
	* Set the ratio of the trace of the initial covariance matrix used to compute the covariance reset threshold
	* @parma crr: the input ratio
	*/
	void setCRratio(const float& crr) { this->crRatio = crr; }

	/**
	* @brief Set function
	* Set the frequency of the covariance matrix used to compute the covariance reset 
	* @parma crr: the input frequency
	*/
	void setCRfrequency(const float& crf) { this->crFrequency = crf; }

	/**
	* @brief Set function
	* Set the compensated force in the corresponding buffer
	* @param fc: the latest compensated force sample to be set in the buffer
	*/
	void pushCompensatedForceSample(const float& fc) { this->compFz.push_back(fc); }

	/**
	* @brief Set function
	* Set the detected contact in the corresponding buffer
	* @param contact: the latest detected contact sample to be set in the buffer
	*/
	void pushContactDetectionSample(const bool& contact) { this->contactDetection.push_back(contact); }

	/**
	* @brief Set function
	* Set the detected transitionin the corresponding buffer
	* @param contact: the latest detected transition sample to be set in the buffer
	*/
	void pushTransitionDetectionSample(const bool& trans) { this->transitionDetection.push_back(trans); }

	/**
	* @brief Set function
	* Set the latest sample of the g function in the corresponding buffer
	* @param contact: the latest sample of the g function to be set in the buffer
	*/
	void pushGRuptureArgSample(const float& gentry) { this->gRuptureArg.push_back(gentry); }

	/**
	* @brief Set function
	* Set the latest sample of the g function in the corresponding buffer
	* @param contact: the latest sample of the g function to be set in the buffer
	*/
	void pushGPunctureArgSample(const float& gentry) { this->gPunctureArg.push_back(gentry); }

	/**
	* @brief Set function
	* Set the latest sample of the rupture decision function in the corresponding buffer
	* @param gval: the latest sample of the rupture decision function to be set in the buffer
	*/
	void pushGRuptureFcnSample(const float& gval) { this->gRuptureFcn.push_back(gval); }

	/**
	* @brief Set function
	* Set the latest sample of the puncture decision function in the corresponding buffer
	* @param gval: the latest sample of the puncture decision function to be set in the buffer
	*/
	void pushGPunctureFcnSample(const float& gval) { this->gPunctureFcn.push_back(gval); }


	/**
	* @brief Set function
	* Set the initial state for the consided RLS instance
	* @param theta0: the initial state vector
	*/
	void setInitialState(const Eigen::VectorXf& theta0) { this->theta.clear(); this->theta.push_back(theta0); }
	

	inline float getErrStdDev() { return this->errStdDev; }

private:

	int interactionModelType;							//!< Integer specifying the adopted interaction model
	int inputSize;										//!< Input data size
	std::vector < float > timestamps;					//!< Vector with the timestamps associated to the data
	std::vector < float > recFz;						//!< Reconstructed force along the needle shaft direction 
	std::vector < float > fEl;							//!< Elastic component of the reconstructed force (f = K*x)
	std::vector < float > fVis;							//!< Viscous component of the reconstructed force (f = D*x*v)
	std::vector < float > gtFz;							//!< Ground truth force along the needle shaft direction 
	std::vector < float > compFz;						//!< Ground truth force along the needle shaft direction 
	std::vector < float > err;							//!< Force error between ground truth and reconstructed force
	std::vector < float > layerErr;						//!< Derivative of the force error between ground truth and reconstructed force
	std::vector < float > d_f;							//!< Derivative of the reconstructed force
	std::vector < float > gRuptureFcn;					//!< Decision function adopted for tissue layer rupture detection 
	std::vector < float > gPunctureFcn;					//!< Decision function adopted for tissue layer puncture detection
	std::vector < float > gRuptureArg;					//!< Decision function argument for tissue layer rupture detection
	std::vector < float > gPunctureArg;					//!< Decision function argument for tissue layer puncture detection
	std::vector < bool > transitionDetection;			//!< Transition detection signal
	std::vector < bool > contactDetection;				//!< Contact detection signal
	std::vector < bool > gtTransitions;					//!< Ground-truth Transition signal
	std::vector < Eigen::VectorXf > theta;				//!< Estimating coefficient vector
	std::vector < Eigen::VectorXf > phi;				//!< Measurement input data
	std::vector < Eigen::MatrixXf > covMat;				//!< Covariance matrixs

	float lambda;										//!< Forgetting factor
	float covResetThreshold;							//!< Threshold of the covariance resetting condition
	float crRatio;										//!< Ratio of the trace of the initial covariance matrix used to compute the covariance reset threshold
	float crFrequency;									//!< Frequency of the initial covariance matrix used to compute the covariance reset 
	float sigma0;										//!< Singular value denoting the default variation of the error signal
	float sigma1;										//!< Singular value denoting the abrubt variation of the error signal
	Eigen::MatrixXf Psi0;								//!< Initial covariance matrix
	int transitionNum;									//!< Number of occurred transitions
	bool ruptureEvent;									//!< Boolean variable coding the puncturing event
	bool punctureEvent;									//!< Boolean variable coding the puncturing event
	float delta_dzf;									//!< Delta parameter of the dead-zone function

	bool estimateK;

	bool started;										//!< Started flag
	bool restored;										//!< Restored flag: flag to state if the algorithm has been restored after clutch (in case of teleoperation)
	float start_t;										//!< Starting time of the RLS

	float treset;
	float ttrue;
	Timer clock;

	int stdvarWinSize;
	
		/// TEST NORMALIZED SIGNAL
		float S;
		float S2;
		float S11;
		float fzMean;
		float fzStdDev;

		float errSum;
		float errSum2;
		float errMean;
		float errStdDev;

	
	std::stringstream errSS;							//!< Stringstream object of the force error
	std::stringstream derrSS;							//!< Stringstream object of the derivative of the force error
	std::stringstream dfSS;								//!< Stringstream object of the derivative of the reconstructed force 
	std::stringstream recFzSS;							//!< Stringstream object of the reconstructed force
	std::stringstream fElSS;							//!< Stringstream object of the elastic component of the reconstructed force
	std::stringstream fVisSS;							//!< Stringstream object of the viscous component of the reconstructedforce
	std::stringstream gtFzSS;							//!< Stringstream object of the ground truth force
	std::stringstream thetaSS;							//!< Stringstream object of the theta vector
	std::stringstream phiSS;							//!< Stringstream object of the phi vector
	std::stringstream covMatSS;							//!< Stringstream object of the covariance Psi matrix
	std::stringstream detTransSS;						//!< Stringstream object of the detected transitions
	std::stringstream detContactSS;						//!< Stringstream object of the detected contacts
	std::stringstream gtTransSS;						//!< Stringstream object of the ground truth transitions
	std::stringstream fcompSS;							//!< Stringstream object of the ground truth transitions

};

#endif // RLS_HPP_
