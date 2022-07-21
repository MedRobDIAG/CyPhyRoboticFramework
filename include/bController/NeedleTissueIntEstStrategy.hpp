#ifndef NEEDLE_TISSUE_INT_EST_STRATEGY_HPP_
#define NEEDLE_TISSUE_INT_EST_STRATEGY_HPP_

// Project Header files
#include "TaskStrategy.hpp"
#include "Phantom.hpp"
#include "MultiLayerGelPhantom.hpp"
#include "RLS.hpp"
#include "FTSensor.hpp"
#include "HapticInterface.hpp"

// Eigen Header files
#include <Eigen\Dense>

enum RLS_STATE { RLS_IDLE_STATE, RLS_RUPTURE_OCCURRED_STATE, RLS_RUPTURE_COMPENSATED_STATE, RLS_RUNNING_STATE, RLS_RUNNING_DETECTION_STATE, RLS_WAITING_STATE };
enum INSERTION_STATE {UNDEFINED, NO_RUPTURE, EXPECTED_RUPTURE, DETECTED_RUPTURE, FORCE_DROP_MEASURING, COMPENSATED_FORCE, INVERTED_MOTION_EVENT};

class NeedleTissueIntEstStrategy : public TaskStrategy {

public:
	
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/*
	* @brief Constructor of NeedleTissueIntEstStrategy
	* @param p: the pointer to Phantom
	* @param ft: the pointer to FTSesnor
	* @param hs: a map list of haptic interfaces
	* @param v: the pointer to VREPProxy
	* @param r: the pointer to RobotInterface
	*/
	NeedleTissueIntEstStrategy(Phantom* p, FTSensor* ft, std::map< std::string, HapticInterface* > hs, VREPProxy* v = nullptr, RobotInterface* r = nullptr);

	/*
	* @brief Destroyer of NeedleTissueIntEstStrategy
	*/
	~NeedleTissueIntEstStrategy();

	/**
	* @brief Init function
	* Initialize the TaskStrategy class
	*/
	void init();

	/**
	* @brief Main function
	* Implements the task of the NeedleTissueIntEstStrategy
	* @return the return state of the routine
	*/
	int execTask();

	/**
	* @brief Main function
	* Implements the task of the NeedleTissueIntEstStrategy
	* @return the return state of the routine
	*/
	int execTask_bkp();

	/**
	* @brief Main function
	* Implements the task of the NeedleTissueIntEstStrategy
	* @return the return state of the routine
	*/
	int execTask_bkp2();

	/**
	* @brief Terminate function
	* Terminate the NeedleTissueIntEstStrategy class
	*/
	void terminate();

	/**
	* @brief Visualization function
	* Request to the VREP simulator to render the line of the needle direction in the virtual scene
	*/
	void showNeedleDirection();

	/**
	* @brief Estimation function
	* Compute a batch estimation of the previous layer coefficients based on the input buffers
	* @param fzBuff: the buffer of the force measurement
	* @param phiBuff: the buffer of the input vector phi
	* @param old_depth: the previous depth considered for the layer
	* @param new_depth: the previous depth evaluated for the layer
	* @return the vector of estimated coefficients
	*/
	Eigen::VectorXf layerBatchEstimation(const std::vector <float>& fzBuff, const std::vector <Eigen::VectorXf>& phiBuff, const float& old_depth, const float& new_depth);

	/**
	* @brief Detection function
	* Detect detection of contacts between needle and tissues, based on the analysis of the force derivative signal
	* @param fraw: the raw force measurement
	* @param v: the insertion velocity
	* @return the boolean signal of the contact event
	*/
	bool detectNeedleTissueContacts(const float& fraw, const float& v);

	/**
	* @brief Set function
	* Set the size of the window for the evaluation of the rupture force interval
	* @param size: the input size
	*
	*/
	inline void setFzRuptureWinSize(const float& size) { this->fzRuptureWinSize = size; }

	/**
	* @brief Update function
	* Compute the variance of the input signal over the specified window with size localVarianceWindow
	* @param idx: the current index
	* @param fz: the current input signal sample
	* @return stddev: the output local standard deviation of the input signal
	*/
	float evaluateLocalVariance(const int& idx, const float& in);

private:

	RLS rlsAlgorithm;				//!< Static instance of the RLS algorithm
	std::vector<RLS*> layerRLS;		//!< Vector of RLS instances, one for each traversed layer
	Eigen::Matrix4f Tbee0;			//!< Initial Transformation matrix of the robot EE pose wrt the base frame
	Eigen::Matrix4f Twr;			//!< Constant transformation matrix of the robot base frame in the virtual world reference frame of the simulator
	bool drawNeedleLine;			//!< Flag stating if the needle direction has to be rendered in the simulator scene
	bool resOffsetSet;				//!< Flag stating if the offset on the residual vector has been set at the initializiation stages
	bool posOffsetSet;
	bool firstItInLayer;
	float z0;
	int rlsState;					//!< State of the RLS algorithm employed in this task
	int insertionState;				//!< State of the needle insertion task
	int gtTransitionNum;			//!< Number of true transitions occurring during the experiment
	float fzHold;
	Eigen::VectorXf res0;
	Eigen::Vector6f f0;
	float offsetSign;

	// For unknown depths
	int estimatedTransitionNum;		//!< Number of first needle-tissue contacts estimated 
	int estimatedRuptureNum;		//!< Number of first tissue ruptures estimated 
	bool estimatedRuptureSig;		//!< tissue ruptures estimated 
	int estimatedTransitionSig;
	bool estimatedTransitionRaise;
	bool estimatedTransitionTrail;

	/// test
	int offIdx_prev;
	float z_init;
	std::vector < float > layerDepths;
	std::vector < float > layerZ0;
	bool layerDepthSet;
	bool insertionAccomplished;
	int crossedLayersNum;
	float Bcomp;
	float BcompPrev;
	float Kcomp;
	bool cusumTransition;
	bool cusumTransitionPrev;

	float z_i_;
	float z_i_prev;
	float depth_i_;
	float fzPrev;
	float fzDot;
	float fzDotFilt;
	float fzFilt;

	std::vector <float> Bzsum;
	int nominalSimLayerID;
	int positionBasedLayerID;
	float simTransition;

	int task_idx;
	int rupture_idx;
	int fzRuptureWinSize;
	int localVarianceWinSize;
	bool waitingForRuptureWindowEnd;
	bool layer_compensated;
	bool ignoreCUSUMdetection;
	bool first_detection;
	
	std::vector < float > localVarianceWindow;
	std::vector < float > fzFullBuffer;
	std::vector < float > timeFullBuffer;
	std::vector < float > fzLayerBuffer;
	std::vector < float > vLayerBuffer;
	std::vector < float > timeLayerBuffer;
	std::vector < Eigen::VectorXf > phiLayerBuffer;
	int fzLayerBuffer_startIdx;
	int vLayerBuffer_startIdx;
	int timeLayerBuffer_startIdx;
	int phiLayerBuffer_startIdx;
	int fzLayerBuffer_endIdx;
	int vLayerBuffer_endIdx;
	int timeLayerBuffer_endIdx;
	int phiLayerBuffer_endIdx;

	std::vector < float >::iterator ruptureIt_begin;
	std::vector < float >::iterator ruptureIt_end;
	float max_force_val;

	float t_inv;
	bool autonomousMotionInverted;

	/* Entities */
	MultiLayerGelPhantom staticPhantom;			//!< Pointer to a Phantom object
	Phantom* phantom;							//!< Pointer to a Phantom object
	std::map< std::string, HapticInterface* > haptics; 
	FTSensor* ftsensor;							//!< Pointer to a FTSensor object
};


#endif // NEEDLE_TISSUE_INT_EST_STRATEGY_HPP_