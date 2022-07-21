#ifndef FORCEFEEDBACK_NEEDLEINSERTION_HPP_
#define FORCEFEEDBACK_NEEDLEINSERTION_HPP_


// Project Header files
#include "TaskStrategy.hpp"
#include "Phantom.hpp"
#include "MultiLayerGelPhantom.hpp"
#include "FTSensor.hpp"
#include "Geomagic.hpp"
#include "Armband.hpp"
#include "HRing.hpp"
#include "WeartDevice.hpp"

// Eigen Header files
#include <Eigen\Dense>

class ForceFeedbackNeedleInsertionStrategy : public TaskStrategy {

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		/*
		* @brief Default constructor
		* @param p: the pointer to Phantom
		* @param ft: the pointer to FTSesnor
		* @param hs: a map list of haptic interfaces
		* @param v: the pointer to VREPProxy
		* @param r: the pointer to RobotInterface
		*/
		ForceFeedbackNeedleInsertionStrategy(Phantom* p, FTSensor* ft, std::map <std::string, HapticInterface* > h, VREPProxy* v = nullptr, RobotInterface* r = nullptr);

	/*
	* @brief Default destroyer
	*
	*/
	~ForceFeedbackNeedleInsertionStrategy();

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
	* @brief Terminate function
	* Terminate the NeedleTissueIntEstStrategy class
	*/
	void terminate();

	/**
	* @brief Force feedback computation function
	* Compute the force feedback signal to be sent back to the haptic device
	* @param: integer id specifying the haptic device to be accounted on which the feedback is expected to be sent (default is 0, i.e., the first 
	* haptic device of the specified list
	* @return the 3D vector of the linear force feedback signal
	*/
	std::vector < Eigen::VectorXf > computeForceFeedback(const int& hapticIdx = 0);

	/**
	* @brief Visualization function
	* Request to the VREP simulator to render the line of the needle direction in the virtual scene
	*/
	void ForceFeedbackNeedleInsertionStrategy::showNeedleDirection();


private:

	/* Entities */
	MultiLayerGelPhantom staticPhantom;			//!< Pointer to a Phantom object
	Phantom* phantom;							//!< Pointer to a Phantom object
	//HapticInterface* haptic;					//!< Pointer to a HapticInterface object
	std::map <std::string, HapticInterface* > haptics; 
	FTSensor* ftsensor;							//!< Pointer to a FTSensor object

	/* Class variables */
	float cumulativeFriction;					//!< Cumulative friction of the previous layers of the punctured target
	float simPuncturedLayerCounter;				//!< Signal acquired from the simulated scene with the increasing number of traversed layers
	float currentLayerID;						//!< ID of the current layer where the needle is
	std::vector < float > z_anchors;			//!< Array with the starting z of the layers
	Eigen::Vector3f anchor;						//!< Anchor point to evaluate the elastic force
	Eigen::VectorXf forcefeedback;				//!< 3D vector of the force feedback

	Eigen::Matrix4f Twr;						//!< Constant transformation matrix of the robot base frame in the virtual world reference frame of the simulator
	bool drawNeedleLine;						//!< Flag stating if the needle direction has to be rendered in the simulator scene
};


#endif // FORCEFEEDBACK_NEEDLEINSERTION_HPP_