// Project Header files
#include "ManualGuidanceStrategy.hpp"
#include "KUKARobot.hpp"
#include "Configuration.hpp"

/**
* @brief Init function
* Initialize the control strategy
*/
void ManualGuidanceStrategy::init() {}


/**
* @brief Main function
* Implements the control law associated to the ControlStrategy
*/
Eigen::VectorXf ManualGuidanceStrategy::controlLaw() {

	Eigen::VectorXf qdcmd; // commanded joint velocity vector
	Eigen::VectorXf gains; // Vector of manual guidance control gains
	Eigen::VectorXf residual; // Custom model-based residual
	Eigen::VectorXf modelResidual; // Custom model-based residual
	Eigen::VectorXf FRIresidual; // FRI residual (in case of using built-in residual of KUKA robot)
	std::vector<float> gainsvec; // Vector of gain for the manual guidance in vector form
	int jointNum = robot->getJointNum();
	float resThres;	// Residual threshold for the manual guidance
	float fcut; // Cut-off frequency 
	bool applyFilter; // flag stating if the residual vector must be filtered before use in control
	bool useModelResidual; // flag stating if the residual vector used for control is the the custom model-based one

	// Initialize the output joint velocity command vector
	qdcmd.setZero(jointNum);
	gains.setZero(jointNum);

	// Get Configuration object and useful variables
	Configuration* config = Configuration::GetInstance("");
	resThres = config->getMGResThresh();
	gainsvec = config->getMGGains();
	for (int i = 0; i < gainsvec.size(); i++) {
		gains(i) = gainsvec[i];
	}
	applyFilter = config->applyFilterForManualGuidance();
	useModelResidual = this->robot->useModelBasedResidual();
	fcut = config->getMGCutoffFreq();

	modelResidual = this->robot->getResidualVector();
	if (config->getRobotManipulatorType() == "KUKA") {
		FRIresidual = dynamic_cast<KUKARobot*>(this->robot)->getFRIResidual();
	}

	// Select chosen residual
	residual = (useModelResidual) ? modelResidual : FRIresidual;

	for (int i = 0; i < residual.size(); i++) {

		// Evaluate the joint velocity control inputs
		double opposite_sign = (residual(i) > 0) ? 1.0 : -1.0;
		if (std::fabs(residual(i)) > resThres) {
			qdcmd(i) = (gains(i) * (std::fabs(residual(i)) - resThres) * opposite_sign);
		}

	}



	return qdcmd;


}
