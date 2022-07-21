// Project Header files
#include "KukaProxy.hpp"
#include "utils.hpp"
#include "Timer.hpp"

// Windows Header files
#include <Windows.h>

#include <fstream> /// TO BE REMOVED, IT's ONLY FOR TESTING
#include <sstream> /// TO BE REMOVED, IT's ONLY FOR TESTING

#define EOK 0

/**
* @brief Default constructor
*/
KukaProxy::KukaProxy() : RobotProxy(){

	this->availability(false);
	this->setRunning(false);
	this->t_ = 0.0;

	this->stateReady = false;
	this->inputReady = false;
}

/**
* @brief Default destroyer
*/
KukaProxy::~KukaProxy()
{
}



/**
* @brief Default init function
*/
void KukaProxy::init() {

	// Debug print
	debugPrint<char>("[KukaProxy] This is a debug print",NULL);

	// FRICommands
	this->input = FRICommands::Ones() * FRI_INIT_DUMMY_VALUE ;
	
	// FRI Joint Reference
	this->jointRefs = FRIJointRefs::Ones() * FRI_INIT_DUMMY_VALUE;;

	// FRIState
	this->state.jointPositions.setZero();
	this->state.jointTorques.setZero();
	this->state.extEstJointTorques.setZero();


	stateReady = false;
	inputReady = false;
}

/**
* @brief Regulation routine
* Regulate the robot joint configuration to the input desired vector, using the Reflexxes motion library
* Require jointRefs variable to be set
*/
void KukaProxy::autonomousJointPositionRegulation() {

	TypeIRML					*RML = NULL;
	TypeIRMLInputParameters		*IP = NULL;
	TypeIRMLOutputParameters	*OP = NULL;
	float msrFloatJointPosition[NUMBER_OF_JOINTS];
	float cmdFloatJointPosition[NUMBER_OF_JOINTS];
	int	ResultValue;
	
	RML = new TypeIRML(NUMBER_OF_JOINTS, CYCLE_TIME);
	IP = new TypeIRMLInputParameters(NUMBER_OF_JOINTS);
	OP = new TypeIRMLOutputParameters(NUMBER_OF_JOINTS);
	std::memset(msrFloatJointPosition, 0.0, sizeof(float) * NUMBER_OF_JOINTS);
	std::memset(cmdFloatJointPosition, 0.0, sizeof(float) * NUMBER_OF_JOINTS);
	ResultValue = 0;
	
	// Availability check
	if ((FRI->GetCurrentControlScheme() != FastResearchInterface::JOINT_POSITION_CONTROL) || (!FRI->IsMachineOK())){

		std::cout << "Program is going to stop the robot." << std::endl;
		FRI->StopRobot();

		FRI->GetMeasuredJointPositions(msrFloatJointPosition);
		FRI->SetCommandedJointPositions(msrFloatJointPosition);

		std::cout << "Restarting the joint position control scheme." << std::endl;
		ResultValue = FRI->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL);

		if ((ResultValue != EOK) && (ResultValue != EALREADY))
		{
			std::cout << "An error occurred during starting up the robot..." << std::endl;
			delete	RML;
			delete	IP;
			delete	OP;

			std::exit(1);
		}
	}

	// Get the current meaured joint position configuration
	FRI->GetMeasuredJointPositions(msrFloatJointPosition);

	for (int i = 0; i < NUMBER_OF_JOINTS; i++)
	{
		IP->CurrentPosition->VecData[i] = (double)rad2deg<float>((msrFloatJointPosition[i]));
		IP->TargetPosition->VecData[i]  = (double)rad2deg<float>((this->jointRefs(i)));
		IP->MaxVelocity->VecData[i]     = (double)50.0;
		IP->MaxAcceleration->VecData[i] = (double)50.0;
		IP->SelectionVector->VecData[i] = true;
	}

	ResultValue = TypeIRML::RML_WORKING;


	while ((FRI->IsMachineOK()) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED))
	{
		FRI->WaitForKRCTick();

		ResultValue = RML->GetNextMotionState_Position(*IP, OP);

		if ((ResultValue != TypeIRML::RML_WORKING) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED))
		{
			std::cout << "ERROR during trajectory generation (" << ResultValue << "). " << std::endl;
		}


		for (int i = 0; i < NUMBER_OF_JOINTS; i++)
		{
			cmdFloatJointPosition[i] = deg2rad<double>(OP->NewPosition->VecData[i]);
		}

		FRI->SetCommandedJointPositions(cmdFloatJointPosition);

		*(IP->CurrentPosition) = *(OP->NewPosition);
		*(IP->CurrentVelocity) = *(OP->NewVelocity);
	}

	if (!FRI->IsMachineOK()){
		std::cout << "ERROR, machine is not ready." << std::endl;
		delete	RML;
		delete	IP;
		delete	OP;
		std::exit(1);
	}
	
	//std::memcpy(cmdEigJointPosition.data(), msrFloatJointPosition, sizeof(float)*NUMBER_OF_JOINTS);

	std::cout << "Desired joint configuration reached." << std::endl;


}


/**
* @brief Default run function
*/
void KukaProxy::run() {

	// Define required variables
	Eigen::Vector7f msrEigJointPositions;
	FRIState robotState;
	FRICommands cmdEigJointPosition;
	float msrFloatJointPosition[JOINT_NUM];
	float msrFloatJointVelocity[JOINT_NUM];
	float msrFloatJointTorque[JOINT_NUM];
	float estFloatExtJointTorque[JOINT_NUM];
	float estFloatExtCartForces[TWIST_DIM];
	float cmdFloatJointPosition[JOINT_NUM];
	float** friFloatMassMatrix;
	double tic, toc, tac, tictoc, dt, Ts, rate;
	Timer clock;
	int resval;

	// Get the clock rate
	Ts = CYCLE_TIME;
	rate = 1.0 / CYCLE_TIME;
	clock.setRate(rate);

	// Initialize data
	std::memset(msrFloatJointPosition, 0x0, JOINT_NUM * sizeof(float));
	std::memset(msrFloatJointVelocity, 0x0, JOINT_NUM * sizeof(float));
	std::memset(msrFloatJointTorque, 0x0, JOINT_NUM * sizeof(float));
	std::memset(estFloatExtJointTorque, 0x0, JOINT_NUM * sizeof(float));
	std::memset(estFloatExtCartForces, 0x0, TWIST_DIM * sizeof(float));
	std::memset(cmdFloatJointPosition, 0x0, JOINT_NUM * sizeof(float));
	friFloatMassMatrix = new float*[JOINT_NUM];
	for (int i = 0; i < JOINT_NUM; i++) {
		friFloatMassMatrix[i] = new float[JOINT_NUM];
		std::memset(friFloatMassMatrix[i], 0x0, JOINT_NUM * sizeof(float));
	}
	//msrEigJointPositions = this->getFRIState().jointPositions;

	// Sleep
	clock.timeSleep(1.0);

	// Initialize the Fast Research Interface object
	FRI = new FastResearchInterface(".\\config\\980039-FRI-Driver.init");
	if ((FRI->GetCurrentControlScheme() != FastResearchInterface::JOINT_POSITION_CONTROL) || (!FRI->IsMachineOK()))
	{
		debugPrint<char>("Program is going to stop the robot!!!", NULL);
		FRI->StopRobot();
		debugPrint<char>("Restarting the joint position control scheme.", NULL);
		resval = FRI->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL);
		if ((resval != EOK) && (resval != EALREADY))
		{
			debugPrint<char>("An error occurred during starting up the robot...", NULL);
			this->available = false;
			this->running = false;
		}
	}

	// Simultaneous get and set of the current joint position, for sanity check
	FRI->GetMeasuredJointPositions(msrEigJointPositions.data());
	FRI->SetCommandedJointPositions(msrEigJointPositions.data());//*/

	// Move to desired initial joint configuration
	if (this->getFRIJointRefs() != FRIJointRefs::Ones() * FRI_INIT_DUMMY_VALUE) {
		this->autonomousJointPositionRegulation();
	}

	// Update the measured joint position vector with the currently reached configuration
	FRI->WaitForKRCTick();
	FRI->GetMeasuredJointPositions(msrEigJointPositions.data());

	// Set the robot state 
	robotState.jointPositions = msrEigJointPositions;
	FRI->GetMeasuredJointTorques(robotState.jointTorques.data());
	FRI->GetEstimatedExternalJointTorques(robotState.extEstJointTorques.data());
	FRI->GetEstimatedExternalCartForcesAndTorques(robotState.extEstCartForces.data());
	FRI->GetCurrentMassMatrix(friFloatMassMatrix);
	FRI->GetCurrentGravityVector(robotState.friGravityVector.data());
	
	// Conver inertia matrix from float** to Eigen::Matrix 
	for (int i = 0; i < JOINT_NUM; i++) {
		for (int j = 0; j < JOINT_NUM; j++) {
			robotState.friInertiaMatrix(i, j) = friFloatMassMatrix[i][j];
		}
	}

	debugPrint<Eigen::Vector7f>("Setting the initial joint position vector in KukaProxy: ", robotState.jointPositions);
	this->setFRIState(robotState);

	// Notify the other instances that the proxy is available
	this->available = true;

	
	/// TEST
	this->setFRICommands(robotState.jointPositions);

	// Wait for proper commands initialization
	/*cmdEigJointPosition = FRICommands::Ones() * FRI_INIT_DUMMY_VALUE;
	while (cmdEigJointPosition == FRICommands::Ones() * FRI_INIT_DUMMY_VALUE) {// (*)

		// Measure starting time
		tic = clock.getCurTime();

		//----------------------------------------------------------------//
		// Query the proper commanded joint position vector
		cmdEigJointPosition = this->getFRICommands(); // (*)
		//debugPrint<char>("Waiting for non-zero cmdEigJointPosition vector ...", NULL);
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

	}//*/
	
	// When out from the while above, set running to true
	this->running = true;
	debugPrint<char>("Starting the main loop in KukaProxy ...", NULL);

	// condition variable predicates
	this->stateReady = false;
	this->inputReady = true;
	this->stateConsumed = true;
	this->inputConsumed = false;

	// Start the main loop

	while (this->isRunning() && FRI->IsMachineOK()) {	// Until a stop signal is not sent, and the machine is successfully running ...

		// Measure starting time
		tic = clock.getCurTime();

		FRI->WaitForKRCTick();

		/*** Sensing ***/
		FRI->GetMeasuredJointPositions(msrFloatJointPosition);
		FRI->GetMeasuredJointTorques(msrFloatJointTorque);
		FRI->GetEstimatedExternalJointTorques(estFloatExtJointTorque);
		FRI->GetEstimatedExternalCartForcesAndTorques(estFloatExtCartForces);
		FRI->GetCurrentMassMatrix(friFloatMassMatrix);
		FRI->GetCurrentGravityVector(robotState.friGravityVector.data());

		// Conver inertia matrix from float** to Eigen::Matrix 
		for (int i = 0; i < JOINT_NUM; i++) {
			for (int j = 0; j < JOINT_NUM; j++) {
				robotState.friInertiaMatrix(i, j) = friFloatMassMatrix[i][j];
			}
		}

		// Fill the structure to set in the class
		std::memcpy(robotState.jointPositions.data(), msrFloatJointPosition, sizeof(float)*NUMBER_OF_JOINTS);
		std::memcpy(robotState.jointTorques.data(), msrFloatJointTorque, sizeof(float)*NUMBER_OF_JOINTS);
		std::memcpy(robotState.extEstJointTorques.data(), estFloatExtJointTorque, sizeof(float)*NUMBER_OF_JOINTS);
		std::memcpy(robotState.extEstCartForces.data(), estFloatExtCartForces, sizeof(float)*TWIST_DIM);
		robotState.stamp = this->t_;

		// Change sign to consider these torques as the applied from the environment
		robotState.extEstJointTorques *= -1.0;

		// Set the robot state 
		this->setFRIState(robotState);
		
		/*** Acting ***/
		// Get the command from the Controller
		FRICommands cmdEigJointPosition = this->getFRICommands();
		FRI->SetCommandedJointPositions(cmdEigJointPosition.data());

		// Measure ending time
		toc = clock.getCurTime();

		tictoc = clock.elapsedTime(tic, toc);

		// Wait until Ts
		if (tictoc < Ts) {
			//clock.timeSleep(Ts - tictoc);
		}

		// Measure the final time after sleep to check the actual rate of the thread
		tac = clock.getCurTime();
		dt = clock.elapsedTime(tic, tac);

		this->t_ += dt;
		//std::cout << "[KP] rate = " << 1.0 / dt << " Hz" << std::endl;
	}

	// Stop the robot
	int ret = -1;
	ret = FRI->StopRobot();

	if (ret == EOK) {
		debugPrint<char>("FRI successfully closed the robot. Exiting from KUKA Proxy ... ", NULL);
	}
	else {
	
		debugPrint<char>("FRI not closed correctly!! Exiting from KUKA Proxy ... ", NULL);
	}


	// Delete Mass matrix pointer
	for (int i = 0; i < JOINT_NUM; i++) {
		delete friFloatMassMatrix[i];
	}
	delete friFloatMassMatrix;
}


/*
* @brief Get function
* Get the full robot state from the proxy
* @return the RobotState structure
*/
RobotState KukaProxy::getRobotState() {

	RobotState rs;
	FRIState fs;
	
	// Get the current joint configuration of the real KUKA robot from FRI
	fs = this->getFRIState();

	rs.msrJointPosition = fs.jointPositions;
	rs.msrJointTorque = fs.jointTorques;
	rs.extEstJointTorque = fs.extEstJointTorques;
	rs.extEstCartForce = fs.extEstCartForces;
	rs.stamp = fs.stamp;

	return rs;


}



/**
* @brief Set function
* Set the state of the robot aquired from FRI
* @param state_ the state of the robot (i.e., joint position, joint velocity, joint torque)
*/
void KukaProxy::setFRIState(const FRIState& state_) {
	{
		std::lock_guard<std::mutex> lock(this->stateMtx);
		if (!this->stateReady) {
			this->state = state_;
		}

		stateReady = true;
	}
	
	cvState.notify_all();
}

/**
* @brief Get function
* Get the state of the robot aquired from FRI
* @return the state of the robot (i.e., joint position, joint velocity, joint torque)
*/
FRIState KukaProxy::getFRIState() {

	FRIState ret;

	{
	std::unique_lock<std::mutex> lock(this->stateMtx);
	while (!stateReady && running) {
		cvState.wait(lock);
	}
	ret = this->state;
	stateReady = false;
	}
	return ret;
}



/**
*@brief Get function
* Get the full state of the robot from the proxy
* @return the full state of the robot from the proxy
*/
/*RobotState KukaProxy::getRobotState() {

	RobotState ret;
	ret.msrJointPosition.setZero(this->jointDOFs);
	ret.msrJointTorque.setZero(this->jointDOFs);
	ret.extEstJointTorque.setZero(this->jointDOFs);

	{
		std::unique_lock<std::mutex> lock(this->stateMtx);
		while (!stateReady && running) {
			cvState.wait(lock);
		}

		ret.msrJointPosition = this->state.jointPositions;
		ret.msrJointTorque = this->state.jointTorques;
		ret.extEstJointTorque = this->state.extEstJointTorques;
		ret.extEstCartForce = this->state.extEstCartForces;
		ret.stamp = this->state.stamp;

		stateReady = false;

	}

	return ret;

}//*/



/**
* @brief Set function
* Set the input of the robot as a vector of commanded joint positions
* @param the commanded joint position vector
*/
void KukaProxy::setFRICommands(const FRICommands& input_) {
	this->input = input_;
}

/**
* @brief Get function
* Retrieves the input of the robot as a vector of commanded joint positions
* @return the commanded joint position vector
*/
FRICommands KukaProxy::getFRICommands() {

	FRICommands ret;
	ret = this->input;
	return ret;
}


/**
* @brief Set function
* Set the Vector with reference joint position values
* @param the Vector with reference joint position values
*/
void KukaProxy::setFRIJointRefs(const FRIJointRefs& ref_) {

	this->jointRefs = ref_;

}

/**
* @brief Get function
* Retrieves the Vector with reference joint position values
* @return the Vector with reference joint position values
*/
FRIJointRefs KukaProxy::getFRIJointRefs() {

	return this->jointRefs;

}

/**
* @brief Default clear function
*/
void KukaProxy::clear() {

	// Set running on false
	this->setRunning(false);

}

/**
* @brief Set function 
* Set the current vector of measured joint position
* @param q: the current vector of measured joint position
*/
void KukaProxy::setMsrJointPosition(const Eigen::VectorXf& q) {

	this->state.jointPositions = q;

}

/**
* @brief Get function 
* Get the current vector of measured joint position
* @return the current vector of measured joint position
*/
Eigen::VectorXf KukaProxy::getMsrJointPosition() {

	return this->state.jointPositions;

}

/**
* @brief Set function 
* Set the current vector of measured joint torque
* @param tau: the current vector of measured joint torque
*/
void KukaProxy::setMsrJointTorque(const Eigen::VectorXf& tau) {

	this->state.jointTorques = tau;

}

/**
* @brief Get function 
* Get the current vector of measured joint torque
* @return the current vector of measured joint torque
*/
Eigen::VectorXf KukaProxy::getMsrJointTorque() {

	return this->state.jointTorques;

}

/**
* @brief Set function 
* Set the current vector of estimated external joint torque
* @param tau: the current vector of estimated external joint torque
*/
void KukaProxy::setEstExtJointTorque(const Eigen::VectorXf& tau) {

	this->state.extEstJointTorques = tau;

}

/**
* @brief Get function 
* Get the current vector of estimated external joint torque
* @return the current vector of estimated external joint torque
*/
Eigen::VectorXf KukaProxy::getEstExtJointTorque() {

	return this->state.extEstJointTorques;

}

/**
* @brief Set function 
* Set the current vector of estimated external Cartesian force
* @param q: the current vector of estimated external Cartesian force
*/
void KukaProxy::setEstExtCartForce(const Eigen::Vector6f& f) {

	this->state.extEstCartForces = f;

}

/**
* @brief Get function 
* Get the current vector of estimated external Cartesian force
* @return the current vector of estimated external Cartesian force
*/
Eigen::Vector6f KukaProxy::getEstExtCartForce() {

	return this->state.extEstCartForces;

}

