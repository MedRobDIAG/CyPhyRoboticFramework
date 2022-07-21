// Project Header files
#include "RegistrationStrategy.hpp"
#include "Configuration.hpp"
#include "SystemState.hpp"
#include "ICP.hpp"
#include "Timer.hpp"

/*
* @brief Constructor of RegistrationStrategy
* @param v: the pointer to VREPProxy
* @param p: the pointer to Phantom
* @param r: the pointer to RobotInterface
*/
RegistrationStrategy::RegistrationStrategy(AbdomenPhantom* p, VREPProxy* v, RobotInterface* r) : TaskStrategy(v,r){


	// Initialize variables
	this->regPointsNum = 0;
	this->robotPoints.setZero(MAX_REGISTRATION_POINTS_NUM, SPACE_DIM);

	/* Entities */
	this->phantom = p;

}

/*
* @brief Destroyer of RegistrationStrategy
*/
RegistrationStrategy::~RegistrationStrategy() {}


/**
* @brief Init function
* Initialize the TaskStrategy class
*/
void RegistrationStrategy::init() {}

/**
* @brief Main function
* Implements the task of the RegistrationStrategy
* @return the return state of the routine
*/
int RegistrationStrategy::execTask() {

	int ret;
	Eigen::MatrixXd phantomPoints;
	
	// Get Singleton Configuration object
	Configuration* config = Configuration::GetInstance("");

	// Get Singleton SystemState object
	SystemState* sys = SystemState::GetInstance();

	/*Eigen::Matrix<double,4,3> testMatrix;
	testMatrix <<
		-0.567311, -0.151416, 0.0179547,
	-0.570675, -0.00114742, 0.124223,
	-0.569664, 0.125422, 0.0199541,
	-0.459815, -0.0607869, 0.123795;//*/

	if (!this->areRegPointsAcquired()) {
		
		// Acquire (online) or load points (offline)
		if (config->isProcessingOnline()) { // Acquire online
		
			if(sys->getAction() == ACTION::REQUESTED){
			
				// Get the end-effector position 
				Eigen::Vector3d pee = this->robot->getEEPosition().cast<double>();

				// Add to the set of acquired points
				this->robotPoints.row(this->regPointsNum) = pee;
				//this->robotPoints.row(this->regPointsNum) = testMatrix.row(this->regPointsNum); /// TEST

				std::cout << "Acquired robot EE coordinates = " << this->robotPoints.row(this->regPointsNum) << std::endl;

				// Increae the number of acquired points for registration
				this->regPointsNum++;
				
				// Set the user request as cleared
				sys->setAction(ACTION::CLEARED);
			}

		}
		else { // Load offline
		
			// Load the external points
			this->robotPoints = phantom->loadAcquiredPoints("regPoints.txt");
			this->regPointsNum = robotPoints.rows();

		}


		// Set the return code
		ret = PROCESSING;
	
	
	}
	else { // if the points are acquired, launch ICP, store the result and rasise exit

		// Set loaded or acquired points in Phantom object
		this->phantom->setExtPoints(this->robotPoints);

		// Load point in the Phantom frame from file
		std::string phantomPointsFilename = config->getPhantomPtsFilename();
		phantomPoints = this->phantom->loadRadioPts(phantomPointsFilename.c_str());

		// ICP class instance
		ICP icp;
		icp.setIterations(300);
		icp.setDataAssociationKnown(true);

		// Populate icp dataset
		icp.populateSrcSet(this->robotPoints);
		icp.populateDstSet(phantomPoints);

		// Run the ICP algorithm
		icp.run();

		// Extract the registered transformation
		Eigen::Matrix4d T = icp.getTsol();

		// Set the transformation in the given class
		Eigen::Matrix4d Tkp = T.inverse();
		this->phantom->setTransformation(Tkp);
		std::cout << "Final transformation T = \n" << Tkp << std::endl;


		// Set the pose of the Phantom in the V-REP simulator
		//this->setPhantomPoseInSimulator(Tkp, true);
		this->vrep->setObjectPose("_Phantom", Tkp, "KukaOrigin", simx_opmode_blocking, this->simPort);

		// Save the transformation
		this->phantom->saveTransformation("Trp.txt");
		this->phantom->saveAcquiredPoints("regPoints.txt");

		// Set the return code
		ret = TASK_RETURN_CODE::FINISHED;
	}


	return ret;

}


/**
* @brief Terminate function
* Terminate the RegistrationStrategy class
*/
void RegistrationStrategy::terminate() {


}

/** +++ OBSOLETE +++ **/
/**
* @brief Set function
* Set the pose of the phantom in the V-REP simulator
* @param T: the input transformation defining the pose of the phantom
* @param simRunning: if the set has to be done while the simulation in running (default is false)
*/
/*void RegistrationStrategy::setPhantomPoseInSimulator(const Eigen::Matrix4d& T, const bool& simRunning) {

	float p_[SPACE_DIM];
	float q_[4];

	if (this->vrep->isAvailable()) {

		// Set on V-REP
		Eigen::Vector3f eigp = (T.block<3, 1>(0, 3)).cast<float>();
		Eigen::Matrix3d Rkp = T.topLeftCorner(SPACE_DIM, SPACE_DIM);
		Eigen::Quaternionf q(Rkp.cast<float>());
		std::memcpy(p_, eigp.data(), sizeof(float)*SPACE_DIM);

		q_[0] = q.coeffs()(0);
		q_[1] = q.coeffs()(1);
		q_[2] = q.coeffs()(2);
		q_[3] = q.coeffs()(3);

		if (!simRunning) {
			// Stop the simulation
			this->vrep->stopSim(this->simPort);
		}


		// Set position
		this->vrep->setObjectPosition("_Phantom", p_, "KukaOrigin", simx_opmode_blocking, this->simPort);

		// Wait for a while
		Timer clock;
		clock.timeSleep(0.5);

		// Set orientation
		this->vrep->setObjectQuaternion("_Phantom", q_, "KukaOrigin", simx_opmode_blocking, this->simPort);

	}

}//*/