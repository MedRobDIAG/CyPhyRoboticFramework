// Project Header files
#include "Logger.hpp"
#include "KUKARobot.hpp"
#include "Geomagic.hpp"
#include "FTSensor.hpp"
#include "Timer.hpp"
#include "utils.hpp"

// System Header file
#include <Windows.h>
#include <string>

#ifdef DEBUG
#include <iostream>
#endif // DEBUG

/**
* @brief Default constructor of the Logger class
*
*/
Logger::Logger() {

	// Initialize class variables
	this->folderPath = std::string("");
	this->startLog = false;
	this->saveLog = false;

	// Initialize pointers
	this->robot = NULL;
	this->haptic = NULL;
	this->ftsensor = NULL;

	// Initialize stringstream objects
	this->qmSS.str("");
	this->qcSS.str("");
	this->qmdotSS.str("");
	this->qcdotSS.str("");
	this->taumsrSS.str("");
	this->TbeeSS.str("");
	this->vbeeSS.str("");
	this->ftmeasSS.str("");
	this->vhapticSS.str("");
	this->resFbeeSS.str("");
	this->resFriFbeeSS.str("");
	this->friGravityVectorSS.str("");
	this->friInertiaMatrixSS.str("");
	this->myResidualSS.str("");
	this->friResidualSS.str("");
	this->friCartFroceSS.str("");

}

/**
* @brief Init function
*
*/
void Logger::init() {

	// Initialize the singleton SystemState and Configuration classes
	// SystemState
	this->systemstate = SystemState::GetInstance();

	// Configuration
	this->config = Configuration::GetInstance(""); // Input filename string is useless here, as the singleton class is supposed to be already instantiated by SystemManager

	std::map< std::string, Instrument* >::iterator robIt, hpIt, ftIt;

	// Take the pointer of the RobotInterface instance
	robIt = this->instruments.find("Robot");
	if (robIt != this->instruments.end()) {
		this->robot = (RobotInterface*)this->instruments["Robot"];
	}

	hpIt = this->instruments.find("Geomagic");
	if (hpIt != this->instruments.end()) {
		this->haptic = (Geomagic*)this->instruments["Geomagic"];
	}

	if (config->isForceSensorRequested()) {
		ftIt = this->instruments.find("FTSensor");
		if (ftIt != this->instruments.end()) {
			this->ftsensor = (FTSensor*)this->instruments["FTSensor"];
		}
	}


	this->exit_loop = false;
	this->started = false;
	this->accomplished = false;
	this->time_ = 0.0;

}


/**
* @brief Set function
* Set the proxies on the current HLInterface object
*/
void Logger::setProxies(std::map < std::string, ExtSystemProxy* >& proxies) {}

/**
* @brief Set function
* Set the entities on the current HLInterface object
*/
void Logger::setInstruments(std::map < std::string, Instrument* >& instruments) {

}

/**
* @brief Run function
*/
void Logger::mainLoop() {

	// Define required variables
	Eigen::Vector7f qc, qm, qcdot, qmdot, taumsr; // suffix: 'c' stands for commanded, 'm' for measured
	Eigen::Matrix4f Tbee;
	Eigen::Vector6f vbee;
	Eigen::Vector6d ftmeas;
	Eigen::Vector6f vhaptic;
	Eigen::Vector7f myResidual;
	Eigen::Vector7f friResidual;
	Eigen::Vector6f friCartForce;
	Eigen::Vector6f resFbee;
	Eigen::Vector6f resFriFbee;
	Eigen::Vector7f friGravityVec;
	Eigen::Matrix7f friInertiaMatrix;
	Eigen::Vector7f modelGravityVec;
	Eigen::Matrix7f modelInertiaMat;
	float detJJT, detJTJ;
	bool clutchState;

	// Time variables
	double tictoc, dt, rate, tic, toc, tac, Ts;

	// Get the clock rate
	Timer clock;
	rate = 200.0;
	clock.setRate(rate);
	//rate = clock.getRate();
	Ts = 1.0 / rate;

	// Initialize stringstream objects
	this->qmSS.str("");
	this->qcSS.str("");
	this->qmdotSS.str("");
	this->qcdotSS.str("");
	this->taumsrSS.str("");
	this->TbeeSS.str("");
	this->vbeeSS.str("");
	this->ftmeasSS.str("");
	this->vhapticSS.str("");
	this->clutchStateSS.str("");
	this->resFbeeSS.str("");
	this->resFriFbeeSS.str("");
	this->friGravityVectorSS.str("");
	this->friInertiaMatrixSS.str("");
	this->myResidualSS.str("");
	this->friResidualSS.str("");
	this->friCartFroceSS.str("");

	int counter = 0;
	while (this->ok()) {
	
		// Measure starting time
		tic = clock.getCurTime();

		//----------------------------------------------------------------//

		if (this->startLog) {


			// Do stuff ...
			/**** Data acquisition ****/

			// Get data from KUKARobot object
			qm = this->robot->getMsrJointPosition();
			qc = this->robot->getCmdJointPosition();
			qcdot = this->robot->getCmdJointVelocity();
			qmdot = this->robot->getMsrJointVelocity();
			taumsr = this->robot->getMsrJointTorques();
			Tbee = this->robot->getTbee();
			vbee = this->robot->getEEVelocity();
			myResidual = this->robot->getResidualVector();
			resFbee = this->robot->getExtContactForce();
			modelGravityVec = this->robot->getModelDynParams().g;
			modelInertiaMat = this->robot->getModelDynParams().B;

			if (this->config->getRobotManipulatorType() == "KUKA") {
				KUKARobot* kuka = dynamic_cast<KUKARobot*>(this->robot);
				friResidual = kuka->getFRIResidual();
				resFriFbee = kuka->getFriResFbee();
				friCartForce = kuka->getFRICartesianForces();
				friGravityVec = kuka->getFRIGravityVec();
				friInertiaMatrix = kuka->getFRIInertiaMatrix();
			}


			// Get data from FTSensor object
			if (config->isForceSensorRequested()) {
				ftmeas = this->ftsensor->getWrench();
				if (ftmeas.norm() > 1e3) ftmeas.setZero();
			}
			else {
				ftmeas.setZero();
			}

			// Get data from Geomagic object
			vhaptic = this->haptic->getHIPVel();
			clutchState = this->haptic->isClutchActive();


			/**** Update stringstream objects ****/

			// measured joint positions
			this->qmSS << time_ << ", ";
			for (int i = 0; i < qm.size(); i++) {
				this->qmSS << qm(i) << ((i == qm.size() - 1) ? "; " : ", ");
			}
			this->qmSS << "\n";

			// commanded joint positions
			this->qcSS << time_ << ", ";
			for (int i = 0; i < qc.size(); i++) {
				this->qcSS << qc(i) << ((i == qc.size() - 1) ? "; " : ", ");
			}
			this->qcSS << "\n";

			// measured joint velocities
			this->qmdotSS << time_ << ", ";
			for (int i = 0; i < qmdot.size(); i++) {
				this->qmdotSS << qmdot(i) << ((i == qmdot.size() - 1) ? "; " : ", ");
			}
			this->qmdotSS << "\n";

			// commanded joint velocities
			this->qcdotSS << time_ << ", ";
			for (int i = 0; i < qcdot.size(); i++) {
				this->qcdotSS << qcdot(i) << ((i == qcdot.size() - 1) ? "; " : ", ");
			}
			this->qcdotSS << "\n";

			// measured joint torques
			this->taumsrSS << time_ << ", ";
			for (int i = 0; i < taumsr.size(); i++) {
				this->taumsrSS << taumsr(i) << ((i == taumsr.size() - 1) ? "; " : ", ");
			}
			this->taumsrSS << "\n";

			// End-effector transformation
			this->TbeeSS << time_ << ", ";
			for (int i = 0; i < SPACE_DIM + 1; i++) {
				for (int j = 0; j < SPACE_DIM + 1; j++) {
					this->TbeeSS << Tbee(i,j) << ((i == SPACE_DIM && j == SPACE_DIM) ? "; " : ", ");
				}
			}
			this->TbeeSS << "\n";

			// End-effector velocity
			this->vbeeSS << time_ << ", ";
			for (int i = 0; i < vbee.size(); i++) {
				this->vbeeSS << vbee(i) << ((i == vbee.size() - 1) ? "; " : ", ");
			}
			this->vbeeSS << "\n";

			// F/T Sensor measurements
			this->ftmeasSS << time_ << ", ";
			for (int i = 0; i < ftmeas.size(); i++) {
				this->ftmeasSS << ftmeas(i) << ((i == ftmeas.size() - 1) ? "; " : ", ");
			}
			this->ftmeasSS << "\n";

			// Haptic velocity
			this->vhapticSS << time_ << ", ";
			for (int i = 0; i < vhaptic.size(); i++) {
				this->vhapticSS << vhaptic(i) << ((i == vhaptic.size() - 1) ? "; " : ", ");
			}
			this->vhapticSS << "\n";

			// Clutch button state
			this->clutchStateSS << time_ << ", " << clutchState << std::endl;

			// Model-based residual
			this->myResidualSS << time_ << ", ";
			for (int i = 0; i < myResidual.size(); i++) {
				this->myResidualSS << myResidual(i) << ((i == myResidual.size() - 1) ? "; " : ", ");
			}
			this->myResidualSS << "\n";

			// FRI residual
			this->friResidualSS << time_ << ", ";
			for (int i = 0; i < friResidual.size(); i++) {
				this->friResidualSS << friResidual(i) << ((i == friResidual.size() - 1) ? "; " : ", ");
			}
			this->friResidualSS << "\n";

			// Cartesian Force reconstructed from model-based residual
			this->resFbeeSS << time_ << ", ";
			for (int i = 0; i < resFbee.size(); i++) {
				this->resFbeeSS << resFbee(i) << ((i == resFbee.size() - 1) ? "; " : ", ");
			}
			this->resFbeeSS << "\n";

			// Cartesian Force reconstructed from FRI residual
			this->resFriFbeeSS << time_ << ", ";
			for (int i = 0; i < resFriFbee.size(); i++) {
				this->resFriFbeeSS << resFriFbee(i) << ((i == resFriFbee.size() - 1) ? "; " : ", ");
			}
			this->resFriFbeeSS << "\n";

			// Cartesian Force computed by FRI 
			this->friCartFroceSS << time_ << ", ";
			for (int i = 0; i < friCartForce.size(); i++) {
				this->friCartFroceSS << friCartForce(i) << ((i == friCartForce.size() - 1) ? "; " : ", ");
			}
			this->friCartFroceSS << "\n";

			// Gravity vector computed by FRI 
			this->friGravityVectorSS << time_ << ", ";
			for (int i = 0; i < friGravityVec.size(); i++) {
				this->friGravityVectorSS << friGravityVec(i) << ((i == friGravityVec.size() - 1) ? "; " : ", ");
			}
			this->friGravityVectorSS << "\n";

			// Inertia matrix computed by FRI
			this->friInertiaMatrixSS << time_ << ", ";
			for (int i = 0; i < KUKA_JOINT_NUM; i++) {
				for (int j = 0; j < KUKA_JOINT_NUM; j++) {
					this->friInertiaMatrixSS << friInertiaMatrix(i, j) << ((i == KUKA_JOINT_NUM -1 && j == KUKA_JOINT_NUM -1) ? "; " : ", ");
				}
			}
			this->friInertiaMatrixSS << "\n";



			// Gravity vector computed by the model
			this->modelGravityVectorSS << time_ << ", ";
			for (int i = 0; i < modelGravityVec.size(); i++) {
				this->modelGravityVectorSS << modelGravityVec(i) << ((i == modelGravityVec.size() - 1) ? "; " : ", ");
			}
			this->modelGravityVectorSS << "\n";

			// Inertia matrix computed by the model
			this->modelInertiaMatrixSS << time_ << ", ";
			for (int i = 0; i < KUKA_JOINT_NUM; i++) {
				for (int j = 0; j < KUKA_JOINT_NUM; j++) {
					this->modelInertiaMatrixSS << modelInertiaMat(i, j) << ((i == KUKA_JOINT_NUM - 1 && j == KUKA_JOINT_NUM - 1) ? "; " : ", ");
				}
			}
			this->modelInertiaMatrixSS << "\n";


			// Determinant of the Jacobian matrix
			//this->detJJTSS << time_ << ", " << detJJT << "; " << std::endl;
			//this->detJTJSS << time_ << ", " << detJTJ << "; " << std::endl;

			// If saveLog flag is raised, save all the data acquired so far and reset the logger
			if(this->saveLog){
				this->saveLogsAndReset();
			}

		}

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
		
		if (this->startLog) {
			//this->time_ += dt;
		}

		//debugPrint<double>("[Logger] Running rate:", 1.0 / dt);
	}
	



}


/**
* @brief Save request function
* Set the saveLog flag to request saving the data acquired so far
*/
void Logger::requestSaveAndWait() {

	Timer clock;

	this->saveLog = true;

	while (this->startLog) {
		clock.timeSleep(0.1);
	}

}


/**
* @brief Save and close function
* Save the data acquired so far on the corresponding files, and reset the logger
*/
void Logger::saveLogsAndReset() {

	if (this->startLog) {
		/*** Save logs ***/
		std::string robotPath = this->folderPath + "\\" + "KUKARobot" + "\\";
		std::string hapticPath = this->folderPath + "\\" + "Geomagic" + "\\";
		std::string ftPath = this->folderPath + "\\" + "FTSensor" + "\\";

		// Robot
		this->robot->saveCSVFile(this->qmSS, robotPath.c_str(), "qmsr.txt");
		this->robot->saveCSVFile(this->qcSS, robotPath.c_str(), "qcmd.txt");
		this->robot->saveCSVFile(this->qmdotSS, robotPath.c_str(), "qdotmsr.txt");
		this->robot->saveCSVFile(this->qcdotSS, robotPath.c_str(), "qdotcmd.txt");
		this->robot->saveCSVFile(this->taumsrSS, robotPath.c_str(), "taumsr.txt");
		this->robot->saveCSVFile(this->TbeeSS, robotPath.c_str(), "Tbee.txt");
		this->robot->saveCSVFile(this->vbeeSS, robotPath.c_str(), "vbee.txt");
		this->robot->saveCSVFile(this->myResidualSS, robotPath.c_str(), "modelResidual.txt");
		this->robot->saveCSVFile(this->friResidualSS, robotPath.c_str(), "friResidual.txt");
		this->robot->saveCSVFile(this->resFbeeSS, robotPath.c_str(), "residualFbee.txt");
		this->robot->saveCSVFile(this->resFriFbeeSS, robotPath.c_str(), "residualFRIFbee.txt");
		this->robot->saveCSVFile(this->friCartFroceSS, robotPath.c_str(), "FRICartesianForce.txt");
		this->robot->saveCSVFile(this->friGravityVectorSS, robotPath.c_str(), "FRIGravityVector.txt");
		this->robot->saveCSVFile(this->friInertiaMatrixSS, robotPath.c_str(), "FRIInertiaMatrix.txt");
		this->robot->saveCSVFile(this->modelGravityVectorSS, robotPath.c_str(), "modelGravityVector.txt");
		this->robot->saveCSVFile(this->modelInertiaMatrixSS, robotPath.c_str(), "modelInertiaMatrix.txt");
		this->robot->saveCSVFile(this->detJJTSS, robotPath.c_str(), "detJJT.txt");
		this->robot->saveCSVFile(this->detJTJSS, robotPath.c_str(), "detJTJ.txt");

		// Geomagic
		this->haptic->saveCSVFile(this->vhapticSS, hapticPath.c_str(), "velHaptic.txt");
		this->haptic->saveCSVFile(this->clutchStateSS, hapticPath.c_str(), "clutchState.txt");

		// FT sensor
		if (config->isForceSensorRequested()) {
			this->ftsensor->saveCSVFile(this->ftmeasSS, ftPath.c_str(), "FTmeas.txt");
		}

		this->systemstate->lockKBMtx();

		// Ask for notes to save
		std::cout << "Type a note on this log session: (press ENTER to confirm)" << std::endl;
		std::stringstream note;
		std::string input("");

		std::cin.ignore();
		std::getline(std::cin, input);

		std::cout << "note inserted: " << input << std::endl;
		note = std::stringstream(input);

		this->systemstate->unlockKBMtx();


		// Choose any arbitrary entity to store the note file
		this->robot->saveCSVFile(note, this->folderPath.c_str(), "\\ LogNotes.txt");
	}

	// Reset startLog and time_
	this->stop();


}


/**
* @brief Clear function
*/
void Logger::clear() {

}



/**
* @brief Create function
* Create the logging folder
* @return the path of the logging folder
*/
std::string Logger::createMainFolder() {

	char buf[256];
	int sessionNum = 0;
	bool folderCreated = false;
	std::string folderName = std::string("LogSession_");
	std::string candidateName;
	
	// Get the current program directory
	GetCurrentDirectoryA(256, buf);

	// Iterate to create a folder with a not pre-existing name (e.g., from previous logs)
	while (folderCreated == false) {
	
		candidateName = std::string(buf) + "\\" + folderName + std::to_string(sessionNum);
		sessionNum++;


		// Try to create the candidate folder
		folderCreated = CreateDirectory(candidateName.c_str(), NULL);

		if (folderCreated) {
			std::cout << "Log Folder #" << sessionNum << " created. " << std::endl;
		}
	}
	
	// Assign the path to the class memeber variable
	this->folderPath = candidateName;

	// Create subfolders
	this->createFolder("KUKARobot");
	this->createFolder("Geomagic");
	this->createFolder("FTSensor");
	this->createFolder("RLS");

	// Return the log folder path
	return this->folderPath;

}



/**
* @brief Create function
* Create a generic folder with the name specified by folderName, in the path folderPath
* @param folderName the name of the folder to be created
* @return 1 if the folder has been created, 0 if it already exists and -1 in case of error
*/
int Logger::createFolder(const char* folderName) {

	int ret = -1;
	std::string fullFolderPath = std::string(this->folderPath) + "\\" + std::string(folderName);

	if (!CreateDirectory(fullFolderPath.c_str(), NULL)) {
		ret = 1;
	}

	// Return
	return ret;


}
