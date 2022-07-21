// Project Header files
#include "Configuration.hpp"
#include "DBWrapper.hpp"
#include "utils.hpp"

// Standard Header files
#include <string>
#include <cctype>
#include <algorithm>


/**
* @brief Load function
* Load the configuration parameters from a file
*/
void Configuration::loadFromFile() {

	// Create the structure to store the content of the file
	std::vector < std::pair < std::string, std::string > > content;

	// Create the static instance to access the file system
	DBWrapper db(this->filename.c_str());

	// Read and parse the files in the convention <comment, value>
	content = db.readLabeledFile();

	this->parseFileContent(content);
}

/**
* @brief Save function
* Save the configuration parameters on a file
* @param filename the name of the file that has to the configuration parameters
*/
void Configuration::saveToFile(const char* filename) {}

/**
* @ Parse function
* Parse the content read by the lower layers and assign the read values to the corresponding variables
* @param content the preliminary parsed content of the configuration file
* @return true if data are ok
*/
bool Configuration::parseFileContent(std::vector < std::pair < std::string, std::string > >& content) {

	debugPrint<char>("Starting parsing file content ...", NULL);

	// Check if input data are ok to be processed
	if (content.size() == 1 && content[0].first == "ERROR") {
		return false;
	}

	for (int i = 0; i < content.size(); i++) {
		std::string comment = content[i].first;
		std::string value = content[i].second;

		if (comment.find("Haptic devices List") != std::string::npos) {
			std::vector < std::string > vec = parseStringLine(value);
			this->hapticDeviceList.resize(vec.size());
			for (int j = 0; j < vec.size(); j++) {
				this->hapticDeviceList[j] = vec[j];
				debugPrint<std::string>("j-th haptic device:", this->hapticDeviceList[j]);
			}
			this->hapticDeviceType = this->hapticDeviceList[0];
		}
		else if (comment.find("Haptic device") != std::string::npos) {
			this->hapticDeviceType = value;
		}
		else if (comment.find("Teleoperation device") != std::string::npos) {
			this->teleopDeviceType = value;
		}
		else if (comment.find("Robot manipulator") != std::string::npos) {
			this->robotManipulatorType = value;
		}
		else if (comment.find("Simulator Software") != std::string::npos) {
			this->simulatorType = value;
		}
		else if (comment.find("Phantom type") != std::string::npos) {
			this->phantomType = value;
		}
		else if (comment.find("Keyboard Shared Memory Name") != std::string::npos) {
			this->kbShdMemName = value;
		}
		else if (comment.find("Log data") != std::string::npos) {
			this->logData = (std::stod(value)) ? true : false;
		}
		else if (comment.find("Initial control mode") != std::string::npos) {
			this->controlMode = std::stod(value);
		}
		else if (comment.find("Online Processing") != std::string::npos) {
			this->onlineProcessing = (std::stod(value)) ? true : false;
		}
		else if (comment.find("Offline dataset log number") != std::string::npos) {
			this->offLogDataNum = std::stod(value);
		}//*/
		else if (comment.find("Phantom points filename") != std::string::npos) {
			this->phantomPtsFile = value;
		}
		else if (comment.find("Use registered Phantom") != std::string::npos) {
			this->withRegisteredPhantom = (std::stod(value)) ? true : false;
		}
		else if (comment.find("Phantom transformation file name") != std::string::npos) {
			this->regTransfFile = value;
		}
		else if (comment.find("Batch or recursive") != std::string::npos) {
			this->fricEstRecursive = (std::stod(value)) ? true : false;
		}
		else if (comment.find("Estimate gravity") != std::string::npos) {
			this->estimateGravity = (std::stod(value)) ? true : false;
		}
		/*else if (comment.find("Online estimation") != std::string::npos) {
			this->fricEstimationOnline = (std::stod(value)) ? true : false;
		}//*/
		/*else if (comment.find("Offline estimation Log number") != std::string::npos) {
		this->fricEstLogDataNum = std::stod(value);
		}//*/
		else if (comment.find("Friction data size") != std::string::npos) {
			this->fricEstStateData = std::stod(value);
		}
		else if (comment.find("G-N Batch number of iterations") != std::string::npos) {
			this->fricEstIterationsNum = std::stod(value);
		}
		else if (comment.find("Friction parameters filename") != std::string::npos) {
			this->FrictionParamFile = value;
		}
		else if (comment.find("Ground Truth Force data") != std::string::npos) {
			this->rniGTForceData = std::stod(value);
		}
		/*else if (comment.find("Online identification") != std::string::npos) {
			this->onlineIdentification = (std::stod(value)) ? true : false;
		}//*/
		/*else if (comment.find("Offline identification Log number") != std::string::npos) {
			this->rniLogDataNum = std::stod(value);
		}//*/
		else if (comment.find("Forgetting factor") != std::string::npos) {
			this->rniLambda = std::stod(value);
		}
		else if (comment.find("Interaction model") != std::string::npos) {
			this->rniInteractionModel = std::stod(value);
		}
		else if (comment.find("Use 3D model information") != std::string::npos) {
			this->rniWith3DModelDepths = (std::stod(value)) ? true : false;
		}
		else if (comment.find("Force lowpass filter coefficient") != std::string::npos) {
			this->rniForceLPFcoeff = std::stod(value);
		}
		else if (comment.find("Force derivative lowpass filter coefficient") != std::string::npos) {
			this->rniForceDerLPFcoeff = std::stod(value);
		}
		else if (comment.find("Force derivative contact threshold") != std::string::npos) {
			this->rniForceDerThreshold = std::stod(value);
		}
		else if (comment.find("Covariance reset threshold ratio") != std::string::npos) {
			this->rniCRratio = std::stod(value);
		}
		else if (comment.find("Covariance reset frequency") != std::string::npos) {
			this->rniCRfrequency = std::stod(value);
		}
		else if (comment.find("Tissue rupture window size") != std::string::npos) {
			this->rniRuptureWinSize = std::stod(value);
		}
		else if (comment.find("A posteriori compensation") != std::string::npos) {
			this->rniPostCompensation = std::stod(value);
		}
		else if (comment.find("sigma0") != std::string::npos) {
			this->rniSigma0 = std::stod(value);
		}
		else if (comment.find("sigma1") != std::string::npos) {
			this->rniSigma1 = std::stod(value);
		}
		else if (comment.find("Initial variances") != std::string::npos) {
			std::vector < std::string > vec = parseStringLine(value);
			this->rniInitVariances.resize(vec.size());
			for (int j = 0; j < vec.size(); j++) {
				this->rniInitVariances[j] = std::stod(vec[j]);
			}
		}
		else if (comment.find("Force feedback source") != std::string::npos) {
			this->ffbType = std::stod(value);
		}
		else if (comment.find("Tracking Gain") != std::string::npos) {
			this->ac_gain = std::stod(value);
		}
		else if (comment.find("Trajectory Type") != std::string::npos) {
			this->trajType = std::stod(value);
		}
		else if (comment.find("Residual threshold") != std::string::npos) {
			this->mg_resThresh = std::stod(value);
		}
		else if (comment.find("Gains vector") != std::string::npos) {
			std::vector < std::string > vec = parseStringLine(value);
			this->mg_gains.resize(vec.size());
			for (int j = 0; j < vec.size(); j++) {
				this->mg_gains[j] = std::stod(vec[j]);
			}
		}
		else if (comment.find("Filter residual") != std::string::npos) {
			this->mg_applyFilterOnResidual = (std::stod(value)) ? true : false;
		}
		else if (comment.find("Low-pass cut-off frequency") != std::string::npos) {
			this->mg_fcut = std::stod(value);
		}
		else if (comment.find("Use V-REP") != std::string::npos) {
			this->withSimulator = (std::stod(value)) ? true : false;
		}
		else if (comment.find("V-REP IP address") != std::string::npos) {
			this->vrepIPAddress = value;
		}
		else if (comment.find("Dynamic simulation") != std::string::npos) {
			this->useDynamicEngine = (std::stod(value)) ? true : false;
		}
		else if (comment.find("V-REP Object names list") != std::string::npos) {
			std::vector < std::string > vec = parseStringLine(value);
			this->vrepObjects.resize(vec.size());
			for (int j = 0; j < vec.size(); j++) {
				this->vrepObjects[j] = vec[j];
				//debugPrint<std::string>("j-th vrep object:", this->vrepObjects[j]);
			}
		}
		else if (comment.find("Real robot") != std::string::npos) {
			this->withRealRobot = (std::stod(value)) ? true : false;
		}
		else if (comment.find("Use F/T Sensor") != std::string::npos) {
			this->withForceSensor = (std::stod(value)) ? true : false;
		}
		else if (comment.find("Real or simulated sensor") != std::string::npos) {
			this->simulateFTSensor = (std::stod(value)) ? true : false;
		}
		else if (comment.find("Mount end-effector") != std::string::npos) {
			this->withEE = (std::stod(value)) ? true : false;
		}
		else if (comment.find("Robot config filename") != std::string::npos) {
			this->robotConfig = value;
		}
		else if (comment.find("Geomagic config filename") != std::string::npos) {
			this->geomagicConfig = value;
		}
		else if (comment.find("F/T Sensor config filename") != std::string::npos) {
			this->ftSensorConfig = value;
		}
		else if (comment.find("end-effector config filename") != std::string::npos) {
		this->endEffectorConfig = value;
		}
		else if (comment.find("Phantom config filename") != std::string::npos) {
		this->phantomConfig = value;
		}


	}


	return true;
}

