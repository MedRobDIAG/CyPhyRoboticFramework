// Project Header files
#include "HRing.hpp"
#include "utils.hpp"

/**
* @brief Default constructor of the HRing class
*/
HRing::HRing() : HapticInterface() {
	this->fbForce.setZero(2);
	this->Rms.setZero(2, 3);
}

/**
* @brief Default constructor of the HRing class
*/
HRing::HRing(const std::string& name_) : HapticInterface(name_) {
	this->fbForce.setZero(2);
	this->Rms.setZero(2, 3);
}

/**
* @brief Default destroyer of the HRing class
*/
HRing::~HRing() {}

void HRing::loadParamsFromConfigFile(const std::string& comment, const std::string& value) {

	if (comment.find("Force scale factor") != std::string::npos) {
		this->force_scale = std::stod(value);
	}
	else if (comment.find("Master-Slave rotation matrix (row-wise)") != std::string::npos) {

		std::vector < double > vec = parseCSVLine(value);
		int cols = 3;
		int rows = vec.size() / cols;
		this->Rms.setZero(rows, cols);
		for (int j = 0; j < vec.size(); j++) {
			this->Rms(j) = (float)vec[j];
		}

		std::cout << "Rms HRing = \n" << this->Rms << std::endl;

	}
	else if (comment.find("Enable force feedback") != std::string::npos) {
		this->useForceFeedback = (std::stod(value)) ? true : false;
	}



}
