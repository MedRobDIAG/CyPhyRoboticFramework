// Project Header files
#include "Armband.hpp"
#include "utils.hpp"
// Standard Header files
#include <vector>


/**
* @brief Default constructor of the Armband class
*/
Armband::Armband() : HapticInterface() {


	this->Rms.setIdentity(3, 3);
	this->fbForce.setZero(3);

}


/**
* @brief Default constructor of the Armband class
*/
Armband::Armband(const std::string& name_) : HapticInterface(name_) {

	this->Rms.setIdentity(3, 3);
	this->fbForce.setZero(3);

}


/**
* @brief Init function
* Check the input pair of comment+value to assign the corresponding parameter
* The function assumes that configFile has been previously set
* @param comment: the string specifying the title comment of parameter
* @param value: the string specifying the value of the parameter
*/
void Armband::loadParamsFromConfigFile(const std::string& comment, const std::string& value) {

	if (comment.find("Force scale factor") != std::string::npos) {
		this->force_scale = std::stod(value);
	}
	else if (comment.find("Master-Slave rotation matrix (row-wise)") != std::string::npos) {

		std::vector < double > vec = parseCSVLine(value);
		this->Rms.setIdentity(3, 3);
		for (int j = 0; j < vec.size(); j++) {
			this->Rms(j) = (float)vec[j];
		}

		// Since the vector-based index queries the matrix columnwise, 
		// transpose the resulting matrix to get the correct matrix from file
		this->Rms.transposeInPlace();
	}
	else if (comment.find("Enable force feedback") != std::string::npos) {
		this->useForceFeedback = (std::stod(value)) ? true : false;
	}
}

