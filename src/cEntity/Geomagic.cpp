// Project Header files
#include "Geomagic.hpp"
#include "utils.hpp"

/**
* @brief Default constructor of the Geomagic class
*/
Geomagic::Geomagic() : HapticInterface() {

	// Initialize velocity scale
	this->lvelScale = 0.0;
	this->avelScale = 0.0;

	// Initialize Master-slave rotation matrix
	this->Rms.setIdentity(3,3);
	this->fbForce.setZero(3);


}

/**
* @brief Default constructor of the Geomagic class
*/
Geomagic::Geomagic(const std::string& name_) : HapticInterface(name_) {

	// Initialize velocity scale
	this->lvelScale = 0.0;
	this->avelScale = 0.0;
	this->clutchState = false;

	// Initialize Master-slave rotation matrix
	this->Rms.setIdentity(3,3);
	this->fbForce.setZero(3);

}


/**
* @brief Init function
* Check the input pair of comment+value to assign the corresponding parameter
* The function assumes that configFile has been previously set
* @param comment: the string specifying the title comment of parameter
* @param value: the string specifying the value of the parameter
*/
void Geomagic::loadParamsFromConfigFile(const std::string& comment, const std::string& value) {

	if (comment.find("Linear Velocity scale factor") != std::string::npos) {
		this->lvelScale = std::stod(value);
	}
	if (comment.find("Angular Velocity scale factor") != std::string::npos) {
		this->avelScale = std::stod(value);
	}
	else if (comment.find("Force scale factor") != std::string::npos) {
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


