// Project Header files
#include "EndEffector.hpp"
#include "DBWrapper.hpp"
#include "utils.hpp"

/**
* @brief Init function 
* Check the input pair of comment+value to assign the corresponding parameter
* The function assumes that configFile has been previously set
* @param comment: the string specifying the title comment of parameter
* @param value: the string specifying the value of the parameter
*/
void EndEffector::loadParamsFromConfigFile(const std::string& comment, const std::string& value) {

	if (comment.find("Size") != std::string::npos) {
		std::vector < double > vec = parseCSVLine(value);
		for (int j = 0; j < vec.size(); j++) {
			this->size[j] = vec[j];
		}
	}
	if (comment.find("CoM") != std::string::npos) {
		std::vector < double > vec = parseCSVLine(value);
		for (int j = 0; j < vec.size(); j++) {
			this->CoM[j] = vec[j];
		}
	}
	else if (comment.find("Weight") != std::string::npos) {
		this->weight = std::stod(value);
	}

}
