// Project Header files
#include "HapticInterface.hpp"

/**
* @brief Default constructor of the HapticInterface class
*/
HapticInterface::HapticInterface() : Instrument() {}

/**
* @brief Default constructor of the HapticInterface class
*/
HapticInterface::HapticInterface(const std::string& name_) : Instrument(name_) {}

/**
* @brief Init function
* Check the input pair of comment+value to assign the corresponding parameter
* The function assumes that configFile has been previously set
* @param comment: the string specifying the title comment of parameter
* @param value: the string specifying the value of the parameter
*/
void HapticInterface::loadParamsFromConfigFile(const std::string& comment, const std::string& value) {}
