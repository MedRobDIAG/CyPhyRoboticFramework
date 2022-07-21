#ifndef PHANTOM_HPP_
#define PHANTOM_HPP_

// Project Header files
#include "Instrument.hpp"

// Standard Header files
#include <vector>

// Eigen Header files
#include <Eigen/Dense>

class Phantom : public Instrument {


public:

	/**
	* @brief Default constructor of Phantom class
	*
	*/
	Phantom() : Instrument() {}

	/**
	* @brief constructor of Phantom class with name argument
	* @param name_ the name of the instrument
	*/
	Phantom(const std::string& name_) : Instrument(name_) {}

	/**
	* @brief Default destroyer of Phantom class
	*
	*/
	~Phantom(){}

	/**
	* @brief Init function
	* Check the input pair of comment+value to assign the corresponding parameter
	* The function assumes that configFile has been previously set
	* @param comment: the string specifying the title comment of parameter
	* @param value: the string specifying the value of the parameter
	*/
	virtual void loadParamsFromConfigFile(const std::string& comment, const std::string& value) = 0;

	/**
	* @brief Extract function
	* Extract the transformation expressing the pose of the phantom wrt the robot, from the input file content
	* @param the content of the file organized as set of string lines
	* @return the transformation
	*/
	Eigen::Matrix4d extractTransformation(std::vector <std::string>& content);

	/**
	* @brief Get function
	* Retrievee the Homogeneous transformation matrix expressing the pose of the phantom in the robot frame
	* @return the transformation matrix
	*/
	Eigen::Matrix4d getTransformation() { return this->Trp; }

	/**
	* @brief Set function
	* Set the Homogeneous transformation matrix expressing the pose of the phantom in the robot frame
	* @param T the transformation matrix to be set
	*/
	void setTransformation(const Eigen::Matrix4d& T) { this->Trp = T; }

	/**
	* @brief Save function
	* Save the registered transformation Trp on the requested file
	* @brief filename the name of the file on which save the transformation
	*/
	void saveTransformation(const char* filename);

	/**
	* @brief Load function
	* Load the registered transformation Trp from the requested file
	* @brief filename the name of the file from which load the transformation
	*/
	void loadTransformation(const char* filename);


protected:

	Eigen::Matrix4d Trp;			//!< Homogeneous transformation matrix expressing the pose of the phantom in the robot frame
};

#endif // PHANTOM_HPP_
