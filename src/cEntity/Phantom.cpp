// Project Header files
#include "Phantom.hpp"
#include "DBWrapper.hpp"
#include "utils.hpp"



/**
* @brief Extract function
* Extract the transformation expressing the pose of the phantom wrt the robot, from the input file content
* @param the content of the file organized as set of string lines
* @return the transformation
*/
Eigen::Matrix4d Phantom::extractTransformation(std::vector <std::string>& content) {

	Eigen::Matrix4d T;
	std::string line;
	std::string comma(",");
	std::string semicolumn(";");
	int commaPos = -1;

	// Initialize points
	int N = content.size();
	T.setIdentity();

	// Extract content
	for (int i = 0; i < content.size(); i++) {

		std::string val[SPACE_DIM+1];
		line = content[i];

		for (int j = 0; j < SPACE_DIM+1; j++) {
			commaPos = line.find(comma);
			val[j] = line.substr(0, commaPos);
			line = line.substr(commaPos + 1, line.length());

			// Update the points structure
			T(i, j) = std::stod(val[j]);
		}

	}

	// Return data
	return T;

}


/**
* @brief Save function
* Save the registered transformation Trp
* @brief filename the name of the file on which save the transformation
*/
void Phantom::saveTransformation(const char* filename){

	// Create a DatabaseWrapper object
	DBWrapper db(filename);
	std::stringstream Tcontent, Pcontent;
	std::string Tcomment("# Transformation computed by Registration");
	Eigen::Matrix4d T;

	// Get the transformation to save
	T = this->Trp;

	// Fill the content of the Transformation file
	Tcontent << Tcomment << std::endl
		<< T(0, 0) << ", " << T(0, 1) << ", " << T(0, 2) << ", " << T(0, 3) << ", " << std::endl
		<< T(1, 0) << ", " << T(1, 1) << ", " << T(1, 2) << ", " << T(1, 3) << ", " << std::endl
		<< T(2, 0) << ", " << T(2, 1) << ", " << T(2, 2) << ", " << T(2, 3) << ", " << std::endl
		<< T(3, 0) << ", " << T(3, 1) << ", " << T(3, 2) << ", " << T(3, 3) << "; ";

	// Save the transformation
	db.write(Tcontent); 

}


/**
* @brief Load function
* Load the registered transformation Trp from the requested file
* @brief filename the name of the file from which load the transformation
*/
void Phantom::loadTransformation(const char* filename) {

	DBWrapper db(filename);
	std::vector < std::string > content;

	// Read the file
	content = db.read();

	// Extract the required data
	this->Trp = this->extractTransformation(content);



}



