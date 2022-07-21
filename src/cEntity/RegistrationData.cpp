// Projet Header files
#include "RegistrationData.hpp"
#include "DBWrapper.hpp"

#ifdef DEBUG
#include <iostream>
#endif //DEBUG

/**
* @brief Constructor of the RegistrationData Class
*/
RegistrationData::RegistrationData(){

	this->T.setIdentity();

}


/**
* @brief Default destroyer of the RegistrationData Class
*/
RegistrationData::~RegistrationData(){}

/**
* @brief Load the registered transformation from a file
* @param filename: the name of the file containing the transformation of the sensor wrt the CT scan
* @return true if the transformation has been loaded. False otherwise
*/
bool RegistrationData::loadTransformation(const char* filename){

	bool loaded;
	DBWrapper db(filename);
	std::vector < std::string > content;

	// Read the file
	content = db.read();

	// Extract the required data
	loaded = this->extractTransformData(content);

	return loaded;

}

/**
* @brief Load the CT point coordinates from a file
* @param filename: the name of the file containing the 3D point coordinates in the CT scan frame
* @return true if the data have been loaded. False otherwise
*/
bool RegistrationData::loadGTPoints(const char* filename) {

	bool loaded;
	DBWrapper db(filename);
	std::vector < std::string > content;

	// Read the file
	content = db.read();

	// Extract the CT points data
	loaded = this->extractGTPointData(content);

	return loaded;

}


/**
* @brief Load the Polaris point coordinates from a file
* @param filename: the name of the file containing the 3D point coordinates in the Polaris frame
* @return true if the data have been loaded. False otherwise
*/
bool RegistrationData::loadPolarisPoints(const char* filename) {

	bool loaded, success;
	DBWrapper db(filename);
	std::vector < std::string > content;

	// Read the file
	content = db.read();

	// Extract the CT points data
	loaded = this->extractPolarisPointData(content);

	// Return boolean flag
	return (content[0] != "ERROR");


}



/**
* @brief Load the coordinates of the probe tip from a file
* @param filename: the name of the file containing the coordinates of the probe tip
* @return true if the data have been loaded. False otherwise
*/
bool RegistrationData::loadProbeTip(const char* filename) {

	bool loaded;
	DBWrapper db(filename);
	std::vector < std::string > content;

	// Read the file
	content = db.read();

	// Extract the CT points data
	loaded = this->extractProbeTipData(content);

	return loaded;

}



/**
* @brief Extract data function
* Extract the expected data from the content of the registered transformation file to save data properly (set internally Tctp)
* @param content: the vector of strings with the content of the read file
*/
bool RegistrationData::extractTransformData(const std::vector < std::string >& content){

	std::string line;
	std::string comma(",");
	std::string semicolumn(";");
	int commaPos = -1;

	if (content.size() != 4){
#ifdef DEBUG
		std::cout << "ERROR: Wrong numbers of lines detected in the Registered Transformation file." << std::endl;
#endif //DEBUG
		return false;
	}

	for (int i = 0; i < content.size(); i++){

		std::string val[4];
		line = content[i];

		for (int j = 0; j < 4; j++){
			commaPos = line.find(comma);
			val[j] = line.substr(0, commaPos);
			line = line.substr(commaPos + 1, line.length());

			this->T(i, j) = std::stod(val[j]);
		}

	}

#ifdef DEBUG
	std::cout << "this->Tctp: \n" << this->T << std::endl;
#endif //DEBUG


	return true;
}


/**
* @brief Extract data function
* Extract the expected data from the content of the CT point file to save data properly (set internally CTpoints)
* @param content: the vector of strings with the content of the read file
*/
bool RegistrationData::extractGTPointData(const std::vector < std::string >& content) {


	std::string line;
	std::string comma(",");
	std::string semicolumn(";");
	int commaPos = -1;

	// Take the number of lines
	int N = content.size();

	if (content.size() < 3 || content[0] == "ERROR") {
#ifdef DEBUG
		std::cout << "ERROR: Wrong numbers of lines detected in the GT Points file." << std::endl;
#endif //DEBUG
		return false;
	}

	// Initialize the CTPoints object
	this->GTPoints.setZero(N, COORDINATE_SIZE);

	for (int i = 0; i < content.size(); i++) {

		std::string val[COORDINATE_SIZE];
		line = content[i];

		for (int j = 0; j < COORDINATE_SIZE; j++) {
			commaPos = line.find(comma);
			val[j] = line.substr(0, commaPos);
			line = line.substr(commaPos + 1, line.length());
			this->GTPoints(i, j) = std::stod(val[j]);
		}

	}

#ifdef DEBUG
	std::cout << "this->GTPoints: \n" << this->GTPoints << std::endl;
#endif //DEBUG


	return true;


}



/**
* @brief Extract data function
* Extract the expected data from the content of the Polaris point file to save data properly (set internally PolarisPoints)
* @param content: the vector of strings with the content of the read file
*/
bool RegistrationData::extractPolarisPointData(const std::vector < std::string >& content) {

	std::string line;
	std::string comma(",");
	std::string semicolumn(";");
	int commaPos = -1;

	// Take the number of lines
	int N = content.size();

	if (content.size() < 3 || content[0] == "ERROR") {
#ifdef DEBUG
		std::cout << "ERROR: Wrong numbers of lines detected in the CT Points file." << std::endl;
#endif //DEBUG
		return false;
	}

	// Initialize the CTPoints object
	this->polarisPoints.setZero(N, COORDINATE_SIZE);

	for (int i = 0; i < content.size(); i++) {

		std::string val[COORDINATE_SIZE];
		line = content[i];

		for (int j = 0; j < COORDINATE_SIZE; j++) {
			commaPos = line.find(comma);
			val[j] = line.substr(0, commaPos);
			line = line.substr(commaPos + 1, line.length());

			this->polarisPoints(i, j) = std::stod(val[j]);
		}

	}

#ifdef DEBUG
	std::cout << "this->polarisPoints: \n" << this->polarisPoints << std::endl;
#endif //DEBUG


	return true;

}



/**
* @brief Extract data function
* Extract the expected data from the content of the probe tip coordinates file to save data properly (set internally CTpoints)
* @param content: the vector of strings with the content of the read file
*/
bool RegistrationData::extractProbeTipData(const std::vector < std::string >& content) {


	std::string line;
	std::string comma(",");
	std::string semicolumn(";");
	int commaPos = -1;

	if (content.size() != 1) {
#ifdef DEBUG
		std::cout << "ERROR: Wrong numbers of lines detected in the Probe Tip coordinate file." << std::endl;
#endif //DEBUG
		return false;
	}

	for (int i = 0; i < content.size(); i++) {

		std::string val[3];
		line = content[i];

		for (int j = 0; j < 3; j++) {
			commaPos = line.find(comma);
			val[j] = line.substr(0, commaPos);
			line = line.substr(commaPos + 1, line.length());

			this->probeTip(j) = std::stod(val[j]);
		}

	}

#ifdef DEBUG
	std::cout << "this->probeTip: \n" << this->probeTip << std::endl;
#endif //DEBUG


	return true;


}

/**
* @brief Conversion function
* Convert the vector of point coordinates provided in input to the corresponding Eigen matrix
* @param the landmark point coordinates in vector structure
*/
void RegistrationData::populatePolarisPoints(const std::vector < Eigen::Vector3d >& points) {

	// Size of the acquired points
	int N = points.size();

	// Initialize the polarisPoints object
	this->polarisPoints.setZero(N, COORDINATE_SIZE);

	// Fill the Eigen structure
	for (int i = 0; i < N; i++) {
		this->polarisPoints.row(i) = points[i];
	}

#ifdef DEBUG
	std::cout << "this->polarisPoints: " << this->polarisPoints << std::endl;
#endif // DEBUG

}

/**
* Save function
* Save the homogeneous transformation matrix on a file
* params the name of the file on which the transformation has to be saved
* params the name of the file on which the Polaris points coordinates have to be saved
* @return true if the Transformation has been saved
*/
bool RegistrationData::saveTransformation(const char* Tfilename, const char* Pfilename){



	bool Tsaved, Psaved;
	DBWrapper db;
	std::stringstream Tcontent, Pcontent;
	std::string Tcomment("# Transformation computed by Registration");
	std::string Pcomment("# 3D Point coordinates of the landmarks in the Polaris frame");
	Eigen::Matrix4d T;
	Eigen::MatrixXd pPoints = this->polarisPoints;

	// Get the transformation to save
	T = this->T;

	// Fill the content of the Transformation file
	Tcontent << Tcomment << std::endl
		<< T(0, 0) << ", " << T(0, 1) << ", " << T(0, 2) << ", " << T(0, 3) << ", " << std::endl
		<< T(1, 0) << ", " << T(1, 1) << ", " << T(1, 2) << ", " << T(1, 3) << ", " << std::endl
		<< T(2, 0) << ", " << T(2, 1) << ", " << T(2, 2) << ", " << T(2, 3) << ", " << std::endl
		<< T(3, 0) << ", " << T(3, 1) << ", " << T(3, 2) << ", " << T(3, 3) << "; ";

	// Fill the content of the polaris points file
	Pcontent << Pcomment << std::endl
		<< pPoints(0, 0) << ", " << pPoints(0, 1) << ", " << pPoints(0, 2) << ", " << std::endl
		<< pPoints(1, 0) << ", " << pPoints(1, 1) << ", " << pPoints(1, 2) << ", " << std::endl
		<< pPoints(2, 0) << ", " << pPoints(2, 1) << ", " << pPoints(2, 2) << ", " << std::endl
		<< pPoints(3, 0) << ", " << pPoints(3, 1) << ", " << pPoints(3, 2) << "; ";

	// Save the transformation
	db.setDBFile(Tfilename);
	Tsaved = db.write(Tcontent);

	// Save the polaris points
	db.setDBFile(Pfilename);
	Psaved = db.write(Pcontent);

	return Tsaved && Psaved;

}


/**
* Save function
* Save the calibration stats on a file
* params the name of the file on which the calibration stats have to be saved
*/
void RegistrationData::saveStats(const char* filename) {

	DBWrapper db(filename);
	std::stringstream content;

	for (int i = 0; i < this->stats.size(); i++) {
	
		content << this->stats(i) << "; " << std::endl;

	}

	// Save the transformation
	db.write(content);


}

