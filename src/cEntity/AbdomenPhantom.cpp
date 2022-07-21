// Project Header files
#include "AbdomenPhantom.hpp"
#include "DBWrapper.hpp"
#include "utils.hpp"

/**
* @brief Default constructor of AbdomenPhantom class
*
*/
AbdomenPhantom::AbdomenPhantom() : Phantom() {}

/**
* @brief constructor of AbdomenPhantom class with name argument
* @param name_ the name of the instrument
*/
AbdomenPhantom::AbdomenPhantom(const std::string& name_) : Phantom(name_) {}

/**
* @brief Default destroyer of AbdomenPhantom class
*
*/
AbdomenPhantom::~AbdomenPhantom() {}

/**
* @brief Init function
* Check the input pair of comment+value to assign the corresponding parameter
* The function assumes that configFile has been previously set
* @param comment: the string specifying the title comment of parameter
* @param value: the string specifying the value of the parameter
*/
void AbdomenPhantom::loadParamsFromConfigFile(const std::string& comment, const std::string& value) {}

/**
* @brief Load function
* Load the radiopaque points from the file with the given name
* @param filename the name of the file to be loaded
* @return the radiopaque points
*/
Eigen::MatrixXd AbdomenPhantom::loadRadioPts(const char* filename) {

	// Create the structure to store the content of the file
	std::vector < std::string > content;

	// Create the static instance to access the file system
	DBWrapper db(filename);

	// Read and parse the files in the convention <comment, value>
	content = db.readCSVFile();

	// Convert read data
	Eigen::MatrixXd phantomPts;
	phantomPts = this->extractRadioPts(content);
	this->radioPts = phantomPts;

	// Return data
	return phantomPts;

}

/**
* @brief Extract function
* Extract the point coordinates of the phantom from the input file content
* @param the content of the file organized as set of string lines
* @return the set of point coordinates of the phantom radiopaque points
*/
Eigen::MatrixXd AbdomenPhantom::extractRadioPts(std::vector < std::string >& content) {

	Eigen::MatrixXd points;
	std::string line;
	std::string comma(",");
	std::string semicolumn(";");
	int commaPos = -1;

	// Initialize points
	int N = content.size();
	points.setZero(N, SPACE_DIM);

	// Extract content
	for (int i = 0; i < content.size(); i++) {

		std::string val[SPACE_DIM];
		line = content[i];

		for (int j = 0; j < SPACE_DIM; j++) {
			commaPos = line.find(comma);
			val[j] = line.substr(0, commaPos);
			line = line.substr(commaPos + 1, line.length());

			// Update the points structure
			points(i, j) = std::stod(val[j]);
		}

	}

	// Return data
	return points;


}

/**
* @brief Save function
* Save the acquired points on the requested file
* @brief filename the name of the file on which save the acquired points
*/
void AbdomenPhantom::saveAcquiredPoints(const char* filename) {


	// Create a DatabaseWrapper object
	DBWrapper db(filename);
	std::string comment("# Point coordinates acquired for Registration Task");
	std::stringstream content("");
	Eigen::MatrixXd extPoints;

	std::cout << "Before saving registration points ... " << std::endl;
	std::cout << this->extPts << std::endl;

	// Get the transformation to save
	extPoints = this->extPts;

	// Fill the content of the Transformation file
	content << comment << std::endl;
	for (int i = 0; i < extPoints.rows(); i++) {
		for (int j = 0; j < extPoints.cols(); j++) {
			content << extPoints(i, j) << (j == extPoints.cols() - 1 ? ";\n" : ",");
		}
	}

	// Save the transformation
	db.write(content);
}


/**
* @brief Load function
* Load the acquired points on the requested file
* @param filename the name of the file on which save the acquired points
* @return the eset of external point coordinates
*/
Eigen::MatrixXd AbdomenPhantom::loadAcquiredPoints(const char* filename) {


	DBWrapper db(filename);
	std::vector < std::string > content;

	// Read the file
	content = db.read();

	// Extract the required data
	Eigen::MatrixXd extPoints;
	std::string line;
	std::string comma(",");
	std::string semicolumn(";");
	int commaPos = -1;

	// Initialize points
	int N = content.size();

	// Extract content
	extPoints.setZero(N, SPACE_DIM);
	for (int i = 0; i < N; i++) {

		std::string val[SPACE_DIM];
		line = content[i];

		for (int j = 0; j < SPACE_DIM; j++) {
			commaPos = line.find(comma);
			val[j] = line.substr(0, commaPos);
			line = line.substr(commaPos + 1, line.length());

			// Update the points structure
			extPoints(i, j) = std::stod(val[j]);
		}

	}


	// Set the extPoints
	this->extPts = extPoints;

	// Return 
	return extPoints;
}