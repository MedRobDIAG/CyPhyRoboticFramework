// Project Header files
#include "SinTrajectory.hpp"
#include "DBWrapper.hpp"

/**
* @brief Constructor of the SinTrajectory object
* The constructor takes as argument the list of boolean values specifying on which component the sinusoidal trajectory should be defined
* @param x_axis: boolean value specifying if the sinusoidal is to be generated on the x axis
* @param y_axis: boolean value specifying if the sinusoidal is to be generated on the y axis
* @param z_axis: boolean value specifying if the sinusoidal is to be generated on the z axis
*
*/
/*SinTrajectory::SinTrajectory(const bool& x_axis, const bool& y_axis, const bool& z_axis) : Trajectory() {

	this->xyzMasks[0] = x_axis;
	this->xyzMasks[1] = y_axis;
	this->xyzMasks[2] = z_axis;

	// Load the config filename
	const char* f = ".\\config\\sinTrajectory.txt";
	this->filename = std::string(f);
	this->loadDataFromConfigFile();

}//*/


/**
* @brief Constructor of the Trajectory object with filename string parameter
* @param fname: the filename string parameter of the configuration file
*
*/
SinTrajectory::SinTrajectory(const std::string& fname) : Trajectory(fname) { 

	this->loadDataFromConfigFile();

}

/**
* @brief Destroyer of the Trajectory object
*
*/
SinTrajectory::~SinTrajectory() {}

/**
* @brief Load function
* Load the parameters of the class from the given input file
* @param filename: the name of the file containing the parameters to load
*/
void SinTrajectory::loadDataFromConfigFile() {

	// Create the structure to store the content of the file
	std::vector < std::pair < std::string, std::string > > content;

	// Create the static instance to access the file system
	DBWrapper db(this->filename.c_str());

	// Read and parse the files in the convention <comment, value>
	content = db.readLabeledFile();

	// Extrapolate data
	for (int i = 0; i < content.size(); i++) {
		std::string comment = content[i].first;
		std::string value = content[i].second;

		if (comment.find("Magnitude") != std::string::npos) {
			this->A = std::stod(value);
		}
		else if (comment.find("Frequency") != std::string::npos) {
			this->f = std::stod(value);
		}
		else if (comment.find("Shift [deg]") != std::string::npos) {
			this->shift = std::stod(value) * M_PI / 180.0;
		}
		else if (comment.find("x-axis movement") != std::string::npos) {
			this->xyzMasks[0] = (std::stod(value)) ? true : false;
		}
		else if (comment.find("y-axis movement") != std::string::npos) {
			this->xyzMasks[1] = (std::stod(value)) ? true : false;
		}
		else if (comment.find("z-axis movement") != std::string::npos) {
			this->xyzMasks[2] = (std::stod(value)) ? true : false;
		}
	}

}

/**
* @brief Evaluation function
* Evaluate the function
*/
Eigen::Vector6f SinTrajectory::eval() {

	Eigen::Vector6f fcn;
	fcn.setZero();

	for (int i = 0; i < SPACE_DIM; i++) {
		if (this->xyzMasks[i]) {
			fcn(i) =  0.5 * this->A * (1.0 - cos(2.0 * M_PI * this->f * this->t));
			fcn(i + 3) = this->A * M_PI * this->f * sin(2.0 * M_PI * this->f * this->t);
		}
		else {
			fcn(i) = 0.0;
			fcn(i + 3) = 0.0;
		}
	}

	if (this->t > (1.0 / this->f)) {
		this->accomplished = true;
	}//*/


	return fcn;

}

