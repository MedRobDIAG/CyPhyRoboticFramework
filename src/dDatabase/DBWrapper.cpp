// Project Header file
#include "DBWrapper.hpp"

// System header files
#include <fstream>

#ifdef DEBUG
#include <iostream>
#endif //DEBUG

// Project Header files
#include "utils.hpp"

/**
* @brief Default constructor of the DBWrapper class
*/
DBWrapper::DBWrapper() {}


/**
* @brief Constructor of the DBWrapper class with the filename argument
*/
DBWrapper::DBWrapper(const char* file_) {

	this->file = file_;

}

/**
* @brief Default destroyer of the DBWrapper class
*/
DBWrapper::~DBWrapper() {}

/**
* @brief Open function
* Open the the given file
*/
void DBWrapper::open() {}

/**
* @brief Write function
* Write the input content on the given file
* @param filename the name of the file on which the content has to be written
* @param the content to be written on the file
* @return true if the content has been successfulyy saved on the input file
*/
bool DBWrapper::write(const std::stringstream& content) {

	std::ofstream outFile;

	outFile.open(this->file, std::ofstream::out);

	outFile << content.str();

	return true;


}

/**
* @brief Read and parse function
* Read and parse the content of the using file by separating the upper comment and the lower value
* @return a vector of strings, containing i) the keyword of the values and ii) the value itself
*/
std::vector < std::pair < std::string, std::string > > DBWrapper::readLabeledFile() {


	std::ifstream inFile;
	std::vector < std::pair < std::string, std::string > > content;
	std::pair <std::string, std::string> contentPair_i;
	std::string line;

	// Open the file
	inFile.open(this->file, std::ios_base::in);

#ifdef DEBUG
	//std::cout << "File path: " << this->file << std::endl;
#endif //DEBUG

	// lineID = 0: looking for the commment line
	// lineID = 1: looking for the value line
	int lineID = 0;

	if (!inFile) { // if cannot open the file
		std::string error;
		std::pair <std::string, std::string> errorPair;
		errorPair.first = "ERROR";
		errorPair.second = "ERROR";
		content.push_back(errorPair);
	}
	else {

		while (!inFile.eof()) {

			// Extract the current line
			std::getline(inFile, line);
			
			if (line.length() > 0) { // if not empty line ...
				if (lineID == 0 && line.find("###") != std::string::npos) { // ... first comment line
					// Store the first element of the current pair as the comment line
					contentPair_i.first = line;
					lineID = 1;
				}
				else if (lineID == 1 && line.find("#") == std::string::npos) { // ... second value line
					// Store the first element of the current pair as the comment line
					contentPair_i.second= line;
					lineID = 0;			

					content.push_back(contentPair_i);
				}
			}
		}
	}

	/*for (int i = 0; i < content.size(); i++){

	#ifdef DEBUG
		std::cout << content[i].first << std::endl;
		std::cout << content[i].second << std::endl;
#endif //DEBUG

	}//*/

	return content;


}


/**
* @brief Load function
* Load and extract data from the input log 
* @param file: the log file
* @return a structure containing the loaded data
*/
std::vector < Eigen::VectorXf > DBWrapper::loadCSVLog(const std::string& file) {

	std::vector < Eigen::VectorXf > out;
	std::vector < std::string > content;

	this->setDBFile(file.c_str());
	content = this->readCSVFile();
	for (int i = 0; i < content.size(); i++) {
		std::vector < double > vec = parseCSVLine(content[i]);
		Eigen::VectorXf q_j(vec.size());
		for (int j = 0; j < vec.size(); j++) {
			q_j(j) = (float)vec[j];
		}
		out.push_back(q_j);
	}

	return out;
}

/**
* @brief Read function
* Read the conent of the file, assuming it's writte in the CSV format
* @ return a vector of string containing the set of values of the read CSV structure
*/
std::vector < std::string > DBWrapper::readCSVFile() {

	std::vector < std::string > content;
	std::ifstream inFile;
	std::string line;
	std::string comma(",");
	std::string semicolumn(";");
	int commaPos = -1;


	// Open the file
	inFile.open(this->file, std::ios_base::in);

#ifdef DEBUG
	//std::cout << "File path: " << this->file << std::endl;
#endif //DEBUG

	if (!inFile) { // if cannot open the file
		std::cout << "Error in accessing the file " << this->file << ". Could not read the content." << std::endl;
		std::string error("ERROR");
		content.push_back(error);
	}
	else {

		while (!inFile.eof()) {

			// Extract the current line
			std::getline(inFile, line);

			if (line.length() > 0) { // if not empty line ...
				if (line.find("###") == std::string::npos) { // not comment line
					content.push_back(line);
				}
			}
		}
	}

	// Return content
	return content;
}




/**
* @brief Read function
* Read the content on the given file
* @return a pair of strings, containing i) the keyword of the values and ii) the value itself
*/
std::vector < std::string > DBWrapper::read() {
	std::ifstream inFile;
	std::vector < std::string > content;
	std::string line;

	// Open the file
	inFile.open(this->file, std::ios_base::in);

#ifdef DEBUG
	//std::cout << "File path: " << this->file << std::endl;
#endif //DEBUG


	if (!inFile) { // if cannot open the file
		std::string error;
		error = "ERROR";
		content.push_back(error);
	}
	else {
		while (!inFile.eof()) {

			// Extract the current line
			std::getline(inFile, line);

			if (line.length() > 0) { // if not empty line ...
				if (line.find("#") == std::string::npos) { // ... and not comment line

					// Store the current line in the output structure
					content.push_back(line);
				}
			}
		}
	}

#ifdef DEBUG
	//std::cout << "Content of the file: " << std::endl;
#endif //DEBUG

	/*for (int i = 0; i < content.size(); i++){

	#ifdef DEBUG
	std::cout << content[i] << std::endl;
	#endif //DEBUG

	}//*/

	return content;

}

/**
* @brief Close function
* Close the given file
*/
void DBWrapper::close() {}