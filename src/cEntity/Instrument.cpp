// Project Header files
#include "Instrument.hpp"
#include "DBWrapper.hpp"
#include <iostream> 

// STL Header files
#include <vector>

/**
* @brief Init function
* Initialize the Instrument object from the configFile
* The function assumes that configFile has been previously set
*/
void Instrument::initFromConfig() {

	// Create the structure to store the content of the file
	std::vector < std::pair < std::string, std::string > > content;

	// Create the static instance to access the file system
	DBWrapper db(this->configFile.c_str());

	// Read and parse the files in the convention <comment, value>
	content = db.readLabeledFile();

	for (int i = 0; i < content.size(); i++) {
		std::string comment = content[i].first;
		std::string value = content[i].second;

		this->loadParamsFromConfigFile(comment, value);
	}


}


/**
* @brief Save function
* Save the input stringstream object as CSV file with specified name on the given path
* @param ss the stringstream object with the file content to be saved
* @param path the path in which the file should be saved
* @param filename the name of the file
* @return true if the file has been correctly saved, false otherwise
*/
bool Instrument::saveCSVFile(const std::stringstream& ss, const char* path, const char* filename) {

	// Get the full path of the file
	std::string fullpath = std::string(path) + std::string(filename);

	// Create the Database wrapper object
	DBWrapper db(fullpath.c_str());

	// Write the input content on the given file
	db.write(ss);

	return true;

}


