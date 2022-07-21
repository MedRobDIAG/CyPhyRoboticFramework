#ifndef INSTRUMENT_HPP_
#define INSTRUMENT_HPP_

// System Header files
#include <string>

// Eigen Header files
#include <Eigen\Dense>

// Twist dimension
#ifndef TWIST_DIM
#define TWIST_DIM 6
#endif // TWIST_DIM

namespace Eigen {

	typedef Eigen::Matrix<double, TWIST_DIM, 1> Vector6d;
	typedef Eigen::Matrix<float, TWIST_DIM, 1> Vector6f;

}

__declspec(align(16)) class Instrument {

public:
	
	/**
	* @brief Default constructor of the Instrument class
	*/
	Instrument() { this->name = "NotAssigned"; }

	/**
	* @brief Constructor of the Instrument class with name argument
	* @param name_ the name of the Instrument
	*/
	Instrument(const std::string& name_) : name(name_){}

	/**
	* @brief Operator== overload function
	* Overload the operator == for the Instrument class. The comparison is done on the evaluation of the name field
	* @param rhs: The Instrument object to be compared
	*/
	inline bool operator==(const Instrument& rhs) const { return (this->name.find(rhs.getName()) != std::string::npos); }  // p1.operator==(p2)

	/**
	* @brief Default destroyer of the Instrument class
	*/
	~Instrument() {}

	/**
	* @brief Init function
	* Initialize the Instrument object from the configFile
	* The function assumes that configFile has been previously set
	*/
	void initFromConfig();

	/**
	* @brief Init function (virtual)
	* Check the input pair of comment+value to assign the corresponding parameter
	* The function assumes that configFile has been previously set
	*/
	virtual void loadParamsFromConfigFile(const std::string& comment, const std::string& value) = 0;

	/**
	* @brief Set function
	* Set the name of the Instrument object
	* @param name_ the name of the instrument to be set
	*/
	inline void setName(const std::string& name_) { this->name = name_; }

	/**
	* @brief Get function
	* Get the name of the Instrument object
	* @return the name of the instrument
	*/
	inline std::string getName() const { return this->name; }

	/**
	* @brief Set function
	* Set the name of the Instrument object
	* @param name_ the name of the instrument to be set
	*/
	inline void setConfigFile(const std::string& config_) { this->configFile = config_; }

	/**
	* @brief Get function
	* Get the name of the configuration file Instrument object
	* @return the name of the configuration file
	*/
	inline std::string getConfigFile() { return this->configFile; }

	/**
	* @brief Save function
	* Save the input stringstream object as CSV file with specified name on the given path
	* @param ss the stringstream object with the file content to be saved
	* @param path the path in which the file should be saved
	* @param filename the name of the file
	* @return true if the file has been correctly saved, false otherwise
	*/
	bool saveCSVFile(const std::stringstream& ss, const char* path, const char* filename);

	/**
	* @brief new operator ovverriding function
	*/
	void* operator new(size_t i) { return _mm_malloc(i, 16); }

	/**
	* @brief delete operator ovverriding function
	*/
	void operator delete(void* p) { _mm_free(p); }

protected:

	std::string name;				//!< Name of the Instrument
	std::string configFile;			//!< Name of the config file for the Instrument
};


#endif // 
