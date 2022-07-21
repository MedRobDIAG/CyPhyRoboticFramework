#ifndef DATASET_HPP_
#define DATASET_HPP_

// Standard Header files
#include <vector>
#include <mutex>

// Eigen Header files
#include <Eigen\Dense>

// Project Header files
#include "utils.hpp"

/**
* The Singleton class defines the `GetInstance` method that serves as an
* alternative to constructor and lets clients access the same instance of this
* class over and over.
*/
class Dataset {


protected:

	/*
	* @brief Constructor of the Dataset class with log number argument
	* @param logNum: the number of the log to be loaded in the Dataset instance
	*/
	Dataset(const int& logNum) : logID(logNum), loaded(false), started(false), index(0) {}

	/**
	* @brief Destroyer of the Dataset class
	*/
	~Dataset() {}

	int logID;														//!< ID number of the log to be loaded in the Dataset object

public:


	/**
	* Singletons should not be cloneable.
	*/
	Dataset(Dataset &other) = delete;

	/**
	* Singletons should not be assignable.
	*/
	void operator=(const Dataset &) = delete;

	/**
	* This is the static method that controls the access to the singleton
	* instance. On the first run, it creates a singleton object and places it
	* into the static field. On subsequent runs, it returns the client existing
	* object stored in the static field.
	*/
	static Dataset *GetInstance(const int& value);

	/**
	* @brief Get function
	* Retrieves the offline log number of the dataset
	*/
	inline int getLogNumber() { return this->logID; }

	/**
	* @brief Load function
	* Load the dataset from the input log identifier number
	*/
	void load();

	/**
	* @brief Get function
	* Retrieves the joint position vector from the Dataset at the given index idx
	* @param idx: the dataset index
	* @return the  current joint position vector
	*/
	inline Eigen::VectorXf getJointPostionSample(const int& idx) { return this->qData[idx].bottomRows(this->qData[idx].size()-1); }

	/**
	* @brief Get function
	* Retrieves the joint velocity vector from the Dataset at the given index idx
	* @param idx: the dataset index
	* @return the  current joint velocity vector
	*/
	inline Eigen::VectorXf getJointVelocitySample(const int& idx) { return this->qdData[idx].bottomRows(this->qdData[idx].size() - 1); }

	/**
	* @brief Get function
	* Retrieves the residual vector from the Dataset at the given index idx
	* @param idx: the dataset index
	* @return the  current residual vector
	*/
	inline Eigen::VectorXf getResidualSample(const int& idx) { return this->rData[idx].bottomRows(this->rData[idx].size() - 1); }

	/**
	* @brief Get function
	* Retrieves the FRI residual vector from the Dataset at the given index idx
	* @param idx: the dataset index
	* @return the FRI current residual vector
	*/
	inline Eigen::VectorXf getFRIResidualSample(const int& idx) { return this->rFRIData[idx].bottomRows(this->rFRIData[idx].size() - 1); }

	/**
	* @brief Get function
	* Retrieves the joint toruqe vector from the Dataset at the given index idx
	* @param idx: the dataset index
	* @return the  current joint torque vector
	*/
	inline Eigen::VectorXf getJointTorqueSample(const int& idx) { return this->tauData[idx].bottomRows(this->tauData[idx].size() - 1); }

	/**
	* @brief Get function
	* Retrieves the F/T sensor measurementvector from the Dataset at the given index idx
	* @param idx: the dataset index
	* @return the  current F/T sensor measurement vector
	*/
	inline Eigen::VectorXf getFTSensorMeasSample(const int& idx) { return this->fmeasData[idx].bottomRows(this->fmeasData[idx].size() - 1); }

	/**
	* @brief Get function
	* Retrieves the state of the clutch button in the haptic interface from the Dataset at the given index idx
	* @param idx: the dataset index
	* @return the current clucth button state
	*/
	inline bool getHapticClutchSample(const int& idx) { return (bool)(this->hapticClutchData[idx](1)); }

	/**
	* @brief Get function
	* Retrieves the timestamp from the Dataset at the given index idx
	* @param idx: the dataset index
	* @return the corresponding timestamp
	*/
	inline double getTimeStamp(const int& idx) { return this->qData[idx](0); }

	/**
	* @brief Get function
	* Get the size of the dataset
	* @return the size of the dataset
	*/
	inline int getSize() { return this->size; }

	/**
	* @brief Check function
	* Check if the dataset reading has started
	* @return true if the dataset reading has started
	*/
	inline bool isStarted() { return this->started; }

	/**
	* @brief Start function
	* Start the dataset reading
	*/
	inline void startProcessing() { this->started = true; }

	/**
	* @brief Get function
	* Retrieves the current index of the dataset
	* @return the current index of the dataset
	*/
	inline int getCurrentIndex() { 
		{
			std::lock_guard<std::mutex> lock(idxMtx);
			return this->index;
		}
	}

	/**
	* @brief Index increase function
	* Increase the index variable to acquire the next dataset iteration
	*/
	inline void nextStep() { 
		{
			std::lock_guard<std::mutex> lock(idxMtx);
			this->index++;
		}
	}

	/// TEST FROM CLAUDIO MATLAB SCRIPT
	inline float getPe0etSample(const int& offIdx) { return this->pe0etData[offIdx](1); }
	inline float getVe0etSample(const int& offIdx) { return this->ve0etData[offIdx](1); }
	inline float getFe0etSample(const int& offIdx) { return this->fe0etData[offIdx](1); }
	inline float getMatlabTimeSample(const int& offIdx) { return this->pe0etData[offIdx](0); }

private:

	/**
	* The Singleton's constructor/destructor should always be private to
	* prevent direct construction/desctruction calls with the `new`/`delete`
	* operator.
	*/
	static Dataset * pinstance_;
	static std::mutex mutex_;
	static std::mutex idxMtx;
	std::mutex loadMtx;

	bool loaded;
	bool started;
	int size;
	int index;

	std::vector < Eigen::VectorXf > qData;							//!< Joint position data
	std::vector < Eigen::VectorXf > qdData;							//!< Joint velocity data
	std::vector < Eigen::VectorXf > rData;							//!< Residual data
	std::vector < Eigen::VectorXf > rFRIData;						//!< FRI Residual data
	std::vector < Eigen::VectorXf > tauData;						//!< Torque data
	std::vector < Eigen::VectorXf > fmeasData;						//!< F/T sensor measurement data
	std::vector < Eigen::VectorXf > hapticClutchData;				//!< Clutch enabled/disabled by the haptic interface

	/// TEST FROM CLAUDIO MATLAB SCRIPT
	std::vector < Eigen::VectorXf > pe0etData;
	std::vector < Eigen::VectorXf > ve0etData;
	std::vector < Eigen::VectorXf > fe0etData;

};




#endif // DATASET_HPP_
