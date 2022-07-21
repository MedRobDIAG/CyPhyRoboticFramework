#ifndef CONFIGURATION_HPP_
#define CONFIGURATION_HPP_

// System Header files
#include <vector>

// Standard Header files
#include <mutex>

// Enum for Motion modes
//enum CONTROL_MODE_ID { TELEOPERATION_MODE, AUTONOMOUS_MODE, MANUAL_GUIDANCE_MODE };
enum GT_FORCE_DATA_SOURCE { FT_SENSOR_SOURCE, FRI_RESIDUAL_SOURCE, MODEL_RESIDUAL_SOURCE };
//enum KIN_CONSTRAINT_TYPE { NO_CONSTRAINTS, LINEAR_MOTION, ANGULAR_MOTION, VIRTUAL_FIXTURE, RCM };
//enum TASK_TYPE { REGISTRATION_TASK = 1, DYN_PARAMS_IDENTIFICATION_TASK, AUGMENTED_NEEDLE_INSERTION_TASK, FRICTION_ESTIMATION_TASK };
//enum TASK_INPUT_TYPE { QUIT_INPUT, REGISTRATION_INPUT, DYN_PARAMS_IDENTIFICATION_INPUT, AUGMENTED_NEEDLE_INSERTION_INPUT, FRICTION_ESTIMATION_INPUT };

class Configuration {

public:

protected:

	/*
	* @brief Constructor of the Configuration class with log number argument
	* @param logNum: the number of the log to be loaded in the Configuration instance
	*/
	Configuration(const std::string& fname) : filename(fname){}

	/**
	* @brief Destroyer of the Configuration class
	*/
	~Configuration() {}


public:


	/**
	* Singletons should not be cloneable.
	*/
	Configuration(Configuration &other) = delete;
	
	/**
	* Singletons should not be assignable.
	*/
	void operator=(const Configuration &) = delete;

	/**
	* This is the static method that controls the access to the singleton
	* instance. On the first run, it creates a singleton object and places it
	* into the static field. On subsequent runs, it returns the client existing
	* object stored in the static field.
	*/
	inline static Configuration *GetInstance(const std::string& fname)
	{
		if (pinstance_ == nullptr)
		{
			std::lock_guard<std::mutex> lock(mutex_);
			if (pinstance_ == nullptr)
			{
				pinstance_ = new Configuration(fname);
			}
		}
		return pinstance_;
	}

	/**
	* @brief Load function
	* Load the configuration parameters from a file
	*/
	void loadFromFile();

	/**
	* @brief Save function
	* Save the configuration parameters on a file
	* @param filename the name of the file that has to the configuration parameters
	*/
	void saveToFile(const char* filename);

	/**
	* @ Parse function
	* Parse the content read by the lower layers and assign the read values to the corresponding variables
	* @param content the preliminary parsed content of the configuration file
	* @return true if data are ok
	*/
	bool parseFileContent(std::vector < std::pair < std::string, std::string > >& content);

	/**
	* @brief Get function
	* Get the control flag
	* @return the control flag
	*/
	inline int getControlFlag() { return this->controlMode; }

	/**
	* @brief Get function
	* Get the name of the keyboard shared name
	*/
	inline std::string getKbShdMemName() { return this->kbShdMemName; }

	/**
	* @brief Check function
	* Check the flag stating if the registration runs online or not
	*/
	//inline bool isRegistrationOnline() { return this->onlineRegistration; }

	/**
	* @brief Get function
	* Get the name of the file containing the point coordinates in the phantom reference frame
	* @return the name of the file
	*/
	inline std::string getPhantomPtsFilename() { return this->phantomPtsFile; }

	/**
	* @brief Set function
	* Enable the data logging through teh Logger object
	* @param the data logging flag
	*/
	inline void enableLog(const bool& enable) { this->logData = enable; }

	/**
	* @brief Get function
	* Check if the data have to be logged through the Logger object
	* @return true if data have to be logged, false otherwise
	*/
	inline bool isLogEnabled() { return this->logData ; }

	/**
	* @brief Get function
	* Check if the virtual simulator (V-REP) is requested in the current configuration
	* @return true if the virtual simulator (V-REP) is requested
	*/
	inline bool isSimulatorRequested() { return this->withSimulator; }

	/**
	* @brief Get function
	* Check if the real robot is requested in the current configuration
	* @return true if the real robot is requested
	*/
	inline bool isRealRobotRequested() { return this->withRealRobot; }

	/**
	* @brief Get function
	* Check if the real robot is requested in the current configuration
	* @return true if the real robot is requested
	*/
	inline bool isForceSensorRequested() { return this->withForceSensor; }

	/**
	* @brief Get function
	* Retrieves the IP Address of V-REP to connect remotely
	* @return the IP Address
	*/
	inline std::string getVREPIPAddress() { return this->vrepIPAddress; }

	/**
	* @brief Get function
	* Check if the needle is requested in the current configuration
	* @return true if the needle is requested
	*/
	inline bool isNeedleMounted() { return this->withEE; }

	/**
	* @brief Get function
	* Get the list of names of the V-REP objects that have to be loaded
	* @return the list of names
	*/
	inline std::vector < std::string > getVREPObjNames() { return this->vrepObjects; }

	/**
	* @brief Set function
	* Set the Name of the configuration file for the KUKARobot object
	* @param config_ the name of the configuration file
	*/
	inline void setRobotConfig(const std::string& config_) { this->robotConfig = config_; }

	/**
	* @brief Get function
	* Get the Name of the configuration file for the KUKARobot object
	* @return the name of the configuration file
	*/
	inline std::string getRobotConfig() { return this->robotConfig; }

	/**
	* @brief Set function
	* Set the Name of the configuration file for the Geomagic object
	* @param config_ the name of the configuration file
	*/
	inline void setGeomagicConfig(const std::string& config_) { this->geomagicConfig = config_; }

	/**
	* @brief Get function
	* Get the Name of the configuration file for the Geomagic object
	* @return the name of the configuration file
	*/
	inline std::string getGeomagicConfig() { return this->geomagicConfig; }

	/**
	* @brief Set function
	* Set the Name of the configuration file for the F/T Sensor object
	* @param config_ the name of the configuration file
	*/
	inline void setFTSensorConfig(const std::string& config_) { this->ftSensorConfig = config_; }

	/**
	* @brief Get function
	* Get the Name of the configuration file for the F/T Sensor object
	* @return the name of the configuration file
	*/
	inline std::string getFTSensorConfig() { return this->ftSensorConfig; }

	/**
	* @brief Set function
	* Set the Name of the configuration file for the EndEffector object
	* @param config_ the name of the configuration file
	*/
	inline void setNeedleConfig(const std::string& config_) { this->endEffectorConfig = config_; }

	/**
	* @brief Get function
	* Get the Name of the configuration file for the EndEffector object
	* @return the name of the configuration file
	*/
	inline std::string getNeedleConfig() { return this->endEffectorConfig; }

	/**
	* @brief Set function
	* Set the Name of the configuration file for the Phantom object
	* @param config_ the name of the configuration file
	*/
	inline void setPhantomConfig(const std::string& config_) { this->phantomConfig = config_; }

	/**
	* @brief Get function
	* Get the Name of the configuration file for the Phantom object
	* @return the name of the configuration file
	*/
	inline std::string getPhantomConfig() { return this->phantomConfig; }

	/**
	* @brief Get function
	* Get the residual threhshold for the manual guidance controller
	* @return the residual threhshold for the manual guidance controller
	*/
	inline float getMGResThresh() { return this->mg_resThresh; }

	/**
	* @brief Get function
	* Get the vector of gains for the manual guidance controller
	* @return the vector of gains for the manual guidance controller
	*/
	inline std::vector < float > getMGGains() { return this->mg_gains; }

	/**
	* @brief Get function
	* State if use the low-pass filter for the manual guidance controller
	* @return true if using the low-pass filter for the manual guidance controller, false otherwise
	*/
	inline bool applyFilterForManualGuidance() { return this->mg_applyFilterOnResidual; }

	/**
	* @brief Get function
	* Get the cutoff frequency of the low-passfilter for the manual guidance controller
	* @return the cutoff frequency of the low-passfilter for the manual guidance controller
	*/
	inline float getMGCutoffFreq() { return this->mg_fcut; }

	/**
	* @brief Check function
	* Check if the phantom to be loaded has been registered
	* @return true if the registration transformation has to be loaded, false otherwise
	*/
	inline bool useRegisteredPhantom() { return this->withRegisteredPhantom; }

	/**
	* @brief Get function
	* Get the Name of the file containing the registration transformation for the using phantom
	* @return the Name of the file containing the registration transformation for the using phantom
	*/
	inline std::string getRegTransfFile() { return this->regTransfFile; }

	/**
	* @brief Get function
	* Retrieves the gain for the autonomous control law
	* @return the gain
	*/
	inline float getAutonomousCtrlGain() { return this->ac_gain; }

	/**
	* @brief Get function
	* Retrieves the type of ground truth force data requested for the Remote Needle Insertion Task
	* @return the gt data type
	*/
	inline int getRNIgtForceData() { return this->rniGTForceData; }

	/**
	* @brief Get function 
	* Retrieve the forgetting factor lambda for the RLS 
	* @return the forgetting factor
	*/
	inline float getRNIlambda() { return this->rniLambda; }

	/**
	* @brief Get function
	* Retrieve the interaction model for the RLS
	* @return the interaction model
	*/
	inline int getRNIinteractionModel() { return this->rniInteractionModel; }

	/**
	* @brief Set function
	* Set the interaction model for the RLS
	* @param model: the chosen interaction model to be set
	*/
	inline void setRNIinteractionModel(const int& model) { this->rniInteractionModel = model; }

	/**
	* @brief Get function
	* Get the vector of initial variances of the covariance matrix for RLS in the Remote Needle Insertion Task
	* @return the vector of initial variances of the covariance matrix for RLS in the Remote Needle Insertion Task
	*/
	inline std::vector < float > getRNIInitVariances() { return this->rniInitVariances; }

	/**
	* @brief Get function
	* Get the singular value denoting the default variation of the error signal in the RLS algorithm
	* @return the singular value
	*/
	inline float getRNISigma0() { return this->rniSigma0; }

	/**
	* @brief Get function
	* Get the singular value denoting the default variation of the error signal in the RLS algorithm
	* @return the singular value
	*/
	inline float getRNISigma1() { return this->rniSigma1; }


	/**
	* @brief Check function
	* Check if the Gauss-Newton algorithm for Friction Estimation task has to run in batch or recursive versione
	* @return true if G-N implementation is recursive, false if it is batch
	*/
	inline bool isFrictionEstimationRecursive() { return this->fricEstRecursive; }

	/**
	* @brief Get function
	* Get the Gauss-Newton state data size
	* @return the input data size
	*/
	inline int getFricEstStateData() { return this->fricEstStateData; }

	/**
	* @brief Get function
	* Get the Gauss-Newton batch number of iterations
	* @return the batch number of iterations
	*/
	inline int getFricEstIterationsNum() { return this->fricEstIterationsNum; }

	/**
	* @brief Get function
	* Get the name of the file for friction parameters
	* @return the name of the file for friction parameters
	*/
	inline std::string getFrictionParamsFilename() { return this->FrictionParamFile; }

	/**
	* @brief Check function
	* Check if the gravity is accounted in the friction estimation task
	* @return true if the gravity is accounted in the friction estimation task
	*/
	inline bool isGravityAccountedInFrictionEstimation() { return this->estimateGravity; }

	/**
	* @brief Check function
	* Check if the friction estimation task runs online
	* @return true if the friction estimation task runs online
	*/
	//inline bool isFrictionEstimationOnline() { return this->fricEstimationOnline; }

	/**
	* @brief Get function
	* Get the number of the log folder containing the datasets for offline friction estimation task
	* @return the log number
	*/
	//inline int getFricEstOfflineLogNumber() { return this->fricEstLogDataNum; }

	/**
	* @brief Get function
	* Get the number of the log folder containing the datasets for offline needle-tissue interaction task
	* @return the log number
	*/
	//inline int getRNIOfflineLogNumber() { return this->rniLogDataNum; }

	/**
	* @brief Get function
	* Get the number of the log folder containing the offline dataset to be processed
	* @return the log number
	*/
	inline int getOfflineLogNumber() { return this->offLogDataNum; }


	/**
	* @brief Check function
	* Check if the processing (task, control , ...) runs online
	* @return true if the processing (task, control , ...) runs online
	*/
	inline bool isProcessingOnline() { return this->onlineProcessing; }

	/**
	* @brief Get function
	* Get the current force feedback source chosen for the accomplishment of the task
	* @return the force feedback source type ID (0 = F/T sensor, 1 = model-based residual, 2 = FRI residual)
	*/
	inline int getCurrentFFBSrcType() { return this->ffbType; }

	/**
	* @brief Check function
	* Check if the V-REP dynamic simulation is requested
	* @return true if the dynamic engine is requested 
	*/
	inline bool isDynSimulationRequested() { return this->useDynamicEngine; }

	/**
	* @brief Check function
	* Check if the F/T sensor is simulated by V-REP
	* @return true if the F/T sensor is simulated by V-REP
	* Warning: this requires V-REP dynamic simulation to be enabled
	*/
	inline bool isForceSensorSimulated() { return this->simulateFTSensor; }

	/**
	* @brief Get function
	* Get the type of haptic device selected in the Configuration file
	* @return the type of haptic device selected in the Configuration file
	*/
	inline std::string getHapticDeviceType() { return this->hapticDeviceType; }

	/**
	* @brief Get function
	* Get the list of haptic devices selected in the Configuration file
	* @return the list of haptic device selected in the Configuration file
	*/
	inline std::vector < std::string > getHapticDeviceList() { return this->hapticDeviceList; }

	/**
	* @brief Get function
	* Get the type of teleop device selected in the Configuration file
	* @return the type of teleop device selected in the Configuration file
	*/
	inline std::string getTeleopDeviceType() { return this->teleopDeviceType; }

	/**
	* @brief Get function
	* Get the type of haptic device selected in the Configuration file
	* @return the type of haptic device selected in the Configuration file
	*/
	inline std::string getRobotManipulatorType() { return this->robotManipulatorType; }

	/**
	* @brief Get function
	* Get the type of haptic device selected in the Configuration file
	* @return the type of haptic device selected in the Configuration file
	*/
	inline std::string getSimulatorType() { return this->simulatorType; }

	/**
	* @brief Get function
	* Get the type of trajectory to be performed by the autonomous controller
	* @return the type of trajectory 
	*/
	inline int getAutonomousTrajectoryType() { return this->trajType; }

	/**
	* @brief Get function
	* Get the type of phantom employed in the experiment
	* @return the string with the type of phantom
	*/
	inline std::string getPhantomType() { return this->phantomType; }

	/**
	* @brief Get function
	* Check if the identification algoritm used in the dynamic parameter identification task 
	* is based on the knowledge of the layer depths from the 3D model
	*/
	inline bool use3DLayerDepths() { return this->rniWith3DModelDepths; }

	/**
	* @brief Get function
	* Get the coeffcient of the lowpass filter to be used for force signal the needle-tissue identification task \
	* @return the coefficient
	*
	*/
	inline float getForceLowpassFilterCoeff() { return this->rniForceLPFcoeff; }

	/**
	* @brief Get function
	* Get the coeffcient of the lowpass filter to be used for force derivative signal the needle-tissue identification task \
	* @return the coefficient
	*
	*/
	inline float getForceDerLowpassFilterCoeff() { return this->rniForceDerLPFcoeff; }

	/**
	* @breif Get function
	* Get the threshold for the contact detection function in the needle-tissue identification task
	* @return the threshold
	*/
	inline float getForceDerContactThreshold() { return this->rniForceDerThreshold; }

	/**
	* @brief Set function
	* Set the threshold for the contact detection function in the needle-tissue identification task
	* @param thresh: the threshold to be set
	*/
	inline void setForceDerContactThreshold(const float& thresh) { this->rniForceDerThreshold = thresh; }

	/**
	* @brief Get function
	* Get the ratio of the covariance reset threshold
	* @return the required ratio
	*/
	inline float getCovResetRatio() { return this->rniCRratio; }

	/**
	* @brief Get function
	* Get the frequency of the covariance reset 
	* @return the required frequency
	*/
	inline float getRNICovResetFrequency() { return this->rniCRfrequency; }

	/**
	* @brief Get function
	* Get the size of the rupture window
	* @return the required window size
	*/
	inline int getRNIRuptureWinSize() { return this->rniRuptureWinSize; }

	/**
	* @brief Get function 
	* Get the flag stating, for the RNI task, if the a posteriori compensation is used
	* @return the flag
	*/
	inline bool rniUsePostCompensation() { return this->rniPostCompensation; }

private:

	/**
	* The Singleton's constructor/destructor should always be private to
	* prevent direct construction/desctruction calls with the `new`/`delete`
	* operator.
	*/
	static Configuration * pinstance_;
	static std::mutex mutex_;

	std::string filename;						//!< Name of the Configuration file

	/* System devices */
	std::vector < std::string > hapticDeviceList;
	std::string hapticDeviceType;
	std::string teleopDeviceType;
	std::string robotManipulatorType;
	std::string simulatorType;
	std::string phantomType;

	/* HLF parameters */
	int controlMode;							//!< Controller modality flag
	std::string kbShdMemName;					//!< Name of the shared memory of the keyboard
	bool logData;								//!< State if the data have to be logged or not

	/* Teleoperation control params */
	int ffbType;								//!< Force feedback type in Teleoperation mode (0 = F/T sensor, 1 = model-based residual, 2 = FRI residual)

	/* Autonomous control params*/
	float ac_gain;								//!< Gain for the autonomous control law
	int trajType;								//!< Type of trajectory to be performed by the autonomous Controller

	/* Manual Guidance control params*/
	float mg_resThresh;							//!< Threshold to be evaluated on the residual vector
	std::vector < float > mg_gains;				//!< Joint Gains for the manual guidance control
	bool mg_applyFilterOnResidual;				//!< State if a low-pass filter is applied on the residual vector
	float mg_fcut;								//!< Cut-off frequency of the lowpass filter

	/* Presence flags */
	bool withRealRobot;							//!< Flag stating if the current configuration considers the employment of the real robot
	bool withSimulator;							//!< Flag stating if the current configuration considers the employment of the virtual simulator
	bool withForceSensor;						//!< Flag stating if the current configuration considers the employment of the F/T sensor
	bool withEE;								//!< Flag stating if the current configuration considers the employment of the needle
	
	/* Instruments config filenames*/
	std::string robotConfig;					//!< Name of the configuration file for the KUKARobot object
	std::string geomagicConfig;					//!< Name of the configuration file for the Geomagic object
	std::string ftSensorConfig;					//!< Name of the configuration file for the F/T sensor object
	std::string endEffectorConfig;				//!< Name of the configuration file for the end-effector
	std::string phantomConfig;					//!< Name of the configuration file for the Phantom object

	/* V-REP parameters*/
	std::string vrepIPAddress;					//!< IP Address required to connect to the V-REP simulator remotely
	std::vector < std::string > vrepObjects;	//!< Set of V-REP object names required to be loaded
	bool useDynamicEngine;						//!< State if the V-REP simulation is dynamic or not
	bool simulateFTSensor;						//!< State if V-REP must simulate the F/T sensor

	/* Task parameters*/
	bool onlineProcessing;						//!< Flag stating if the Registration runs online or offline with pre-acquired dataset			
	int offLogDataNum;							// Number of the log folder containing the datasets for offline processing
	std::string phantomPtsFile;					//!< Name of the file containing the point coordinates in the phantom reference frame

	bool withRegisteredPhantom;					//!< Flag stating if the phantom to load has been registered and the transformatio is known
	std::string regTransfFile;					//!< Name of the file containing the registration transformation of the phantom

	//bool fricEstimationOnline;					//!< Set if the Friction estimation task is online
	bool estimateGravity;						//!< Set if the Friction/Gravity estimation process accounts also the gravity
	bool fricEstRecursive;						//!< Set if the taks using the Gauss-Newton minimization runs the recursive or batch implementation for Friction Estimation task
	int fricEstStateData;						//!< Size of the input data used for the implementation of the Gauss-Newton minimization algorithm
	int fricEstIterationsNum;					//!< Number of iterations of the batch version of the Gauss-Newton minimization
	//int fricEstLogDataNum;						//!< Number of the log folder containing the datasets for offline friction estimation task
	
	std::string FrictionParamFile;				//!< Name of the file containing the friction parameters

	/* Needle-Tissue inter. identification*/
	int rniGTForceData;								//!< Ground truth force data source for the Remote Needle Insertion Task
	float rniLambda;								//!< Forgetting factor for the RLS in the Remote Needle Insertion Task
	int rniInteractionModel;						//!< Interaction model for the RLS in the Remote Needle Insertion Task
	float rniSigma0;								//!< Singular value denoting the default variation of the error signal in the RLS algorithm
	float rniSigma1;								//!< Singular value denoting the abrupt variation of the error signal in the RLS algorithm
	bool rniWith3DModelDepths;						//!< Flag stating if the identification algorithm is based on nominal layer depths from 3D model
	float rniForceLPFcoeff;							//!< Coeffcient of the lowpass filter to be used for force signal the needle-tissue identification task 
	float rniForceDerLPFcoeff;						//!< Coeffcient of the lowpass filter to be used for force derivative signal in the needle-tissue identification task 
	float rniForceDerThreshold;						//!< Thresold for the needle-tissue detection in the needle-tissue identification task
	float rniCRratio;								//!< Ratio of Thresold for the needle-tissue detection in the needle-tissue identification task
	float rniCRfrequency;							//!< Frequency of the Covariance Reset (when reset is based on time basis)
	int rniRuptureWinSize;							//!< Size of the window evaluating the elastic component at rupture
	bool rniPostCompensation;						//!< Flag stating if the RNI task must compensate force with a posteriori information
	std::vector < float > rniInitVariances;			//!< Array of initial variances along the diagonal of the covariance matrix for the RLS in the Remote Needle Insertion task

};


#endif // CONFIGURATION_HPP_

