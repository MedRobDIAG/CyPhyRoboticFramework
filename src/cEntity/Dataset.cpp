// Windows Header files
#include <Windows.h>

// Project Header files
#include "Dataset.hpp"
#include "DBWrapper.hpp"


/**
* The first time we call GetInstance we will lock the storage location
*      and then we make sure again that the variable is null and then we
*      set the value. RU:
*/
Dataset *Dataset::GetInstance(const int& value)
{
	if (pinstance_ == nullptr)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		if (pinstance_ == nullptr)
		{
			pinstance_ = new Dataset(value);
		}
	}
	return pinstance_;
}

/**
* @brief Load function
* Load the dataset from the input log identifier number
*/
void Dataset::load() {

	{
		std::lock_guard<std::mutex> lock(loadMtx);
		if (loaded) {
			return;
		}
		else {
			DBWrapper db;
			char buf[256];
			std::string logPath, filePath;
			std::string qFile("qmsr.txt");
			std::string qdFile("qdotcmd.txt");
			std::string rFile("modelResidual.txt");
			std::string rFRIFile("FRIResidual.txt");
			std::string tauFile("taumsr.txt");
			std::string fFile("FTmeas.txt");
			std::string hbFile("clutchState.txt");

			// Get the current program directory
			GetCurrentDirectoryA(256, buf);
			logPath = std::string(buf) + "\\LogSession_" + std::string(std::to_string(this->logID));

			// Joint positionss
			filePath = logPath + "\\KUKARobot\\" + qFile;
			this->qData = db.loadCSVLog(filePath);

			// Joint velocities
			filePath = logPath + "\\KUKARobot\\" + qdFile;
			this->qdData = db.loadCSVLog(filePath);

			// Residuals
			filePath = logPath + "\\KUKARobot\\" + rFile;
			this->rData = db.loadCSVLog(filePath);

			// FRI residuals
			filePath = logPath + "\\KUKARobot\\" + rFRIFile;
			this->rFRIData = db.loadCSVLog(filePath);

			// Joint Torques
			filePath = logPath + "\\KUKARobot\\" + tauFile;
			this->tauData = db.loadCSVLog(filePath);

			// Force
			filePath = logPath + "\\FTSensor\\" + fFile;
			this->fmeasData = db.loadCSVLog(filePath);

			// Haptic clutch button
			filePath = logPath + "\\Geomagic\\" + hbFile;
			this->hapticClutchData = db.loadCSVLog(filePath);

			// Set the size of the dataset
			this->size = this->qData.size();


			/// TEST FROM CLAUDIO MATLAB SCRIPT
			/*std::string pe0etFile("pe0et_.txt");
			std::string ve0etFile("ve0et_.txt");
			std::string fe0etFile("fe0et_.txt");

			// pe0et
			filePath = logPath + "\\RLS\\" + pe0etFile;
			this->pe0etData = db.loadCSVLog(filePath);

			// ve0et
			filePath = logPath + "\\RLS\\" + ve0etFile;
			this->ve0etData = db.loadCSVLog(filePath);

			// fe0et
			filePath = logPath + "\\RLS\\" + fe0etFile;
			this->fe0etData = db.loadCSVLog(filePath);//*/


			loaded = true;

		}
	}

}
