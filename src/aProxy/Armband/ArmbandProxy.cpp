// Project Header files
#include "ArmbandProxy.hpp"
#include "Timer.hpp"
#include "utils.hpp"


#define M_PI        3.14159265358979323846264338327950288   /* pi             */

int Arr[2];
std::vector<HapticDevice> devices;
double previousTime = 0.00;
double currentTime = 0.00;
clock_t tStart;
bool first = true, second = false;




/**
* @brief A class to initialize the armband and connect to the bluetooth com port.
*
*/
VibBrac::VibBrac(int n) {
	devices.resize(n);
	int l_iHapticInitTrial = 1;

	// Set the COM port as seen in the device bluetooth settings
	std::string str = "COM3";
	std::wstring g_sHapticPort(str.length(), L' ');
	std::copy(str.begin(), str.end(), g_sHapticPort.begin());

	std::cout << "Starting" << std::endl;

	// Try to connect to all the armbands. n stands for number of armbands
	for (int i = 0; i<n; i++)
	{
		while (!devices.at(i).is_connected && l_iHapticInitTrial < 3)
		{
			devices.at(i).startCommunication(g_sHapticPort.c_str());
			std::wcout << "Try to connect : " << g_sHapticPort.c_str() << " " << l_iHapticInitTrial << std::endl;
			l_iHapticInitTrial++;
			Sleep(0.3);
		}

		if (l_iHapticInitTrial >= 3) {
			std::cout << "Exceeded number of trials for connecting the Armband. The device will not work." << std::endl;
		}

	}
	std::cout << "Done." << std::endl;
}

/**
* @brief Default Distructor of VibBrac class
*
*/
VibBrac::~VibBrac() {
	for (int i = 0; i<devices.size(); i++)
	{
		devices.at(i).closeCommunication();
	}
}

/**
* @brief Default contructor of ArmbandProxy class
*
*/
ArmbandProxy::ArmbandProxy() : HapticProxy() {//, HLInterface(){

	time_ = 0.0;

}

/**
* @brief Default destroyer of ArmbandProxy class
*
*/
ArmbandProxy::~ArmbandProxy() {
	
	delete this->vibs;

}

/**
* @brief Copy constructor of the ArmbandProxy class
* @param vp the ArmbandProxy
*/
ArmbandProxy::ArmbandProxy(ArmbandProxy& vp) {

}


/**
* @brief Default init function
*/
void ArmbandProxy::init() {

	// Create armband object to access it. The parameter is to select number of armbands.
	// Connect to the Armband 
	this->vibs = new VibBrac(1);

	std::cout << "Initializing new VibBrac dynamic class ..." << std::endl;

	this->hapticDOF = 6;
	this->hapticState.force.setZero(this->hapticDOF);
	this->availability(true);
	this->setRunning(true);
}



/**
* @brief Set function
* Set the feedback haptic force on the Geomagic device
* @param f: the feedback force to be set
*/
inline void ArmbandProxy::sendForce(const Eigen::VectorXf& f) {

	float fz = f(1); // TODO: Generalize this
	Eigen::Vector4i armBand_motorsi;
	armBand_motorsi.setZero();
	float elastic_offest = fz > 0 ? (fz / MAX_Elastic_force) * (MAX_Elastic_force_armband_effect - MIN_FRE_ARMBAND) + MIN_FRE_ARMBAND : 0;
	for (int i_motor = 0; i_motor <= 3; i_motor++)
	{
		armBand_motorsi(i_motor) = elastic_offest;
	}

	//std::cout << "[Armband] fz = " << armBand_motorsi.transpose() << std::endl;
	devices.at(0).run(armBand_motorsi);
}



/**
* @Send PWM to each motor's ArmBand according to the EF position, warning thrwow level frecuncy PWM  when the trajectory isno in a straigh forward  way(z-axis).
*/
Eigen::Vector4i ArmbandProxy::Trajectory_alert(Eigen::Vector3f EF_Pos, Eigen::Vector3f z_obj_line,float Elastic_force)
{
	// The EF position is arranged in x,y,z order in other words 0,1,2 indexes
	EF_Error = z_obj_line - EF_Pos;

	Eigen::Vector4f armBand_motors;

	armBand_motors.setZero();
	if (abs(EF_Error(1)) > MAX_ERROR_THRESHOLD_LOWER_BOUND)
	{
		int startOffset = 30;
		// Proportional controller for each vibrator in the armband
		if (EF_Error(1) < 0) {
			armBand_motors(2) = -EF_Error(1) - MAX_ERROR_THRESHOLD_LOWER_BOUND + ARMBAND_OFFSET;//  -MAX_ERROR_THRESHOLD_LOWER_BOUND is for the signal to change smoothly
		}
		else
		{
			armBand_motors(0) = EF_Error(1) - MAX_ERROR_THRESHOLD_LOWER_BOUND + ARMBAND_OFFSET;
		}
	}
	if (abs(EF_Error(0)) > MAX_ERROR_THRESHOLD_LOWER_BOUND)
	{
		if (EF_Error(0) < 0) {
			armBand_motors(3) = -EF_Error(0) - MAX_ERROR_THRESHOLD_LOWER_BOUND + ARMBAND_OFFSET;//  -MAX_ERROR_THRESHOLD_LOWER_BOUND is for the signal to change smoothly
		}
		else
		{
			armBand_motors(1) = EF_Error(0) - MAX_ERROR_THRESHOLD_LOWER_BOUND + ARMBAND_OFFSET;
		}
	}

	armBand_motors = armBand_motors / MAX_ERROR_THRESHOLD_UPPER_BOUND*MAX_FRE_ARMBAND;
	Eigen::Vector4i armBand_motorsi = armBand_motors.cast <int>();
	//armBand_motorsi.setZero();

	// std::cout << "EF:	"<< EF_Pos.transpose() << "	Error:	" << EF_Error.transpose() << "	PWM:	" << armBand_motorsi.transpose() << std::endl;

	// Sending data to the armband
	int armBand_motorsi0[] = {armBand_motors(0), armBand_motors(1), armBand_motors(2), armBand_motors(3)};

	Eigen::Vector4i armBand_motorsi2=this->Cutting_force2motors(armBand_motorsi0, Elastic_force);
	devices.at(0).run(armBand_motorsi2);
	// std::cout << "Elastic_Force:	"<< Elastic_force << "	Motors PWM:	" << armBand_motorsi2.transpose()  << std::endl;
	return armBand_motorsi;

}

/**
* @Converts the pure forces into PWM signalas to be sent until ArmBand device
*/
Eigen::Vector4i ArmbandProxy::Forces2MotorsDim(Eigen::Vector6f _ForcesAndTorques) {

	//the forces are arranged in the following _ForcesAndTorques[0:3]=Fx,Fy,Fz
	Eigen::Vector4f armBand_motors;
	armBand_motors.setZero();
	//int temp = 150;
	//armBand_motors.get();
	//std::cout << _ForcesAndTorques << std::endl;

	// Proportional controller for each vibrator in the armband based on feedback
	if (_ForcesAndTorques(1)<0) {
		armBand_motors(2) = -_ForcesAndTorques(0);
	}
	else
	{
		armBand_motors(0) =  _ForcesAndTorques(0);
	}
	if (_ForcesAndTorques(2)<0) {
		armBand_motors(3) =  -_ForcesAndTorques(1);
	}
	else
	{
		armBand_motors(1) =  _ForcesAndTorques(1);
	}
	armBand_motors=armBand_motors / MAX_FORCE*MAX_FRE_ARMBAND;
	Eigen::Vector4i armBand_motorsi= armBand_motors.cast <int>();
	
	return armBand_motorsi;
}



/**
* @brief Default run function
*/
void ArmbandProxy::run() {


	// Time variables
	double tictoc, dt, rate, tic, toc, tac, Ts;
	Timer clock;
	clock.setRate(20.0);
	rate = clock.getRate();
	Ts = 1.0 / rate;

	// Wait for some seconds to show the task menu on screen correctly (TODO: find a smart way and remove this horror)
	clock.timeSleep(1.0);

	// Send the haptic force on the device
	//this->sendForce(this->hapticState.force);
	//this->running = false;

	while (this->isRunning()) {

		// Measure starting time
		tic = clock.getCurTime();

		//----------------------------------------------------------------//
		// Do stuff here... 

		//TEST
		//this->hapticState.force(1) = 0.005 * sin(time_);
		
		// Send the haptic force on the device
		this->sendForce(this->hapticState.force);

		//----------------------------------------------------------------//

		// Measure the ending time and the elapsed time
		toc = clock.getCurTime();
		tictoc = clock.elapsedTime(tic, toc);

		// Wait until Ts
		if (tictoc < Ts) {
			clock.timeSleep(Ts - tictoc);
		}

		// Measure the final time after sleep to check the actual rate of the thread
		tac = clock.getCurTime();
		dt = clock.elapsedTime(tic, tac);
		time_ += dt;
		//std::cout << "[ArmandProxy] Running rate:" << (1.0 / dt) << std::endl;

	}

	// Reset haptic force
	this->hapticState.force.setZero(this->getHapticDOF());
	this->sendForce(this->hapticState.force);

}

/**
* @brief Default clear function
*/
void ArmbandProxy::clear() {

	// Set running on false
	this->setRunning(false);

}


/**
* @adding  cutting forces into tissues effect
*/
Eigen::Vector4i ArmbandProxy::Cutting_force2motors(int ForcesAndTorques[], float Elastic_force) {

 Eigen::Vector4i armBand_motorsi;
 armBand_motorsi.setZero();
 //float elastic_offest = Elastic_force>0 ? (Elastic_force / MAX_Elastic_force)*(MAX_Elastic_force_armband_effect)+MIN_FRE_ARMBAND : 0;
 float elastic_offest = Elastic_force>0 ? (Elastic_force / MAX_Elastic_force)*(MAX_Elastic_force_armband_effect- MIN_FRE_ARMBAND)+MIN_FRE_ARMBAND : 0;
 for (int i_motor = 0; i_motor <= 3; i_motor++)
 {
	 armBand_motorsi(i_motor) = elastic_offest; //+ ForcesAndTorques[i_motor]+ elastic_offest;
 }
  
 return armBand_motorsi;
}


/**
* @brief Debug function
* Print the names of all the V-REP objects loaded for the simulation
*/
void ArmbandProxy::print_Forces_Set(Eigen::Vector6f sensorFe0et) {
	Eigen::Vector4i motor_pwm=this->Forces2MotorsDim(sensorFe0et);

	//motor_pwm=this->Cutting_force2motors(motor_pwm);
}