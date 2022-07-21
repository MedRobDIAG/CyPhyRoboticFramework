#ifndef KEYBOARD_PROXY_HPP_
#define KEYBOARD_PROXY_HPP_

// Project Header files
#include "UserProxy.hpp"


#ifdef _WIN32
#include <Windows.h>
#include <conio.h>
#elif unix
#endif

#define ESCAPE 27
#define SPACE_BAR 32
#define ENTER 13
#define POSE_DIM 7

enum USER_INPUTS { QUIT_INPUT = 0, REGISTRATION_INPUT, PLANNING_INPUT, NEEDLE_TISSUE_KV_ESTIMATION_INPUT, NEEDLE_TISSUE_GEROVICH_ESTIMATION_INPUT, FORCE_FEEDBACK_NEEDLE_INSERTION_INPUT, TOTAL_INPUTS};

/**
* Struct containing the status of the keyboard to share between the keyboard and the functionalities threads
*/
struct KeyboardData{

	bool keyhit;			//!< If a key is hit
	int key;				//!< value of the key

};

class KeyboardProxy : public UserProxy {


public:

	/**
	* @brief Default contructor of KeyboardProxy class
	*
	*/
	KeyboardProxy();

	/**
	* @brief Default destroyer of UserProxy class
	*
	*/
	~KeyboardProxy();

	/**
	* @brief Set function
	* Set the KeyboardData struct
	* @param kbd: the KeyboardData struct to be set
	*/
	inline void setKeyboardData(const KeyboardData& kbd) { this->kbdata = kbd; }

	/**
	* @brief Get function
	* Get the KeyboardData struct
	* @return the KeyboardData struct 
	*/
	inline KeyboardData getKeyboardData() { return this->kbdata; }

	/**
	* @brief Default init function
	*/
	void init();

	/**
	* @brief Default run function
	*/
	void run();

	/**
	* @brief Default clear function
	*/
	void clear();

	/**
	* @brief Check if a key has been hit or not
	* @return boolean value
	*/
	bool kbhit();

	/**
	* @brief Get the current char key upon pressure
	* @return the pressed key
	*/
	int getChar();

	/**
	* @brief Show function
	* Print the user menu with the available functionalities
	*
	*/
	inline void showMenu();

	/**
	* @brief Selection function
	* Let the user choose the next task to run
	*/
	int chooseTask();

	/**
	* @brief print function
	* Show the instructions on the screen that the user can choose during the experimental session
	*/
	void printInstructions();

private:

	KeyboardData kbdata;			//!< KeyboardData structure
	std::mutex kbmtx;				//!< Keyboard mutex
};


#endif // KEYBOARD_PROXY_HPP_
