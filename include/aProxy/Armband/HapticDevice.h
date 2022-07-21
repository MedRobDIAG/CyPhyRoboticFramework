#ifndef _HapticDevice_h
#define _HapticDevice_h
#define _CRT_SECURE_NO_WARNINGS

    #include <iostream>
    //#include <QtCore/QThread>
    //#include <QtCore/QtCore>
//#include "RLS.hpp"
//#include "Controller.hpp"

#include <Eigen/Dense>

#include "serial.h"

    #ifdef _WIN32
        #include <windows.h>
        #include <tchar.h>
        #include <strsafe.h>
        #include <process.h>
    #else
        #include <pthread.h> 
		
    #endif

#ifndef _WIN32
    #define Sleep(x) usleep(x * 1000.0)
#endif

    class HapticDevice 
	{
    public:
        HapticDevice();

        #ifdef _WIN32
            Serial* identifier;
        #else
            int identifier;
        #endif
        double interval;
        bool is_connected;
        int frequency; // Bracelet frequencies
        int motorCounter;

        //QElapsedTimer timer;
        //qint64 elapsedTime;
        double felapsedTime;

        int sendData;

        void stop();
        void sendSignal(int);
        void sendSignal_all(const Eigen::Vector4i& motors);
        void stopMotors();
        #ifdef _WIN32 
            void startCommunication(const wchar_t*);
        #else
            void startCommunication(const char*);
        #endif
        void closeCommunication();

        void run(const Eigen::Vector4i& motors_PWM); // this is virtual method, we must implement it in our subclass of QThread

    private:
        bool running;
    };

#endif
